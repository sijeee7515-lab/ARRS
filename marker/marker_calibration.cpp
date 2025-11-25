#include "marker_helpers.hpp"
#include "k4a_helpers.hpp"
#include "extrinsics_io.hpp"
#include "align_point_clouds.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp> 
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>    

#include <iostream>
#include <iomanip>            
#include <sstream>
#include <string>
#include <vector>
#include <thread>

static const std::string kExtrinsicsPath = "../../../../extrinsics.txt";
static const std::string kPlyBasePath = "../../../../output_frames/output_frame_";

const int CALIB_MARKER_ID = 0;

// calibration
void run_calibration(
    std::vector<k4a::device>& devices,
    std::vector<Eigen::Matrix4f>& extrinsics,
    bool use_charuco)
{
    const uint32_t device_count = static_cast<uint32_t>(devices.size());
    extrinsics.clear();
    extrinsics.resize(device_count);
    extrinsics[0] = Eigen::Matrix4f::Identity();

    std::vector<Eigen::Matrix4f> T_cam_from_marker(device_count);

    std::cout << "\n=== CALIBRATION MODE ===\n";
    std::cout << "1. Move the board until RGB Axis lines appear.\n";
    std::cout << "2. HOLD STILL.\n";
    std::cout << "3. Press [SPACE] to capture.\n";

    for (uint32_t i = 0; i < device_count; ++i)
    {
        std::cout << "\n--- Calibrating Camera " << i << " ---\n";
        while (true)
        {
            cv::Mat color, depth;
            get_color_depth(devices[i], color, depth);
            if (color.empty()) continue;

            cv::Mat camMatrix, distCoeffs;
            get_intrinsics(devices[i], camMatrix, distCoeffs);

            cv::Vec3d rvec, tvec;
            bool found = use_charuco
                ? detect_charuco_pose(color, camMatrix, distCoeffs, rvec, tvec)
                : detect_marker_pose(color, camMatrix, distCoeffs, CALIB_MARKER_ID, rvec, tvec);

            if (cv::waitKey(10) == 32) // SPACE
            {
                if (found) {
                    std::cout << "  Starting Capture in 3...";
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    std::cout << " 2...";
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    std::cout << " 1...\n";

                    get_color_depth(devices[i], color, depth);
                    if (use_charuco) found = detect_charuco_pose(color, camMatrix, distCoeffs, rvec, tvec);
                    else found = detect_marker_pose(color, camMatrix, distCoeffs, CALIB_MARKER_ID, rvec, tvec);

                    if (found) {
                        std::cout << "  -> CAPTURED STABLE FRAME!\n";
                        cv::Mat R_cv;
                        cv::Rodrigues(rvec, R_cv);
                        Eigen::Matrix3f R;
                        Eigen::Vector3f t;
                        cv::cv2eigen(R_cv, R);
                        cv::cv2eigen(cv::Mat(tvec), t);
                        T_cam_from_marker[i] = Eigen::Matrix4f::Identity();
                        T_cam_from_marker[i].block<3, 3>(0, 0) = R;
                        T_cam_from_marker[i].block<3, 1>(0, 3) = t;
                        break;
                    }
                    else {
                        std::cout << "  [!] Lost marker during countdown. Try again.\n";
                    }
                }
                else {
                    std::cout << "  [!] Cannot capture: Marker not visible.\n";
                }
            }
        }
        cv::destroyAllWindows();
    }

    const Eigen::Matrix4f& T0 = T_cam_from_marker[0];
    for (uint32_t i = 1; i < device_count; ++i) {
        extrinsics[i] = T0 * T_cam_from_marker[i].inverse();
        std::cout << "\nResult Cam " << i << " -> 0:\n" << extrinsics[i] << "\n";
    }
}

// capture & fuse
void capture_frames_and_save(
    std::vector<k4a::device>& devices,
    const std::vector<Eigen::Matrix4f>& extrinsics,
    int num_frames)
{
    const uint32_t device_count = static_cast<uint32_t>(devices.size());
    if (extrinsics.size() != device_count) throw std::runtime_error("Extrinsics mismatch");

    // helper: pre-fetch calibration structs
    std::vector<k4a::calibration> device_calibs;
    for (auto& dev : devices) device_calibs.push_back(get_calibration(dev));

    std::cout << "\n[Warmup] Flushing buffers to ensure sync...\n";
    for (int i = 0; i < 30; ++i) {
        std::vector<k4a::capture> dummy_caps;
        capture_batch(devices, dummy_caps);
    }
    std::cout << "[Warmup] Done. Starting Capture.\n";

    std::cout << "\n=== BATCH CAPTURE MODE (" << num_frames << " frames) ===\n";

    for (int frame = 0; frame < num_frames; ++frame)
    {
        std::cout << "Frame " << (frame + 1) << " / " << num_frames << "\n";

        std::cout << "  Triggering sync capture...\n";
        std::vector<k4a::capture> captures;
        if (!capture_batch(devices, captures)) {
            std::cerr << "  [!] Sync capture failed (timeout). Skipping frame.\n";
            continue;
        }

        std::cout << "  Processing point clouds...\n";

        auto fused_cloud = frame_to_cloud(captures[0], device_calibs[0]);
        if (!fused_cloud || fused_cloud->points.empty()) {
            std::cerr << "  [!] Empty cloud from Master. Skipping.\n";
            continue;
        }

        for (uint32_t i = 1; i < device_count; ++i)
        {
            auto cloud_i = frame_to_cloud(captures[i], device_calibs[i]);
            if (cloud_i && !cloud_i->points.empty()) {
                Eigen::Matrix3f R_eig = extrinsics[i].block<3, 3>(0, 0);
                Eigen::Vector3f t_eig = extrinsics[i].block<3, 1>(0, 3);
                cv::Mat R_cv, t_cv;
                cv::eigen2cv(R_eig, R_cv);
                cv::eigen2cv(t_eig, t_cv);

                std::cout << "  Aligning Cam " << i << " (" << cloud_i->points.size() << " pts)...\n";
                fused_cloud = align_point_clouds(fused_cloud, cloud_i, R_cv, t_cv);
            }
        }

        std::ostringstream oss;
        oss << kPlyBasePath << std::setfill('0') << std::setw(4) << (frame + 1) << ".ply";
        pcl::io::savePLYFileBinary(oss.str(), *fused_cloud);
        std::cout << "  Saved: " << oss.str() << "\n";
    }
}

int main(int argc, char** argv)
{
    try {
        if (argc < 2) {
            std::cerr << "Usage:\n"
                << "  marker.exe calib <serial>        (Single)\n"
                << "  marker.exe calib_board <serial>  (ChArUco)\n"
                << "  marker.exe capture <frames>\n";
            return 1;
        }

        std::string mode = argv[1];
        std::string master_serial = (argc >= 3 && mode.find("calib") != std::string::npos) ? argv[2] : "";

        uint32_t device_count = k4a_device_get_installed_count();
        if (device_count == 0) return -1;

        std::vector<k4a::device> devices;
        for (uint32_t i = 0; i < device_count; ++i) devices.push_back(open_device(i));

        // start devices with sync
        for (uint32_t i = 0; i < device_count; ++i) {
            bool is_master = (master_serial.empty() && i == 0) || (get_serial(devices[i]) == master_serial);
            int delay = is_master ? 0 : (i + 1) * 160;
            std::cout << "Starting " << i << " (" << get_serial(devices[i]) << ") as "
                << (is_master ? "MASTER" : "SUB") << "\n";
            start_device(devices[i], is_master, delay);
        }

        std::vector<Eigen::Matrix4f> extrinsics;
        if (mode == "calib") {
            run_calibration(devices, extrinsics, false);
            save_extrinsics(extrinsics, kExtrinsicsPath);
            std::cout << "Extrinsics saved to " << kExtrinsicsPath << "\n";
        }
        else if (mode == "calib_board") {
            run_calibration(devices, extrinsics, true);
            save_extrinsics(extrinsics, kExtrinsicsPath);
            std::cout << "Extrinsics saved to " << kExtrinsicsPath << "\n";
        }
        else if (mode == "capture") {
            if (load_extrinsics(extrinsics, kExtrinsicsPath)) {
                int frames = (argc >= 3) ? std::stoi(argv[2]) : 1;
                capture_frames_and_save(devices, extrinsics, frames);
            }
        }

        for (auto& dev : devices) dev.close();
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return -1;
    }
}