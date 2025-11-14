#include "marker_helpers.hpp"
#include "k4a_helpers.hpp"
#include "extrinsics_io.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>    
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

// Executable lives in /marker/out/build/.../bin
// "../../../../" takes us back to /marker
static const std::string kExtrinsicsPath = "../../../../extrinsics.txt";
static const std::string kPlyBasePath = "../../../../output_frames/output_frame_";

// ---------- Calibration ----------

void run_calibration(
    std::vector<k4a::device>& devices,
    std::vector<Eigen::Matrix4f>& extrinsics)
{
    const uint32_t device_count = static_cast<uint32_t>(devices.size());
    extrinsics.clear();
    extrinsics.resize(device_count);

    // cam 0 is reference
    extrinsics[0] = Eigen::Matrix4f::Identity();

    std::vector<Eigen::Matrix4f> T_cam_from_marker(device_count);

    std::cout << "\n=== Calibration: place 5cm ArUco where ALL cameras see it ===\n";

    for (uint32_t i = 0; i < device_count; ++i)
    {
        std::cout << "\n[Camera " << i << "] Capturing for calibration...\n";

        cv::Mat color, depth;
        get_color_depth(devices[i], color, depth);

        cv::Mat camMatrix, distCoeffs;
        get_intrinsics(devices[i], camMatrix, distCoeffs);

        cv::Vec3d rvec, tvec;
        if (!detect_marker_pose(color, camMatrix, distCoeffs, rvec, tvec))
        {
            throw std::runtime_error("Marker not detected on camera " +
                std::to_string(i));
        }

        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);

        Eigen::Matrix3f R;
        Eigen::Vector3f t;
        cv::cv2eigen(R_cv, R);
        cv::cv2eigen(cv::Mat(tvec), t);

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;

        T_cam_from_marker[i] = T;

        std::cout << "Camera " << i << " (camera_from_marker):\n"
            << T << "\n";
    }

    const Eigen::Matrix4f& T0 = T_cam_from_marker[0];

    for (uint32_t i = 1; i < device_count; ++i)
    {
        const Eigen::Matrix4f& Ti = T_cam_from_marker[i];

        Eigen::Matrix4f T0i = T0 * Ti.inverse();  // cam i -> cam 0
        extrinsics[i] = T0i;

        std::cout << "\nExtrinsic (cam " << i << " -> cam 0):\n"
            << T0i << "\n";
    }

    std::cout << "\nCalibration complete.\n";
}

// ---------- Capture & Fuse ----------

void capture_frames_and_save(
    std::vector<k4a::device>& devices,
    const std::vector<Eigen::Matrix4f>& extrinsics,
    int num_frames)
{
    const uint32_t device_count = static_cast<uint32_t>(devices.size());

    if (extrinsics.size() != device_count)
    {
        throw std::runtime_error("Extrinsics size does not match #devices");
    }

    std::cout << "\n=== Capture: capturing " << num_frames
        << " fused frame(s) (marker removed) ===\n";

    for (int frame = 0; frame < num_frames; ++frame)
    {
        std::cout << "\nFrame " << (frame + 1) << " / " << num_frames << "\n";

        auto fused = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        for (uint32_t i = 0; i < device_count; ++i)
        {
            std::cout << "  Capturing from cam " << i << " ...\n";
            auto cloud_i = get_point_cloud(devices[i]); // in cam i(color) frame

            pcl::PointCloud<pcl::PointXYZRGB> transformed;
            pcl::transformPointCloud(*cloud_i, transformed, extrinsics[i]);

            *fused += transformed;
        }

        std::ostringstream oss;
        oss << kPlyBasePath
            << std::setfill('0') << std::setw(4) << (frame + 1)
            << ".ply";
        const std::string ply_path = oss.str();

        std::cout << "  Saving fused cloud to " << ply_path << " ...\n";
        pcl::io::savePLYFileBinary(ply_path, *fused);
        std::cout << "  Saved.\n";
    }
}

// ---------- Main ----------

int main(int argc, char** argv)
{
    try
    {
        std::cout << "Usage:\n"
            << "  " << argv[0] << " calib\n"
            << "  " << argv[0] << " capture <num_frames>\n\n";

        if (argc < 2)
        {
            std::cerr << "Not enough arguments.\n";
            return 1;
        }

        std::string mode = argv[1];

        uint32_t device_count = k4a_device_get_installed_count();
        if (device_count == 0)
        {
            std::cerr << "No Azure Kinect devices found.\n";
            return -1;
        }

        std::cout << "Found " << device_count << " Kinect device(s).\n";

        std::vector<k4a::device> devices;
        devices.reserve(device_count);
        for (uint32_t i = 0; i < device_count; ++i)
        {
            devices.push_back(open_kinect(static_cast<int>(i)));
        }

        std::vector<Eigen::Matrix4f> extrinsics;

        if (mode == "calib")
        {
            run_calibration(devices, extrinsics);
            save_extrinsics(extrinsics, kExtrinsicsPath);
            std::cout << "Extrinsics saved to " << kExtrinsicsPath << "\n";
            std::cout << "Now run:\n  " << argv[0]
                << " capture <num_frames>\n";
        }
        else if (mode == "capture")
        {
            if (argc < 3)
            {
                std::cerr << "capture mode requires <num_frames>.\n";
                return 1;
            }

            int num_frames = std::stoi(argv[2]);
            if (num_frames <= 0)
            {
                std::cerr << "num_frames must be positive.\n";
                return 1;
            }

            if (!load_extrinsics(extrinsics, kExtrinsicsPath))
            {
                std::cerr << "Failed to load extrinsics from "
                    << kExtrinsicsPath << "\n";
                return 1;
            }

            if (extrinsics.size() != device_count)
            {
                std::cerr << "Extrinsics count (" << extrinsics.size()
                    << ") does not match devices (" << device_count
                    << ")\n";
                return 1;
            }

            capture_frames_and_save(devices, extrinsics, num_frames);
        }
        else
        {
            std::cerr << "Unknown mode: " << mode << "\n";
            return 1;
        }

        for (auto& dev : devices)
        {
            dev.close();
        }

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return -1;
    }
}
