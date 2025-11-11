#include "marker_helpers.hpp"
#include "k4a_helpers.hpp"
#include "align_point_clouds.hpp"
#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    try
    {
        uint32_t device_count = k4a_device_get_installed_count();
        std::vector<k4a_device_t> devices(num_devices);

        if (device_count == 0)
        {
            std::cerr << "No Azure Kinect devices found." << std::endl;
            return -1;
        }

        std::cout << "Found " << device_count << " Kinect device(s)." << std::endl;

        std::vector<k4a::device> devices;
        devices.reserve(device_count);
        for (uint32_t i = 0; i < device_count; ++i)
        {
            devices.push_back(open_kinect(i));
        }

        // Poses and point clouds
        std::vector<cv::Mat> rotations(device_count);
        std::vector<cv::Mat> translations(device_count);
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clouds(device_count);

        // Capture and Detect i cameras
        for (uint32_t i = 0; i < device_count; ++i)
        {
            std::cout << "\n[Camera " << i << "] Capturing..." << std::endl;

            cv::Mat color, depth;
            get_color_depth(devices[i], color, depth);

            cv::Mat camMatrix, distCoeffs;
            get_intrinsics(devices[i], camMatrix, distCoeffs);

            cv::Vec3d rvec, tvec;
            bool found = detect_marker_pose(color, camMatrix, distCoeffs, rvec, tvec);

            if (!found)
            {
                std::cerr << "Marker not detected for camera " << i << std::endl;
                continue;
            }

            cv::Mat R;
            cv::Rodrigues(rvec, R);
            rotations[i] = R.clone();
            translations[i] = cv::Mat(tvec).clone();

            // Generate point cloud
            clouds[i] = get_pointcloud(devices[i]);

            std::cout << "Pose estimated for camera " << i << ":\n"
                      << "R = " << R << "\n"
                      << "t = " << tvec << std::endl;
        }
        // Set the world cam reference
        cv::Mat R_ref = rotations[0];
        cv::Mat t_ref = translations[0];

        std::string out_dir = "../data/extrinsics/";
        std::filesystem::create_directories(out_dir);

        // Compute the relative transforms
        for (uint32_t i = 1; i < device_count; ++i)
        {
            if (rotations[i].empty() || translations[i].empty())
                continue;

            // Compute cam_i through cam_0
            cv::Mat R_i0 = rotations[i] * R_ref.t();
            cv::Mat t_i0 = translations[i] - R_i0 * t_ref;

            cv::Mat T_i0 = cv::Mat::eye(4, 4, CV_64F);
            R_i0.copyTo(T_i0(cv::Rect(0, 0, 3, 3)));
            t_i0.copyTo(T_i0(cv::Rect(3, 0, 1, 3)));

            std::string file = out_dir + "T_cam" + std::to_string(i) + "_cam0.yaml";
            cv::FileStorage fs(file, cv::FileStorage::WRITE);
            fs << "T_cam" + std::to_string(i) + "_cam0" << T_i0;
            fs.release();

            std::cout << "\nSaved extrinsics: " << file << std::endl;

            // ICP refinement
            if (clouds[0] && clouds[i])
            {
                std::cout << "Refining Camera " << i << " alignment via ICP..." << std::endl;
                auto refined = align_point_clouds(clouds[0], clouds[i], R_i0, t_i0);
                open3d::visualization::DrawGeometries({clouds[0], refined}, "Refined Alignment");
            }
        }

                for (auto &dev : devices)
            dev.close();

        std::cout << "\nAll calibrations complete!" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
