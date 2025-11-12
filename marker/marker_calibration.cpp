#include "marker_helpers.hpp"
#include "k4a_helpers.hpp"
#include "align_point_clouds.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <filesystem>

int main()
{
    try
    {
        uint32_t device_count = k4a_device_get_installed_count();
        if (device_count == 0)
        {
            std::cerr << "No Azure Kinect devices found." << std::endl;
            return -1;
        }
        std::cout << "Found " << device_count << " Kinect device(s)." << std::endl;

        std::vector<k4a::device> devices;
        for (uint32_t i = 0; i < device_count; ++i)
            devices.push_back(open_kinect(i));

        // Change to PointXYZRGB for colored point clouds
        std::vector<cv::Mat> rotations(device_count);
        std::vector<cv::Mat> translations(device_count);
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds(device_count);

        // Capture and detect cameras
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

            // Generate colored point cloud
            clouds[i] = get_point_cloud(devices[i]);
            std::cout << "Pose estimated for camera " << i << std::endl;
        }

        cv::Mat R_ref = rotations[0];
        cv::Mat t_ref = translations[0];
        std::string out_dir = "../data/extrinsics/";
        std::filesystem::create_directories(out_dir);

        // Create visualizer and show camera 0 first
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Multi-Camera Calibration"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(0.1);

        // Add camera 0 with specific color
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb0(clouds[0]);
        viewer->addPointCloud<pcl::PointXYZRGB>(clouds[0], rgb0, "cloud0");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud0");
        viewer->spinOnce(1000);  // Show for 1 second

        // Align all cameras to camera 0
        for (uint32_t i = 1; i < device_count; ++i)
        {
            if (rotations[i].empty() || translations[i].empty() || !clouds[i])
                continue;

            std::cout << "\n[Aligning Camera " << i << " to Camera 0]" << std::endl;

            // Compute initial transform from cam_i to cam0
            cv::Mat R_i0 = rotations[i] * R_ref.t();
            cv::Mat t_i0 = translations[i] - R_i0 * t_ref;

            // Transform cloud i to camera 0's coordinate system
            Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity();
            Eigen::Matrix3f r;
            Eigen::Vector3f trans;
            cv::cv2eigen(R_i0, r);
            cv::cv2eigen(t_i0, trans);
            init_transform.block<3, 3>(0, 0) = r;
            init_transform.block<3, 1>(0, 3) = trans;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*clouds[i], *transformed, init_transform);

            // Add transformed cloud with different name
            std::string cloud_name = "cloud" + std::to_string(i);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(transformed);
            viewer->addPointCloud<pcl::PointXYZRGB>(transformed, rgb, cloud_name);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);

            // Merge for final result
            *clouds[0] += *transformed;

            std::cout << "Camera " << i << " aligned and displayed\n" << std::endl;
            viewer->spinOnce(1000);  // Update view
        }

        // Close camera devices
        for (auto& dev : devices)
            dev.close();

        // Save merged point cloud
        std::string pcd_file = out_dir + "merged_cloud.pcd";
        pcl::io::savePCDFileBinary(pcd_file, *clouds[0]);
        std::cout << "\nSaved merged point cloud to: " << pcd_file << std::endl;

        std::cout << "\nAll calibrations complete!" << std::endl;
        std::cout << "Showing visualization - use mouse to rotate, scroll to zoom" << std::endl;
        std::cout << "Press 'q' in the visualization window to exit" << std::endl;

        viewer->spin(); // Keep window open
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}