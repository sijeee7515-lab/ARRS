#include "marker_helpers.hpp"
#include "k4a_helpers.hpp"
#include "align_point_clouds.hpp"

#include <opencv2/opencv.hpp>
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

        // Poses and point clouds
        std::vector<cv::Mat> rotations(device_count);
        std::vector<cv::Mat> translations(device_count);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds(device_count);

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

            // Generate point cloud
            clouds[i] = get_point_cloud(devices[i]);
            std::cout << "Pose estimated for camera " << i << std::endl;
        }

        cv::Mat R_ref = rotations[0];
        cv::Mat t_ref = translations[0];
        std::string out_dir = "../data/extrinsics/";
        std::filesystem::create_directories(out_dir);

        pcl::visualization::PCLVisualizer viewer("ICP Result");
        viewer.setBackgroundColor(0, 0, 0);
        viewer.addPointCloud(clouds[0], "cloud0");
        viewer.spinOnce(100);

        for (uint32_t i = 1; i < device_count; ++i)
        {
            if (rotations[i].empty() || translations[i].empty())
                continue;

            // Compute initial transform from cam_i to cam0
            cv::Mat R_i0 = rotations[i] * R_ref.t();
            cv::Mat t_i0 = translations[i] - R_i0 * t_ref;

            Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity();
            cv::cv2eigen(R_i0, init_transform.block<3,3>(0,0));
            cv::cv2eigen(t_i0, init_transform.block<3,1>(0,3));

            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*clouds[i], *transformed, init_transform);

            // ICP refinement
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(transformed);
            icp.setInputTarget(clouds[0]);
            icp.setMaximumIterations(50);
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);

            if (icp.hasConverged())
                std::cout << "ICP converged for Camera " << i << std::endl;
            else
                std::cerr << "ICP did NOT converge for Camera " << i << std::endl;

            // Merge aligned cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>(Final));
            *clouds[0] += *aligned;

            // Update visualizer non-blocking
            std::string name = "cloud" + std::to_string(i);
            viewer.addPointCloud(aligned, name);
            viewer.spinOnce(100);
        }

        for (auto &dev : devices)
            dev.close();

        std::cout << "\nAll calibrations complete!" << std::endl;
        viewer.spin(); // Final visualization window (blocking until closed)
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
