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
        // --- Initialize both cameras ---
        k4a::device cam1 = open_kinect(0);
        k4a::device cam2 = open_kinect(1);

        // --- Capture frames ---
        cv::Mat color1, depth1, color2, depth2;
        get_color_depth(cam1, color1, depth1);
        get_color_depth(cam2, color2, depth2);

        // --- Load intrinsics ---
        cv::Mat camMatrix1, distCoeffs1, camMatrix2, distCoeffs2;
        get_intrinsics(cam1, camMatrix1, distCoeffs1);
        get_intrinsics(cam2, camMatrix2, distCoeffs2);

        // --- Detect cube markers and get their poses ---
        cv::Vec3d rvec1, tvec1, rvec2, tvec2;
        detect_marker_pose(color1, camMatrix1, distCoeffs1, rvec1, tvec1);
        detect_marker_pose(color2, camMatrix2, distCoeffs2, rvec2, tvec2);

        // --- Compute cam2 → cam1 transformation ---
        cv::Mat R1, R2;
        cv::Rodrigues(rvec1, R1);
        cv::Rodrigues(rvec2, R2);

        cv::Mat R_21 = R1 * R2.t();
        cv::Mat t_21 = tvec1 - R_21 * tvec2;

        std::cout << "Rotation (cam2->cam1):\n"
                  << R_21 << std::endl;
        std::cout << "Translation (cam2->cam1):\n"
                  << t_21.t() << std::endl;

        // --- Convert to homogeneous matrix ---
        cv::Mat T_21 = cv::Mat::eye(4, 4, CV_64F);
        R_21.copyTo(T_21(cv::Rect(0, 0, 3, 3)));
        t_21.copyTo(T_21(cv::Rect(3, 0, 1, 3)));

        // --- Save to file ---
        cv::FileStorage fs("../data/extrinsics.yaml", cv::FileStorage::WRITE);
        fs << "T_cam2_cam1" << T_21;
        fs.release();

        // --- Generate point clouds and refine via ICP ---
        auto cloud1 = get_pointcloud(cam1);
        auto cloud2 = get_pointcloud(cam2);

        auto refined = align_point_clouds(cloud1, cloud2, R_21, t_21);
        open3d::visualization::DrawGeometries({refined}, "Refined Alignment");

        // --- Cleanup ---
        cam1.close();
        cam2.close();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
