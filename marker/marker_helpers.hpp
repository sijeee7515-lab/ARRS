#pragma once

#include <opencv2/core.hpp>

// Single Marker Function
bool detect_marker_pose(
    const cv::Mat& image,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    int target_id,
    cv::Vec3d& rvec,
    cv::Vec3d& tvec);

// ChArUco Board Function
bool detect_charuco_pose(
    const cv::Mat& image,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    cv::Vec3d& rvec,
    cv::Vec3d& tvec);

void export_charuco_tiled_A4();