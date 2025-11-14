#pragma once
#include <opencv2/core.hpp>

bool detect_marker_pose(
    const cv::Mat& image,          // BGRA or BGR
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    cv::Vec3d& rvec,
    cv::Vec3d& tvec);
