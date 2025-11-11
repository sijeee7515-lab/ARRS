#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

void detect_marker_pose(
    const cv::Mat &image,
    const cv::Mat &cameraMatrix,
    const cv::Mat &distCoeffs,
    cv::Vec3d &rvec,
    cv::Vec3d &tvec);
