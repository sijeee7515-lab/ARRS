#include "marker_helpers.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  // Add this for cvtColor
#include <iostream>

bool detect_marker_pose(
    const cv::Mat& image,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    cv::Vec3d& rvec,
    cv::Vec3d& tvec)
{
    // Check if image is valid
    if (image.empty())
    {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return false;
    }

    // Convert BGRA to BGR if necessary (Kinect returns BGRA)
    cv::Mat image_bgr;
    if (image.channels() == 4)
    {
        cv::cvtColor(image, image_bgr, cv::COLOR_BGRA2BGR);
    }
    else
    {
        image_bgr = image.clone();
    }

    // OpenCV 4.x ArUco API
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    detector.detectMarkers(image_bgr, corners, ids);

    if (ids.empty())
    {
        std::cerr << "No ArUco markers detected!" << std::endl;
        return false;
    }

    float markerLength = 0.04f; // 4 cm marker
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, markerLength,
        cameraMatrix, distCoeffs,
        rvecs, tvecs);

    // Use first marker
    rvec = rvecs[0];
    tvec = tvecs[0];

    // Draw detected markers and axes on the BGR image
    cv::aruco::drawDetectedMarkers(image_bgr, corners, ids);
    cv::drawFrameAxes(image_bgr, cameraMatrix, distCoeffs, rvec, tvec, markerLength * 0.5f);

    cv::imshow("Detected Marker", image_bgr);
    cv::waitKey(1);

    return true;
}