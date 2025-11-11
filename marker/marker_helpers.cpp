#include "marker_helpers.hpp"
#include <iostream>

void detect_marker_pose(
    const cv::Mat &image,
    const cv::Mat &cameraMatrix,
    const cv::Mat &distCoeffs,
    cv::Vec3d &rvec,
    cv::Vec3d &tvec)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(image, dictionary, corners, ids);

    if (ids.empty())
    {
        throw std::runtime_error("No ArUco markers detected!");
    }

    float markerLength = 0.04f; // 4 cm marker
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, markerLength,
                                         cameraMatrix, distCoeffs,
                                         rvecs, tvecs);

    // Just use first marker
    rvec = rvecs[0];
    tvec = tvecs[0];

    cv::aruco::drawDetectedMarkers(image, corners, ids);
    cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvec, tvec, 0.05f);
    cv::imshow("Detected Marker", image);
    cv::waitKey(1);
}
