#include "marker_helpers.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

bool detect_marker_pose(
    const cv::Mat& image,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    cv::Vec3d& rvec,
    cv::Vec3d& tvec)
{
    if (image.empty())
    {
        std::cerr << "detect_marker_pose: empty image\n";
        return false;
    }

    cv::Mat image_bgr;
    if (image.channels() == 4)
        cv::cvtColor(image, image_bgr, cv::COLOR_BGRA2BGR);
    else
        image_bgr = image.clone();

    cv::Mat gray;
    cv::cvtColor(image_bgr, gray, cv::COLOR_BGR2GRAY);

    // 5cm marker → 0.05 m
    const float markerLength = 0.05f;

    // OpenCV ArUco expects Ptr<Dictionary> and Ptr<DetectorParameters>
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::makePtr<cv::aruco::Dictionary>(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    // For older OpenCV versions where DetectorParameters has no create()
    cv::Ptr<cv::aruco::DetectorParameters> params =
        cv::makePtr<cv::aruco::DetectorParameters>();

    cv::aruco::detectMarkers(gray, dictionary, corners, ids, params, rejected);

    if (ids.empty())
    {
        std::cerr << "No ArUco markers detected\n";
        return false;
    }

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        corners,
        markerLength,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs);

    // Use the first marker
    rvec = rvecs[0];
    tvec = tvecs[0];

    cv::aruco::drawDetectedMarkers(image_bgr, corners, ids);
    cv::drawFrameAxes(image_bgr, cameraMatrix, distCoeffs,
        rvec, tvec, markerLength * 0.5f);

    cv::imshow("Detected Marker", image_bgr);
    cv::waitKey(1);

    return true;
}
