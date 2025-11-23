#include "marker_helpers.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

// --- ORIGINAL SINGLE MARKER METHOD ---
bool detect_marker_pose(
    const cv::Mat& image,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    int target_id,
    cv::Vec3d& rvec,
    cv::Vec3d& tvec)
{
    if (image.empty()) return false;

    cv::Mat image_bgr;
    if (image.channels() == 4)
        cv::cvtColor(image, image_bgr, cv::COLOR_BGRA2BGR);
    else
        image_bgr = image.clone();

    cv::Mat gray;
    cv::cvtColor(image_bgr, gray, cv::COLOR_BGR2GRAY);

    // 12cm marker (Single)
    const float markerLength = 0.12f;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::makePtr<cv::aruco::Dictionary>(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000));

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::Ptr<cv::aruco::DetectorParameters> params =
        cv::makePtr<cv::aruco::DetectorParameters>();

    // Use dictionary pointer (for your OpenCV version)
    cv::aruco::detectMarkers(gray, dictionary, corners, ids, params, rejected);

    if (ids.empty()) {
        // Don't print error here, let the loop handle it
        return false;
    }

    int index = -1;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == target_id) {
            index = static_cast<int>(i);
            break;
        }
    }

    if (index == -1) return false;

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

    rvec = rvecs[index];
    tvec = tvecs[index];

    cv::aruco::drawDetectedMarkers(image_bgr, corners, ids);
    cv::drawFrameAxes(image_bgr, cameraMatrix, distCoeffs,
        rvec, tvec, markerLength * 0.5f);

    cv::imshow("Calibration View", image_bgr);
    cv::waitKey(1);

    return true;
}

// --- NEW CHARUCO BOARD METHOD ---
bool detect_charuco_pose(
    const cv::Mat& image,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    cv::Vec3d& rvec,
    cv::Vec3d& tvec)
{
    if (image.empty()) return false;

    cv::Mat image_bgr;
    if (image.channels() == 4)
        cv::cvtColor(image, image_bgr, cv::COLOR_BGRA2BGR);
    else
        image_bgr = image.clone();

    cv::Mat gray;
    cv::cvtColor(image_bgr, gray, cv::COLOR_BGR2GRAY);

    // --- A4 CONFIGURATION ---
    int squaresX = 5;
    int squaresY = 7;
    float squareLength = 0.040f; // 40mm
    float markerLength = 0.030f; // 30mm

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::makePtr<cv::aruco::Dictionary>(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000));

    // Constructor takes *dictionary (Reference)
    cv::Ptr<cv::aruco::CharucoBoard> board =
        cv::makePtr<cv::aruco::CharucoBoard>(
            cv::Size(squaresX, squaresY),
            squareLength,
            markerLength,
            *dictionary);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();

    // Sensitivity Boost
    params->minMarkerPerimeterRate = 0.01;

    // Detect Markers takes dictionary (Pointer)
    cv::aruco::detectMarkers(gray, dictionary, corners, ids, params, rejected);

    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(image_bgr, corners, ids);
    }

    if (ids.empty()) {
        cv::imshow("Calibration View", image_bgr);
        cv::waitKey(1);
        return false;
    }

    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    cv::aruco::interpolateCornersCharuco(
        corners, ids, gray, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);

    // Require at least 6 corners to prevent DLT crash
    if (charucoIds.size() < 6) {
        // Draw what we found but return false to keep scanning
        cv::imshow("Calibration View", image_bgr);
        cv::waitKey(1);
        return false;
    }

    bool valid = cv::aruco::estimatePoseCharucoBoard(
        charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);

    if (!valid) return false;

    cv::aruco::drawDetectedCornersCharuco(image_bgr, charucoCorners, charucoIds);
    cv::drawFrameAxes(image_bgr, cameraMatrix, distCoeffs, rvec, tvec, 0.15f);

    cv::imshow("Calibration View", image_bgr);
    cv::waitKey(1);

    return true;
}