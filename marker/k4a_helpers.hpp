#pragma once
#include <k4a/k4a.hpp>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>

// Holds raw data so we can capture fast and process later
struct RawFrame {
    cv::Mat color;
    cv::Mat depth;
};

// Core Device Management
k4a::device open_device(int index);
std::string get_serial(k4a::device& device);
void start_device(k4a::device& device, bool is_master, int delay_off_master_usec);

// Calibration & Processing
k4a::calibration get_calibration(k4a::device& device);

// SYNC: Capture from all devices instantly
bool capture_batch(std::vector<k4a::device>& devices, std::vector<k4a::capture>& captures);

// SYNC: Process a capture into a cloud (Offline)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_to_cloud(
    k4a::capture& capture,
    k4a::calibration& calib);

// Legacy Single-Shot Helpers (for Calibration mode)
void get_color_depth(k4a::device& device, cv::Mat& color, cv::Mat& depth);
void get_intrinsics(k4a::device& device, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(k4a::device& device);