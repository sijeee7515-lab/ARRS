#pragma once
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>

k4a::device open_kinect(int index);
void get_color_depth(k4a::device &device, cv::Mat &color, cv::Mat &depth);
void get_intrinsics(k4a::device &device, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
std::shared_ptr<open3d::geometry::PointCloud> get_pointcloud(k4a::device &device);
