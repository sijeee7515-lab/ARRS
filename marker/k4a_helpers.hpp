#pragma once
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

k4a::device open_kinect(int index);
void get_color_depth(k4a::device &device, cv::Mat &color, cv::Mat &depth);
void get_intrinsics(k4a::device &device, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
pcl::PointCloud<pcl::PointXYZ>::Ptr get_pointcloud(k4a::device &device);
