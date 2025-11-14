#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>

// Align cloud2 to cloud1 using an initial R,t estimate.
// Returns a merged colored cloud (cloud1 + transformed cloud2).
// You can also extract the refined transform from the implementation if needed.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_point_clouds(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2,
    const cv::Mat& R,
    const cv::Mat& t);
