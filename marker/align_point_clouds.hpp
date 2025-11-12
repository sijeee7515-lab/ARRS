#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_point_clouds(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2,
    const cv::Mat& R,
    const cv::Mat& t
);