#include "align_point_clouds.hpp"
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr align_point_clouds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
    const cv::Mat &R, const cv::Mat &t)
{
    Eigen::Matrix4f initTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f r;
    Eigen::Vector3f trans;
    cv::cv2eigen(R, r);
    cv::cv2eigen(t, trans);
    initTransform.block<3,3>(0,0) = r;
    initTransform.block<3,1>(0,3) = trans;

    auto cloud2_transformed = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*cloud2, *cloud2_transformed, initTransform);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud2_transformed);
    icp.setInputTarget(cloud1);
    icp.setMaximumIterations(50);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    auto merged = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud1);
    *merged += Final;

    return merged;
}
