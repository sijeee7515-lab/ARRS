#include "align_point_clouds.hpp"
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr align_point_clouds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
    const cv::Mat &R, const cv::Mat &t)
{
    std::cout << "  Cloud1 size: " << cloud1->points.size() << " points" << std::endl;
    std::cout << "  Cloud2 size: " << cloud2->points.size() << " points" << std::endl;

    // Downsample clouds for faster processing
    std::cout << "  Downsampling clouds..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    float leaf_size = 0.01f; // 1cm voxel size
    
    auto cloud1_down = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    voxel_filter.setInputCloud(cloud1);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*cloud1_down);
    
    auto cloud2_down = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    voxel_filter.setInputCloud(cloud2);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*cloud2_down);
    
    std::cout << "  After downsampling - Cloud1: " << cloud1_down->points.size() 
              << ", Cloud2: " << cloud2_down->points.size() << std::endl;

    // Apply initial transformation
    std::cout << "  Applying initial transformation..." << std::endl;
    Eigen::Matrix4f initTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f r;
    Eigen::Vector3f trans;
    cv::cv2eigen(R, r);
    cv::cv2eigen(t, trans);
    initTransform.block<3,3>(0,0) = r;
    initTransform.block<3,1>(0,3) = trans;
    
    auto cloud2_transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*cloud2_down, *cloud2_transformed, initTransform);
    
    // ICP refinement
    std::cout << "  Running ICP..." << std::endl;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud2_transformed);
    icp.setInputTarget(cloud1_down);
    icp.setMaximumIterations(50);
    icp.setMaxCorrespondenceDistance(0.05);  // 5cm max distance
    icp.setTransformationEpsilon(1e-8);      // Convergence criteria
    icp.setEuclideanFitnessEpsilon(1e-6);    // Convergence criteria
    
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    
    if (icp.hasConverged())
    {
        std::cout << "  ICP converged! Fitness score: " << icp.getFitnessScore() << std::endl;
    }
    else
    {
        std::cout << "  ICP did NOT converge!" << std::endl;
    }
    
    // Merge clouds (use full resolution clouds, not downsampled)
    std::cout << "  Merging clouds..." << std::endl;
    
    // Apply the combined transformation to the full-resolution cloud2
    Eigen::Matrix4f final_transform = icp.getFinalTransformation() * initTransform;
    auto cloud2_final = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*cloud2, *cloud2_final, final_transform);
    
    auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud1);
    *merged += *cloud2_final;
    
    std::cout << "  Merged cloud size: " << merged->points.size() << " points" << std::endl;
    
    return merged;
}