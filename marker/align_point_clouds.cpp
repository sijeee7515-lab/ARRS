#include "align_point_clouds.hpp"

#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h> 
#include <pcl/filters/filter.h> 
#include <Eigen/Dense>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <iostream>

// Helper to downsample for speed
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*output);
    return output;
}

// NEW: Manually combine Cloud + Normals (Fixes build error)
pcl::PointCloud<pcl::PointNormal>::Ptr compute_normals(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    float radius)
{
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    ne.compute(*normals);

    // Manual Copy Loop (Replaces concatenateFields)
    auto cloud_with_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    cloud_with_normals->resize(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        const auto& p = cloud->points[i];
        const auto& n = normals->points[i];
        auto& pn = cloud_with_normals->points[i];

        // Copy position
        pn.x = p.x;
        pn.y = p.y;
        pn.z = p.z;

        // Copy normal
        pn.normal_x = n.normal_x;
        pn.normal_y = n.normal_y;
        pn.normal_z = n.normal_z;
        pn.curvature = n.curvature;
    }

    return cloud_with_normals;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_point_clouds(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2,
    const cv::Mat& R,
    const cv::Mat& t)
{
    std::cout << "  [Align] Cloud1: " << cloud1->points.size()
        << " pts, Cloud2: " << cloud2->points.size() << " pts\n";

    // 1. Initial Transform
    Eigen::Matrix4f initTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f r_eig;
    Eigen::Vector3f t_eig;
    cv::cv2eigen(R, r_eig);
    cv::cv2eigen(t, t_eig);
    initTransform.block<3, 3>(0, 0) = r_eig;
    initTransform.block<3, 1>(0, 3) = t_eig;

    auto cloud2_aligned = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::transformPointCloud(*cloud2, *cloud2_aligned, initTransform);

    // 2. Advanced ICP Pipeline (Coarse -> Fine Point-to-Plane)

    // --- STAGE A: Standard ICP (Coarse) ---
    {
        std::cout << "  [Stage A] Coarse Alignment (1.0m search)...\n";
        auto source = downsample(cloud2_aligned, 0.05f);
        auto target = downsample(cloud1, 0.05f);

        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(source);
        icp.setInputTarget(target);
        icp.setMaximumIterations(50);
        icp.setMaxCorrespondenceDistance(1.0f); // Big snap

        pcl::PointCloud<pcl::PointXYZRGB> unused;
        icp.align(unused);

        if (icp.hasConverged()) {
            pcl::transformPointCloud(*cloud2_aligned, *cloud2_aligned, icp.getFinalTransformation());
        }
    }

    // --- STAGE B: Point-to-Plane (Fine) ---
    {
        std::cout << "  [Stage B] Point-to-Plane Polish (5cm radius)...\n";

        // Compute normals
        auto source_normals = compute_normals(cloud2_aligned, 0.05f);
        auto target_normals = compute_normals(cloud1, 0.05f);

        // Filter NaN normals (Important for stability)
        std::vector<int> mapping;
        pcl::removeNaNNormalsFromPointCloud(*source_normals, *source_normals, mapping);
        pcl::removeNaNNormalsFromPointCloud(*target_normals, *target_normals, mapping);

        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp_plane;
        icp_plane.setInputSource(source_normals);
        icp_plane.setInputTarget(target_normals);
        icp_plane.setMaximumIterations(50);
        icp_plane.setMaxCorrespondenceDistance(0.10f); // 10cm fine tune
        icp_plane.setTransformationEpsilon(1e-8);

        pcl::PointCloud<pcl::PointNormal> unused;
        icp_plane.align(unused);

        if (icp_plane.hasConverged()) {
            std::cout << "    -> Converged (Score: " << icp_plane.getFitnessScore() << ")\n";
            pcl::transformPointCloud(*cloud2_aligned, *cloud2_aligned, icp_plane.getFinalTransformation());
        }
        else {
            std::cout << "    -> Failed to converge on planes.\n";
        }
    }

    // 3. Merge
    auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*cloud1);
    *merged += *cloud2_aligned;
    return merged;
}