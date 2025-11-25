#include "align_point_clouds.hpp"

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>

#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <vector>


// downsample
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampleCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    float voxel)
{
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    if (!cloud || cloud->empty())
        return filtered;

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel, voxel, voxel);
    vg.filter(*filtered);

    return filtered;
}

// Compute normals – returns PointNormal cloud
pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    float search_radius)
{
    auto out = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    if (!cloud || cloud->empty())
        return out;

    std::cout << "      [Normals] computing for cloud size = "
        << cloud->size() << ", radius = " << search_radius << "\n";

    // Estimate normals on this cloud
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGB>()));
    ne.setRadiusSearch(search_radius);

    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    ne.compute(*normals);

    std::cout << "      [Normals] got " << normals->size() << " normals\n";

    size_t N = std::min(cloud->size(), normals->size());
    out->resize(N);

    for (size_t i = 0; i < N; i++)
    {
        const auto& p = cloud->points[i];
        const auto& n = normals->points[i];
        auto& o = out->points[i];

        o.x = p.x;
        o.y = p.y;
        o.z = p.z;
        o.normal_x = n.normal_x;
        o.normal_y = n.normal_y;
        o.normal_z = n.normal_z;
        o.curvature = n.curvature;
    }

    return out;
}

//icp
pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_point_clouds(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2,
    const cv::Mat& R,
    const cv::Mat& t)
{
    if (!cloud1 || !cloud2 || cloud1->empty() || cloud2->empty())
    {
        std::cerr << "[ALIGN] ERROR: Empty point cloud.\n";
        return cloud1;
    }

    std::cout << "  [Align] Cloud1 = " << cloud1->size()
        << " pts, Cloud2 = " << cloud2->size() << " pts\n";

    //r,t to eigen
    Eigen::Matrix4f init = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R_eig;
    Eigen::Vector3f t_eig;

    cv::cv2eigen(R, R_eig);
    cv::cv2eigen(t, t_eig);

    init.block<3, 3>(0, 0) = R_eig;
    init.block<3, 1>(0, 3) = t_eig;

    auto cloud2_init = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::transformPointCloud(*cloud2, *cloud2_init, init);

    auto ds1 = downsampleCloud(cloud1, 0.015f);
    auto ds2 = downsampleCloud(cloud2_init, 0.015f);

    if (ds1->empty() || ds2->empty())
    {
        std::cerr << "[ALIGN] ERROR: Downsampled clouds empty.\n";
        return cloud1;
    }

    // clear nans from ds
    std::vector<int> idx1, idx2;
    pcl::removeNaNFromPointCloud(*ds1, *ds1, idx1);
    pcl::removeNaNFromPointCloud(*ds2, *ds2, idx2);

    std::cout << "    ds1 after NaN removal = " << ds1->size() << "\n";
    std::cout << "    ds2 after NaN removal = " << ds2->size() << "\n";

    //point-to-point
    std::cout << "  [Stage A] Coarse ICP (10cm, 20 iters)\n";

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icpA;
    icpA.setInputSource(ds2);
    icpA.setInputTarget(ds1);
    icpA.setMaxCorrespondenceDistance(0.10f);
    icpA.setMaximumIterations(20);

    pcl::PointCloud<pcl::PointXYZRGB> tmpA;
    icpA.align(tmpA);

    if (icpA.hasConverged())
    {
        std::cout << "    -> Coarse ICP converged (score = "
            << icpA.getFitnessScore() << ")\n";
        pcl::transformPointCloud(*cloud2_init, *cloud2_init, icpA.getFinalTransformation());
        pcl::transformPointCloud(*ds2, *ds2, icpA.getFinalTransformation());
    }
    else
    {
        std::cout << "    -> Coarse ICP did NOT converge.\n";
    }

    //point-to-plane
    std::cout << "  [Stage B] Fine ICP (Point-to-Plane)\n";
    std::cout << "    ds1 size        = " << ds1->size() << "\n";
    std::cout << "    ds2 size        = " << ds2->size() << "\n";

    if (ds1->size() < 500 || ds2->size() < 500)
    {
        std::cout << "    -> Not enough points for fine ICP. Skipping.\n";
    }
    else
    {
        auto n1 = computeNormals(ds1, 0.10f);
        auto n2 = computeNormals(ds2, 0.10f);

        std::cout << "    normals n1 pre-filter = " << n1->size() << "\n";
        std::cout << "    normals n2 pre-filter = " << n2->size() << "\n";

        std::vector<int> map1, map2;
        pcl::removeNaNNormalsFromPointCloud(*n1, *n1, map1);
        pcl::removeNaNNormalsFromPointCloud(*n2, *n2, map2);

        std::cout << "    normals n1 = " << n1->size() << "\n";
        std::cout << "    normals n2 = " << n2->size() << "\n";

        if (n1->size() < 500 || n2->size() < 500)
        {
            std::cout << "    -> Not enough valid normals. Skipping fine ICP.\n";
        }
        else
        {
            pcl::IterativeClosestPointWithNormals<
                pcl::PointNormal, pcl::PointNormal> icpB;
            icpB.setInputSource(n2);
            icpB.setInputTarget(n1);
            icpB.setMaxCorrespondenceDistance(0.12f);
            icpB.setMaximumIterations(20);
            icpB.setTransformationEpsilon(1e-6);
            icpB.setEuclideanFitnessEpsilon(1e-5);

            pcl::PointCloud<pcl::PointNormal> tmpB;
            icpB.align(tmpB);

            if (icpB.hasConverged())
            {
                std::cout << "    -> Fine ICP converged (score = "
                    << icpB.getFitnessScore() << ")\n";
                //fine transform to full-res cloud2_init
                pcl::transformPointCloud(*cloud2_init,
                    *cloud2_init,
                    icpB.getFinalTransformation());
            }
            else
            {
                std::cout << "    -> Fine ICP did NOT converge.\n";
            }
        }
    }

    //merging
    auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*cloud1);
    *merged += *cloud2_init;

    return merged;
}
