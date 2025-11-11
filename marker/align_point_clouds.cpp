#include <open3d/Open3D.h>

std::shared_ptr<open3d::geometry::PointCloud> align_point_clouds(
    const std::shared_ptr<open3d::geometry::PointCloud> &cloud1,
    const std::shared_ptr<open3d::geometry::PointCloud> &cloud2,
    const cv::Mat &R, const cv::Mat &t)
{
    using namespace open3d;

    Eigen::Matrix4d initTransform = Eigen::Matrix4d::Identity();
    cv::cv2eigen(R, initTransform.block<3, 3>(0, 0));
    cv::cv2eigen(t, initTransform.block<3, 1>(0, 3));

    auto cloud2_copy = std::make_shared<geometry::PointCloud>(*cloud2);
    cloud2_copy->Transform(initTransform);

    auto result = pipelines::registration::RegistrationICP(
        *cloud2_copy, *cloud1, 0.02,
        Eigen::Matrix4d::Identity(),
        pipelines::registration::TransformationEstimationPointToPlane());

    auto merged = std::make_shared<geometry::PointCloud>(*cloud1);
    *merged += *cloud2_copy->Transform(result.transformation_);

    return merged;
}
