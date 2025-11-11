#include "k4a_helpers.hpp"
#include <iostream>

k4a::device open_kinect(int index)
{
    k4a::device device = k4a::device::open(index);
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;
    device.start_cameras(&config);
    return device;
}

void get_color_depth(k4a::device &device, cv::Mat &color, cv::Mat &depth)
{
    k4a::capture capture;
    if (!device.get_capture(&capture, std::chrono::milliseconds(2000)))
    {
        throw std::runtime_error("Failed to get capture");
    }

    k4a::image colorImg = capture.get_color_image();
    k4a::image depthImg = capture.get_depth_image();

    color = cv::Mat(colorImg.get_height_pixels(),
                    colorImg.get_width_pixels(),
                    CV_8UC4, (void *)colorImg.get_buffer())
                .clone();

    depth = cv::Mat(depthImg.get_height_pixels(),
                    depthImg.get_width_pixels(),
                    CV_16U, (void *)depthImg.get_buffer())
                .clone();
}

void get_intrinsics(k4a::device &device, cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
{
    k4a::calibration calib = device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_1080P);
    k4a_calibration_camera_t cam = calib.color_camera_calibration;

    cameraMatrix = (cv::Mat_<double>(3, 3) << cam.intrinsics.parameters.param.fx, 0, cam.intrinsics.parameters.param.cx,
                    0, cam.intrinsics.parameters.param.fy, cam.intrinsics.parameters.param.cy,
                    0, 0, 1);

    distCoeffs = cv::Mat::zeros(1, 5, CV_64F); // Kinect uses rectified images, so typically no distortion
}

std::shared_ptr<open3d::geometry::PointCloud> get_pointcloud(k4a::device &device)
{
    k4a::capture capture;
    if (!device.get_capture(&capture, std::chrono::milliseconds(2000)))
    {
        throw std::runtime_error("Failed to capture for point cloud");
    }

    k4a::image depthImg = capture.get_depth_image();
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    // You can expand this to use Open3D::AzureKinectSensor or your existing transformation code
    return cloud;
}
