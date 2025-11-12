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
        throw std::runtime_error("Failed to get capture");

    k4a::image colorImg = capture.get_color_image();
    k4a::image depthImg = capture.get_depth_image();

    color = cv::Mat(colorImg.get_height_pixels(),
                    colorImg.get_width_pixels(),
                    CV_8UC4, (void*)colorImg.get_buffer()).clone();

    depth = cv::Mat(depthImg.get_height_pixels(),
                    depthImg.get_width_pixels(),
                    CV_16U, (void*)depthImg.get_buffer()).clone();

    colorImg.reset();
    depthImg.reset();
    capture.reset();
}

void get_intrinsics(k4a::device &device, cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
{
    k4a::calibration calib = device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_1080P);
    k4a_calibration_camera_t cam = calib.color_camera_calibration;

    cameraMatrix = (cv::Mat_<double>(3, 3) << cam.intrinsics.parameters.param.fx, 0, cam.intrinsics.parameters.param.cx,
                    0, cam.intrinsics.parameters.param.fy, cam.intrinsics.parameters.param.cy,
                    0, 0, 1);
    distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_point_cloud(k4a::device &device)
{
    k4a::capture capture;
    if (!device.get_capture(&capture, std::chrono::milliseconds(2000)))
        throw std::runtime_error("Failed to capture for point cloud");

    k4a::image depthImg = capture.get_depth_image();
    if (!depthImg)
        throw std::runtime_error("Invalid depth image");

    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    int width = depthImg.get_width_pixels();
    int height = depthImg.get_height_pixels();
    uint16_t* depthBuffer = reinterpret_cast<uint16_t*>(depthImg.get_buffer());

    k4a::calibration calib = device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_1080P);
    auto cam = calib.depth_camera_calibration;

    double fx = cam.intrinsics.parameters.param.fx;
    double fy = cam.intrinsics.parameters.param.fy;
    double cx = cam.intrinsics.parameters.param.cx;
    double cy = cam.intrinsics.parameters.param.cy;

    for (int v = 0; v < height; ++v)
    {
        for (int u = 0; u < width; ++u)
        {
            uint16_t d = depthBuffer[v * width + u];
            if (d == 0) continue;

            float z = d / 1000.0f;  // mm -> meters
            float x = (u - cx) * z / fx;
            float y = (v - cy) * z / fy;

            cloud->points.emplace_back(x, y, z);
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}
