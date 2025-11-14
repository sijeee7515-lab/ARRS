#include "k4a_helpers.hpp"

#include <iostream>
#include <memory>

k4a::device open_kinect(int index)
{
    k4a::device device = k4a::device::open(index);

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;

    device.start_cameras(&config);
    return device;
}

void get_color_depth(
    k4a::device& device,
    cv::Mat& color,
    cv::Mat& depth)
{
    k4a::capture capture;
    if (!device.get_capture(&capture, std::chrono::milliseconds(2000)))
    {
        throw std::runtime_error("Failed to get capture");
    }

    k4a::image colorImg = capture.get_color_image();
    k4a::image depthImg = capture.get_depth_image();

    if (!colorImg || !depthImg)
    {
        throw std::runtime_error("Invalid color or depth image");
    }

    color = cv::Mat(
        colorImg.get_height_pixels(),
        colorImg.get_width_pixels(),
        CV_8UC4,
        (void*)colorImg.get_buffer()).clone();

    depth = cv::Mat(
        depthImg.get_height_pixels(),
        depthImg.get_width_pixels(),
        CV_16U,
        (void*)depthImg.get_buffer()).clone();

    colorImg.reset();
    depthImg.reset();
    capture.reset();
}

void get_intrinsics(
    k4a::device& device,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs)
{
    k4a::calibration calib =
        device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED,
            K4A_COLOR_RESOLUTION_1080P);

    const k4a_calibration_camera_t& cam = calib.color_camera_calibration;

    cameraMatrix = (cv::Mat_<double>(3, 3) <<
        cam.intrinsics.parameters.param.fx, 0.0, cam.intrinsics.parameters.param.cx,
        0.0, cam.intrinsics.parameters.param.fy, cam.intrinsics.parameters.param.cy,
        0.0, 0.0, 1.0);

    // We use zero distortion in OpenCV for ArUco detection.
    distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
}

// Build a colored point cloud expressed in the COLOR camera frame
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(
    k4a::device& device)
{
    k4a::capture capture;
    if (!device.get_capture(&capture, std::chrono::milliseconds(2000)))
    {
        throw std::runtime_error("Failed to capture for point cloud");
    }

    k4a::image depthImg = capture.get_depth_image();
    k4a::image colorImg = capture.get_color_image();

    if (!depthImg || !colorImg)
    {
        throw std::runtime_error("Invalid depth or color image");
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    const int depth_width = depthImg.get_width_pixels();
    const int depth_height = depthImg.get_height_pixels();

    uint16_t* depthBuffer =
        reinterpret_cast<uint16_t*>(depthImg.get_buffer());

    // Get calibration & transformation
    k4a::calibration calib =
        device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED,
            K4A_COLOR_RESOLUTION_1080P);

    k4a::transformation transform(calib);

    // Align color to depth so each depth pixel has a color
    k4a::image color_in_depth =
        transform.color_image_to_depth_camera(depthImg, colorImg);
    uint8_t* colorBuffer = color_in_depth.get_buffer();

    for (int v = 0; v < depth_height; ++v)
    {
        for (int u = 0; u < depth_width; ++u)
        {
            const int idx = v * depth_width + u;
            uint16_t d = depthBuffer[idx];
            if (d == 0)
                continue;

            k4a_float2_t p2d;
            p2d.xy.x = static_cast<float>(u) + 0.5f;
            p2d.xy.y = static_cast<float>(v) + 0.5f;

            k4a_float3_t p3d;
            bool ok = calib.convert_2d_to_3d(
                p2d,
                static_cast<float>(d),           // depth in mm
                K4A_CALIBRATION_TYPE_DEPTH,      // source: depth cam
                K4A_CALIBRATION_TYPE_COLOR,      // target: color cam
                &p3d);

            if (!ok)
                continue;

            pcl::PointXYZRGB point;
            point.x = p3d.xyz.x / 1000.0f; // mm -> m
            point.y = p3d.xyz.y / 1000.0f;
            point.z = p3d.xyz.z / 1000.0f;

            const int cidx = idx * 4;
            point.b = colorBuffer[cidx + 0];
            point.g = colorBuffer[cidx + 1];
            point.r = colorBuffer[cidx + 2];

            cloud->points.push_back(point);
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}
