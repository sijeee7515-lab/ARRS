#include "k4a_helpers.hpp"
#include <iostream>

k4a::device open_device(int index) {
    return k4a::device::open(index);
}

std::string get_serial(k4a::device& device) {
    return device.get_serialnum();
}

void start_device(k4a::device& device, bool is_master, int delay_off_master_usec) {
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true; // Strict sync

    if (is_master) {
        config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
        config.subordinate_delay_off_master_usec = 0;
    }
    else {
        config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
        config.subordinate_delay_off_master_usec = delay_off_master_usec;
    }
    device.start_cameras(&config);
}

k4a::calibration get_calibration(k4a::device& device) {
    return device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_1080P);
}

bool capture_batch(std::vector<k4a::device>& devices, std::vector<k4a::capture>& captures) {
    captures.resize(devices.size());
    bool success = true;
    for (size_t i = 0; i < devices.size(); ++i) {
        if (!devices[i].get_capture(&captures[i], std::chrono::milliseconds(5000))) {
            std::cerr << "Batch capture timeout on device " << i << "\n";
            success = false;
        }
    }
    return success;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_to_cloud(
    k4a::capture& capture,
    k4a::calibration& calib)
{
    k4a::image colorImg = capture.get_color_image();
    k4a::image depthImg = capture.get_depth_image();

    if (!colorImg.is_valid() || !depthImg.is_valid()) return nullptr;

    k4a::transformation transform(calib);
    k4a::image color_in_depth = transform.color_image_to_depth_camera(depthImg, colorImg);

    uint8_t* colorBuffer = color_in_depth.get_buffer();
    uint16_t* depthBuffer = reinterpret_cast<uint16_t*>(depthImg.get_buffer());

    int width = depthImg.get_width_pixels();
    int height = depthImg.get_height_pixels();

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud->points.reserve(width * height);

    const int step = 1; // Max Quality

    for (int v = 0; v < height; v += step) {
        for (int u = 0; u < width; u += step) {
            int idx = v * width + u;
            uint16_t d = depthBuffer[idx];
            if (d == 0 || d > 3860) continue;

            k4a_float2_t p2d = { (float)u + 0.5f, (float)v + 0.5f };
            k4a_float3_t p3d;
            if (calib.convert_2d_to_3d(p2d, (float)d, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &p3d)) {
                pcl::PointXYZRGB p;
                p.x = p3d.xyz.x / 1000.0f;
                p.y = p3d.xyz.y / 1000.0f;
                p.z = p3d.xyz.z / 1000.0f;
                int c_idx = idx * 4;
                p.b = colorBuffer[c_idx + 0];
                p.g = colorBuffer[c_idx + 1];
                p.r = colorBuffer[c_idx + 2];
                cloud->points.push_back(p);
            }
        }
    }
    return cloud;
}

// Legacy Wrappers
void get_color_depth(k4a::device& device, cv::Mat& color, cv::Mat& depth) {
    k4a::capture capture;
    if (device.get_capture(&capture, std::chrono::milliseconds(3000))) {
        k4a::image c = capture.get_color_image();
        k4a::image d = capture.get_depth_image();
        if (c.is_valid() && d.is_valid()) {
            color = cv::Mat(c.get_height_pixels(), c.get_width_pixels(), CV_8UC4, c.get_buffer()).clone();
            depth = cv::Mat(d.get_height_pixels(), d.get_width_pixels(), CV_16U, d.get_buffer()).clone();
        }
    }
}

void get_intrinsics(k4a::device& device, cv::Mat& M, cv::Mat& D) {
    auto calib = get_calibration(device);
    auto p = calib.color_camera_calibration.intrinsics.parameters.param;
    M = cv::Mat::eye(3, 3, CV_64F);
    M.at<double>(0, 0) = p.fx; M.at<double>(1, 1) = p.fy;
    M.at<double>(0, 2) = p.cx; M.at<double>(1, 2) = p.cy;
    D = cv::Mat::zeros(8, 1, CV_64F);
    D.at<double>(0, 0) = p.k1; D.at<double>(1, 0) = p.k2;
    D.at<double>(2, 0) = p.p1; D.at<double>(3, 0) = p.p2;
    D.at<double>(4, 0) = p.k3; D.at<double>(5, 0) = p.k4;
    D.at<double>(6, 0) = p.k5; D.at<double>(7, 0) = p.k6;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(k4a::device& device)
{
    // --- Get a capture ---
    k4a::capture capture;
    if (!device.get_capture(&capture, std::chrono::milliseconds(2000)))
    {
        throw std::runtime_error("Failed to capture for point cloud");
    }

    k4a::image depthImg = capture.get_depth_image();
    k4a::image colorImg = capture.get_color_image();

    if (!depthImg || !colorImg)
    {
        throw std::runtime_error("Missing depth or color image");
    }

    // --- Get calibration + transformation handle ---
    k4a_calibration_t calib = device.get_calibration(
        K4A_DEPTH_MODE_NFOV_UNBINNED,
        K4A_COLOR_RESOLUTION_1080P);

    k4a_transformation_t transformation =
        k4a_transformation_create(&calib);

    // --- Warp color into depth camera coordinates ---
    k4a::image color_in_depth =
        k4a::image::create(
            K4A_IMAGE_FORMAT_COLOR_BGRA32,
            depthImg.get_width_pixels(),
            depthImg.get_height_pixels(),
            depthImg.get_width_pixels() * 4);

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(
        transformation,
        depthImg.handle(),
        colorImg.handle(),
        color_in_depth.handle()))
    {
        throw std::runtime_error("Failed to align color to depth camera");
    }

    // --- Convert depth map to 3D point cloud (XYZ) ---
    k4a::image pointCloudImg =
        k4a::image::create(
            K4A_IMAGE_FORMAT_CUSTOM,
            depthImg.get_width_pixels(),
            depthImg.get_height_pixels(),
            depthImg.get_width_pixels() * 3 * sizeof(int16_t));

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(
        transformation,
        depthImg.handle(),
        K4A_CALIBRATION_TYPE_DEPTH,
        pointCloudImg.handle()))
    {
        throw std::runtime_error("Failed to convert depth to point cloud");
    }

    k4a_transformation_destroy(transformation);

    // --- Build PCL Point Cloud ---
    auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud->width = depthImg.get_width_pixels();
    cloud->height = depthImg.get_height_pixels();
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    int width = depthImg.get_width_pixels();
    int height = depthImg.get_height_pixels();

    const int16_t* xyz = reinterpret_cast<const int16_t*>(pointCloudImg.get_buffer());
    const uint8_t* bgra = color_in_depth.get_buffer();

    size_t idx = 0;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x, ++idx)
        {
            pcl::PointXYZRGB& pt = cloud->points[idx];

            int16_t X = xyz[3 * idx + 0];
            int16_t Y = xyz[3 * idx + 1];
            int16_t Z = xyz[3 * idx + 2];

            // Z==0 indicates invalid depth
            if (Z == 0)
            {
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                continue;
            }

            // Convert from millimeters → meters
            pt.x = X / 1000.0f;
            pt.y = Y / 1000.0f;
            pt.z = Z / 1000.0f;

            // BGRA aligned to depth image
            const uint8_t* pix = &bgra[4 * idx];
            pt.b = pix[0];
            pt.g = pix[1];
            pt.r = pix[2];
        }
    }

    return cloud;
}
