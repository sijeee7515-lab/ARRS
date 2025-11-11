cmake_minimum_required(VERSION 3.15)
project(MarkerCalibration)

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(Open3D REQUIRED)
find_package(k4a REQUIRED)  # From Azure Kinect SDK

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${K4A_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(marker_calibration
    marker_calibration.cpp
    marker_helpers.cpp
    k4a_helpers.cpp
    align_point_clouds.cpp
)

target_link_libraries(marker_calibration
    ${OpenCV_LIBS}
    ${Open3D_LIBRARIES}
    ${K4A_LIBRARIES}
)
