cmake_minimum_required(VERSION 3.15)
project(MarkerCalibration)
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(k4a REQUIRED)  # Changed from azure-kinect-sensor-sdk

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(marker_calibration
    marker_calibration.cpp
    marker_helpers.cpp
    k4a_helpers.cpp
    align_point_clouds.cpp
)

target_link_libraries(marker_calibration
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    k4a::k4a  # Modern CMake target (handles includes automatically)
)

set_target_properties(marker_calibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)