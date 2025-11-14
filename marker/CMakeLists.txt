cmake_minimum_required(VERSION 3.15)
project(MultiKinectCalibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(OpenCV REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(k4a REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(marker_calibration
    marker_calibration.cpp
    k4a_helpers.cpp
    marker_helpers.cpp
    extrinsics_io.cpp
    align_point_clouds.cpp
)

target_link_libraries(marker_calibration
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    k4a::k4a
)

set_target_properties(marker_calibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
