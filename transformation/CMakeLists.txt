# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

# Find required packages
find_package(k4a REQUIRED)
find_package(OpenCV REQUIRED)
find_package(k4arecord REQUIRED)

add_executable(transformation_example 
    main.cpp
    transformation_helpers.cpp
    transformation_helpers.h)

target_link_libraries(transformation_example PRIVATE
    k4a::k4a
    k4a::k4arecord
    ${OpenCV_LIBS}
)

# Include OpenCV headers (since main.cpp uses cv::FileStorage)
target_include_directories(transformation_example PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)