cmake_minimum_required(VERSION 3.15)
project(MarkerCalibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Manually set vcpkg toolchain if needed
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

# Add subdirectories
add_subdirectory(marker)
add_subdirectory(transformation)
