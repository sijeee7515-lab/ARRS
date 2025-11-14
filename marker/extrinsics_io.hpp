#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

void save_extrinsics(
    const std::vector<Eigen::Matrix4f>& extrinsics,
    const std::string& path);

bool load_extrinsics(
    std::vector<Eigen::Matrix4f>& extrinsics,
    const std::string& path);
