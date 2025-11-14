#include "extrinsics_io.hpp"

#include <fstream>
#include <iostream>

void save_extrinsics(
    const std::vector<Eigen::Matrix4f>& extrinsics,
    const std::string& path)
{
    std::ofstream ofs(path);
    if (!ofs)
    {
        std::cerr << "Failed to open " << path << " for writing extrinsics\n";
        return;
    }

    const std::size_t N = extrinsics.size();
    ofs << N << "\n";
    for (std::size_t i = 0; i < N; ++i)
    {
        const Eigen::Matrix4f& M = extrinsics[i];
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                ofs << M(r, c);
                if (!(r == 3 && c == 3)) ofs << " ";
            }
        }
        ofs << "\n";
    }

    std::cout << "Saved " << N << " extrinsic matrices to " << path << "\n";
}

bool load_extrinsics(
    std::vector<Eigen::Matrix4f>& extrinsics,
    const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs)
    {
        std::cerr << "Failed to open " << path << " for reading extrinsics\n";
        return false;
    }

    std::size_t N = 0;
    ifs >> N;
    if (!ifs || N == 0)
    {
        std::cerr << "Invalid extrinsics file header in " << path << "\n";
        return false;
    }

    extrinsics.clear();
    extrinsics.resize(N, Eigen::Matrix4f::Identity());

    for (std::size_t i = 0; i < N; ++i)
    {
        Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                ifs >> M(r, c);
                if (!ifs)
                {
                    std::cerr << "Error reading extrinsics matrix " << i
                        << " in " << path << "\n";
                    return false;
                }
            }
        }
        extrinsics[i] = M;
    }

    std::cout << "Loaded " << N << " extrinsic matrices from " << path << "\n";
    return true;
}
