#pragma once

#include "map_builder/voxel_map.hpp"
#include "map_builder/trajectory.hpp"

#include <cstdint>
#include <filesystem>
#include <vector>

namespace map_builder {
    std::filesystem::path make_run_folder(const std::filesystem::path &output_dir, const std::uint64_t first_time_ns);
    void save_pcd(const std::filesystem::path &file, const std::vector<Point> &points);
    void save_tum(const std::filesystem::path &file, const Trajectory &trajectory);
}
