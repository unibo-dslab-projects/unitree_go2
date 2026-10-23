// offline check of VoxelMap on a PCD file no ROS:
//  ros2 run map_builder voxel_count replays/<name> 0.05

#include "map_builder/voxel_map.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::fprintf(stderr, "Usage: voxel_count <pcd_file> <voxel_size>\n");
        return EXIT_FAILURE;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;

    if (pcl::io::loadPCDFile(argv[1], cloud) != 0) {
        std::fprintf(stderr, "Failed to load PCD file: %s\n", argv[1]);
        return EXIT_FAILURE;
    }

    std::vector<map_builder::Point> points;
    points.reserve(cloud.size());
    for (const auto& point : cloud.points) {
        points.emplace_back(map_builder::Point{point.x, point.y, point.z, point.intensity});
    }

    map_builder::VoxelMap map(std::stod(argv[2]));
    const std::size_t first = map.insert(points);
    const std::size_t second = map.insert(points);
    std::fprintf(stdout, "Inserted %zu points, voxels: %zu, second inserted: %zu\n", points.size(), first, second);

    return EXIT_SUCCESS;
}
