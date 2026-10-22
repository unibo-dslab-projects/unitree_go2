#pragma once

#include "map_builder/voxel_map.hpp"

#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace map_builder {
    std::vector<Point> from_message(const sensor_msgs::msg::PointCloud2 &message);
    sensor_msgs::msg::PointCloud2 to_message(const std::vector<Point> &points, const std_msgs::msg::Header &header);
}
