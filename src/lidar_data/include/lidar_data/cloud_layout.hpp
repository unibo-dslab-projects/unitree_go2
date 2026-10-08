#pragma once

#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_data {

std::string describe_layout(const sensor_msgs::msg::PointCloud2 &cloud);

}  // namespace lidar_data
