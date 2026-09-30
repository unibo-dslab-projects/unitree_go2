#pragma once

#include "point_cloud/point_cloud.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace point_cloud {

// Throws std::runtime_error when the message lacks one of the six expected fields.
PointCloud from_message(const sensor_msgs::msg::PointCloud2 &message);

sensor_msgs::msg::PointCloud2 to_message(const PointCloud &cloud);

}  // namespace point_cloud
