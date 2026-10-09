#pragma once

#include "schemas.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace go2_preprocessor {
    PointCloud from_message(const sensor_msgs::msg::PointCloud2 &message);
    sensor_msgs::msg::PointCloud2 to_message(const PointCloud &cloud);
}
