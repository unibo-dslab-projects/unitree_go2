#pragma once

#include <rclcpp/node.hpp>

namespace lidar_data {

class CloudInspector : public rclcpp::Node {
public:
    CloudInspector() : Node("cloud_inspector") {}
};

}  // namespace lidar_data
