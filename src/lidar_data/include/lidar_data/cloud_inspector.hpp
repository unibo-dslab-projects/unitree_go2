#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_data {

class CloudInspector : public rclcpp::Node {
public:
    CloudInspector();

private:
    void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
};

}  // namespace lidar_data
