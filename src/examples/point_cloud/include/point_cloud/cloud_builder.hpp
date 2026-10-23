#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace point_cloud {

class CloudBuilder : public rclcpp::Node {
public:
    CloudBuilder();

private:
    void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

}  // namespace point_cloud
