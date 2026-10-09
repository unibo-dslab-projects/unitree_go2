#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace go2_preprocessor {
    // Takes /utlidar/cloud into the body frame, drops the robot's own points and
    // publishes the result on /utlidar/transformed_cloud.
    class PreprocessorNode : public rclcpp::Node {
    public:
        PreprocessorNode();

    private:
        void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    };
}
