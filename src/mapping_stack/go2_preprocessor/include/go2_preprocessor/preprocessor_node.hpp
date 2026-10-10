#pragma once

#include "go2_preprocessor/imu_transform.hpp"
#include "go2_preprocessor/schemas.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace go2_preprocessor {
    // Takes the Go2 lidar and IMU data into the body frame for Point-LIO:
    // /utlidar/cloud without the robot's own points -> /utlidar/transformed_cloud,
    // /utlidar/imu calibrated -> /utlidar/transformed_raw_imu (full) and
    // /utlidar/transformed_imu (gyroscope only).
    // Settings are read-only ROS parameters, read once at start-up.
    class PreprocessorNode : public rclcpp::Node {
    public:
        PreprocessorNode();

    private:
        void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
        void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message);

        Box robot_body_;
        GyroCalibration gyro_calibration_;
        Eigen::Vector3f accel_bias_ = Eigen::Vector3f::Zero();
        bool zero_acceleration_ = true;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    };
}
