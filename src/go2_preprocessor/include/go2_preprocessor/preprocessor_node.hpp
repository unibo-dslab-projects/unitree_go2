#pragma once

#include "go2_preprocessor/calibration.hpp"
#include "go2_preprocessor/imu_transform.hpp"
#include "go2_preprocessor/schemas.hpp"

#include <cstdint>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

namespace go2_preprocessor {
    // takes the Go2 lidar and body IMU data into the body frame for Point-LIO:
    // /utlidar/cloud without the robot's own points -> /utlidar/transformed_cloud,
    // the IMU of /sportmodestate calibrated -> /utlidar/transformed_imu.
    // The lidar's own IMU (/utlidar/imu) is not used: its accelerometer is synthetic.
    // Settings are read-only ROS parameters, read once at start-up.
    class PreprocessorNode : public rclcpp::Node {
    public:
        PreprocessorNode();

    private:
        void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
        void on_body_state(unitree_go::msg::SportModeState::ConstSharedPtr message);

        Box robot_body_;
        SensorCalibration calibration_;
        std::int64_t last_imu_stamp_ns_ = std::numeric_limits<std::int64_t>::min();

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

        rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr body_state_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    };
}
