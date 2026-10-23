#pragma once

#include "go2_calibration/buffer.hpp"
#include "go2_calibration/sequence.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

namespace go2_calibration {
    // The topics both sources use. ROS parameters of the node (topics.*), so another setup can
    // rename them; the message types are the Go2's.
    struct Topics {
        std::string cloud = "/utlidar/cloud";            // sensor_msgs/PointCloud2, lidar frame
        std::string cloud_base = "/utlidar/cloud_base";  // sensor_msgs/PointCloud2, the same points, body frame
        std::string imu = "/utlidar/imu";                // sensor_msgs/Imu, the lidar IMU
        std::string body_state = "/sportmodestate";      // unitree_go/SportModeState, the body IMU
        std::string request = "/api/sport/request";      // unitree_api/Request, sport commands (live only)
        std::string phase = "/calibration/phase";        // std_msgs/String, the current phase name
    };

    // API ids of the Go2 sport service, as in Unitree's ros2_sport_client.h.
    constexpr std::int64_t kBalanceStandApiId = 1002;
    constexpr std::int64_t kStopMoveApiId = 1003;
    constexpr std::int64_t kEulerApiId = 1007;
    constexpr std::int64_t kMoveApiId = 1008;

    std::int64_t to_nanoseconds(const builtin_interfaces::msg::Time &stamp);

    Eigen::Vector3d to_eigen(const geometry_msgs::msg::Vector3 &vector);

    // The points of a cloud with their key (time, intensity, ring); points that are not finite are
    // dropped. Throws std::runtime_error if the cloud lacks one of those fields.
    std::vector<ScanPoint> to_scan_points(const sensor_msgs::msg::PointCloud2 &cloud);

    // The body gyroscope, rad/s, body frame.
    Eigen::Vector3d body_gyro(const unitree_go::msg::SportModeState &state);

    // The sport request for a command, as Unitree's SportClient builds it: Move and Euler carry
    // {"x":...,"y":...,"z":...} as JSON, the others no parameter.
    unitree_api::msg::Request to_request(const Command &command);
}
