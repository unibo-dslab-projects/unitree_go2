#pragma once

#include "go2_calibration/calibrate.hpp"

#include <string>
#include <vector>

namespace go2_calibration {
    // Above this angle between gravity and +z, the lidar accelerometer does not share the gyroscope
    // frame (our Go2: 154 deg) and must not be used: keep go2_preprocessor's zero_acceleration.
    constexpr double kGravityToleranceDeg = 5.0;

    // One line per result and one per check (PASS or FAIL), for the terminal and for the comments
    // at the top of the parameter file.
    std::vector<std::string> describe(const Calibration &calibration);

    // ROS 2 parameter file for the node `node_name`: describe() as comments, then lidar_to_body,
    // imu_to_body, gyro_bias, gyro_leak and range_offset. `source` says where the data came from
    // (a bag or the live run), `created` when.
    std::string to_parameter_file(
        const Calibration &calibration, const std::string &node_name, const std::string &source,
        const std::string &created);
}
