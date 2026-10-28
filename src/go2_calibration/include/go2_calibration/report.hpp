#pragma once

#include "go2_calibration/calibrate.hpp"

#include <string>
#include <vector>

namespace go2_calibration {
    // One line per result and one per check (PASS or FAIL), for the terminal and for the comments
    // at the top of the parameter file.
    std::vector<std::string> describe(const Calibration &calibration);

    // ROS 2 parameter file for the node `node_name`: describe() as comments, then lidar_to_body,
    // body_imu (gyro_bias, accel_scale) and range_offset. The lidar IMU results stay in the
    // comments: go2_preprocessor does not use that IMU. `source` says where the data came from
    // (a bag or the live run), `created` when.
    std::string to_parameter_file(
        const Calibration &calibration, const std::string &node_name, const std::string &source,
        const std::string &created);
}
