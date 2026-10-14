#pragma once

#include "go2_preprocessor/calibration.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_preprocessor {
    // Turns a gyroscope or accelerometer reading from the IMU frame (mounted
    // upside down and tilted 15.1 deg) into the body frame (x forward, y left, z up).
    Eigen::Vector3f imu_to_body(const Eigen::Vector3f &vector);

    // Turns the IMU's own orientation estimate (IMU frame to world) into the
    // body's orientation in the same world frame. A level robot gives identity.
    Eigen::Quaternionf imu_orientation_to_body(const Eigen::Quaternionf &orientation);

    // Subtracts the bias, then adds the leak computed from the corrected z.
    // Expects a reading already in the body frame.
    Eigen::Vector3f calibrate_gyro(const Eigen::Vector3f &angular_velocity, const GyroCalibration &calibration);

    // Subtracts the bias only: CMU applies no leak to the accelerometer.
    // Expects a reading already in the body frame.
    Eigen::Vector3f calibrate_accel(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &bias);
}
