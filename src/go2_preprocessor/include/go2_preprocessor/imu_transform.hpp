#pragma once

#include "go2_preprocessor/calibration.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_preprocessor {
    // turns a gyroscope or accelerometer reading from the IMU frame into the body
    // frame (x forward, y left, z up). `rotation` is SensorCalibration::imu_to_body.
    Eigen::Vector3f imu_to_body(const Eigen::Vector3f &vector, const Eigen::Matrix3f &rotation);

    // turns the IMU's own orientation estimate (IMU frame to world) into the
    // body's orientation in the same world frame. A level robot gives identity.
    Eigen::Quaternionf imu_orientation_to_body(const Eigen::Quaternionf &orientation, const Eigen::Matrix3f &rotation);

    // subtracts the bias, then adds the leak computed from the corrected z.
    // expects a reading already in the body frame.
    Eigen::Vector3f calibrate_gyro(const Eigen::Vector3f &angular_velocity, const GyroCalibration &calibration);

    // subtracts the bias only: CMU applies no leak to the accelerometer.
    // expects a reading already in the body frame.
    Eigen::Vector3f calibrate_accel(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &bias);
}
