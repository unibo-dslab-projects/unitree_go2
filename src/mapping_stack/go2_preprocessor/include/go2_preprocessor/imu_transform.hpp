#pragma once

#include <Eigen/Core>

namespace go2_preprocessor {
    // Gyroscope corrections, all in the body frame.
    struct GyroCalibration {
        Eigen::Vector3f bias = Eigen::Vector3f::Zero();  // rad/s
        // Share of the yaw rate (z) that shows up in the x and y readings.
        float z_to_x_leak = 0.0F;
        float z_to_y_leak = 0.0F;
    };

    // Turns a gyroscope or accelerometer reading from the IMU frame (mounted
    // upside down and tilted 15.1 deg) into the body frame (x forward, y left, z up).
    Eigen::Vector3f imu_to_body(const Eigen::Vector3f &vector);

    // Subtracts the bias, then adds the leak computed from the corrected z.
    // Expects a reading already in the body frame.
    Eigen::Vector3f calibrate_gyro(const Eigen::Vector3f &angular_velocity, const GyroCalibration &calibration);

    // Subtracts the bias only: CMU applies no leak to the accelerometer.
    // Expects a reading already in the body frame.
    Eigen::Vector3f calibrate_accel(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &bias);
}
