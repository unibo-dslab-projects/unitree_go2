#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace go2_preprocessor {
    // the IMU of /sportmodestate. Its axes are the body axes, so it needs no rotation.
    struct BodyImuCalibration {
        Eigen::Vector3f gyro_bias = Eigen::Vector3f::Zero();  // rad/s
        float accel_scale = 1.0F;  // makes the still accelerometer read 9.81 m/s^2, Point-LIO's g
    };

    // everything go2_calibration writes in its parameter file.
    // convention: p_body = R * p_sensor + t. Body frame: x forward, y left, z up.
    struct SensorCalibration {
        Eigen::Isometry3f lidar_to_body = Eigen::Isometry3f::Identity();  // translation in m
        BodyImuCalibration body_imu;
        float range_offset = 0.0F;  // m, subtracted from every lidar range (they read too long)
    };

    // turns the 9 numbers of a rotation parameter, row-major as go2_calibration writes them,
    // into a matrix. Throws std::invalid_argument unless there are 9 values forming a rotation
    // (no stretch, no mirror).
    Eigen::Matrix3f rotation_from_parameter(const std::vector<double> &values);

    // turns the 3 numbers of a vector parameter into a vector.
    // Throws std::invalid_argument unless there are exactly 3 values.
    Eigen::Vector3f vector_from_parameter(const std::vector<double> &values);
}
