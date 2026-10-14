#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_preprocessor {
    struct GyroCalibration {
        Eigen::Vector3f bias = Eigen::Vector3f::Zero();  // rad/s
        float z_to_x_leak = 0.0F;
        float z_to_y_leak = 0.0F;
    };

    // everything go2_calibration measures, as written in its parameter file.
    // convention: p_body = R * p_sensor + t. Body frame: x forward, y left, z up.
    struct SensorCalibration {
        Eigen::Isometry3f lidar_to_body = Eigen::Isometry3f::Identity();  // translation in m
        Eigen::Matrix3f imu_to_body = Eigen::Matrix3f::Identity();
        GyroCalibration gyro;
        float range_offset = 0.0F;  // m, subtracted from every lidar range (they read too long)
    };
}
