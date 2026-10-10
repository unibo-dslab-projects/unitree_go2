#include "go2_preprocessor/imu_transform.hpp"

#include <Eigen/Geometry>

namespace go2_preprocessor {
    namespace {
    constexpr float kPi = 3.14159265F;
    constexpr float kImuTilt = 15.1F * kPi / 180.0F;  // rad

    // Read right to left: a half turn around x undoes the upside-down mount
    // (y and z change sign), then a -15.1 deg turn around y undoes the tilt.
    const Eigen::Quaternionf kImuToBody =
        Eigen::AngleAxisf(-kImuTilt, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(kPi, Eigen::Vector3f::UnitX());
    }  // namespace

    Eigen::Vector3f imu_to_body(const Eigen::Vector3f &vector)
    {
        return kImuToBody * vector;
    }

    Eigen::Vector3f calibrate_gyro(const Eigen::Vector3f &angular_velocity, const GyroCalibration &calibration)
    {
        Eigen::Vector3f corrected = angular_velocity - calibration.bias;

        // The leak follows the true yaw rate, so it uses z after its own bias is gone.
        corrected.x() += calibration.z_to_x_leak * corrected.z();
        corrected.y() += calibration.z_to_y_leak * corrected.z();

        return corrected;
    }

    Eigen::Vector3f calibrate_accel(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &bias)
    {
        return acceleration - bias;
    }
}
