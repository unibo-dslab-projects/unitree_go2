#include "go2_preprocessor/imu_transform.hpp"

#include <Eigen/Geometry>

namespace go2_preprocessor {
    Eigen::Vector3f imu_to_body(const Eigen::Vector3f &vector, const Eigen::Matrix3f &rotation)
    {
        return rotation * vector;
    }

    Eigen::Quaternionf imu_orientation_to_body(const Eigen::Quaternionf &orientation, const Eigen::Matrix3f &rotation)
    {
        // world <- body = (world <- IMU) * (IMU <- body), and IMU <- body is the
        // inverse of `rotation`. a valid rotation is guaranteed by rotation_from_parameter().
        return orientation * Eigen::Quaternionf(rotation).conjugate();
    }

    Eigen::Vector3f calibrate_gyro(const Eigen::Vector3f &angular_velocity, const GyroCalibration &calibration)
    {
        Eigen::Vector3f corrected = angular_velocity - calibration.bias;

        // the leak follows the true yaw rate, so it uses z after its own bias is gone.
        corrected.x() += calibration.z_to_x_leak * corrected.z();
        corrected.y() += calibration.z_to_y_leak * corrected.z();

        return corrected;
    }

    Eigen::Vector3f calibrate_accel(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &bias)
    {
        return acceleration - bias;
    }
}
