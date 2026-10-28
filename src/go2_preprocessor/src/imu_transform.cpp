#include "go2_preprocessor/imu_transform.hpp"

namespace go2_preprocessor {
    Eigen::Vector3f calibrate_gyro(const Eigen::Vector3f &angular_velocity, const Eigen::Vector3f &bias)
    {
        return angular_velocity - bias;
    }

    Eigen::Vector3f calibrate_accel(const Eigen::Vector3f &acceleration, float scale)
    {
        return acceleration * scale;
    }
}
