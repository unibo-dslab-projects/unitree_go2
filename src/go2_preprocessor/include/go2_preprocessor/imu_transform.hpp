#pragma once

#include <Eigen/Core>

namespace go2_preprocessor {
    // subtracts the bias. Expects a reading of the body IMU (/sportmodestate), whose axes are
    // already the body axes.
    Eigen::Vector3f calibrate_gyro(const Eigen::Vector3f &angular_velocity, const Eigen::Vector3f &bias);

    // multiplies by the scale (BodyImuCalibration::accel_scale). Standing still only shows the
    // reading along gravity, where a scale and a bias look the same, so go2_calibration measures
    // a scale.
    Eigen::Vector3f calibrate_accel(const Eigen::Vector3f &acceleration, float scale);
}
