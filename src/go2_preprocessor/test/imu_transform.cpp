// Unit tests for calibrate_gyro() and calibrate_accel().
#include "go2_preprocessor/imu_transform.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

namespace go2_preprocessor {
namespace {

// Our Go2's body IMU at rest (2026-10-19 calibration bag): bias and accelerometer.
TEST(CalibrateGyro, SubtractsTheBias)
{
    const Eigen::Vector3f bias(-0.0097F, 0.0108F, -0.0041F);

    const Eigen::Vector3f corrected = calibrate_gyro(Eigen::Vector3f(-0.0097F, 0.0108F, 0.4959F), bias);

    EXPECT_NEAR(corrected.x(), 0.0F, 1e-6F);
    EXPECT_NEAR(corrected.y(), 0.0F, 1e-6F);
    EXPECT_NEAR(corrected.z(), 0.5F, 1e-6F);
}

// The still reading (|a| = 9.446) must come out as Point-LIO's 9.81, and the small sideways
// components (stance tilt) must grow by the same factor, not stay as they were.
TEST(CalibrateAccel, ScalesEveryAxis)
{
    const Eigen::Vector3f still(-0.146F, -0.133F, 9.444F);

    const Eigen::Vector3f corrected = calibrate_accel(still, 9.81F / still.norm());

    EXPECT_NEAR(corrected.norm(), 9.81F, 1e-5F);
    EXPECT_NEAR(corrected.x(), -0.15162F, 1e-5F);
    EXPECT_NEAR(corrected.y(), -0.13813F, 1e-5F);
}

}  // namespace
}  // namespace go2_preprocessor
