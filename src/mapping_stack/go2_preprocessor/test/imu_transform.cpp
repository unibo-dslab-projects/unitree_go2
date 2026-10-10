// Unit tests for imu_to_body(), calibrate_gyro() and calibrate_accel().
#include "go2_preprocessor/imu_transform.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_preprocessor {
namespace {

// Raw readings below are rounded to 3 decimals, so results drift by a few
// tenths of a thousandth. 1e-3 absorbs that and still catches a wrong sign
// (error of the size of the reading) or a missing 15.1 deg tilt (about 0.13
// for a 0.5 rad/s turn, about 2.5 m/s^2 for gravity).
constexpr float kTolerance = 1e-3F;

// Body y points left, so a nose-down nod is a positive turn around y. The
// upside-down IMU reports it with the opposite sign; the tilt leaves y alone.
TEST(ImuToBody, NoseDownNodIsPositiveY)
{
    const Eigen::Vector3f body = imu_to_body(Eigen::Vector3f(0.0F, -0.5F, 0.0F));

    EXPECT_NEAR(body.x(), 0.0F, 1e-6F);
    EXPECT_NEAR(body.y(), 0.5F, 1e-6F);
    EXPECT_NEAR(body.z(), 0.0F, 1e-6F);
}

// A left turn at 0.5 rad/s, as the tilted, upside-down IMU reports it.
TEST(ImuToBody, LeftTurnIsPositiveZ)
{
    const Eigen::Vector3f body = imu_to_body(Eigen::Vector3f(0.130F, 0.0F, -0.483F));

    EXPECT_NEAR(body.x(), 0.0F, kTolerance);
    EXPECT_NEAR(body.y(), 0.0F, kTolerance);
    EXPECT_NEAR(body.z(), 0.5F, kTolerance);
}

// The same turn also fixes the accelerometer: at rest it must read gravity's
// reaction straight up the body z axis.
TEST(ImuToBody, GravityAtRestPointsUp)
{
    const Eigen::Vector3f body = imu_to_body(Eigen::Vector3f(2.556F, 0.0F, -9.471F));

    EXPECT_NEAR(body.x(), 0.0F, kTolerance);
    EXPECT_NEAR(body.y(), 0.0F, kTolerance);
    EXPECT_NEAR(body.z(), 9.81F, kTolerance);
}

constexpr float kPi = 3.14159265F;

// How the IMU sits on the robot: what it reports as its own orientation when
// the robot stands level and faces the world's x axis.
const Eigen::Quaternionf kLevelRobotImu =
    Eigen::AngleAxisf(-15.1F * kPi / 180.0F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(kPi, Eigen::Vector3f::UnitX());

TEST(ImuOrientationToBody, LevelRobotGivesNoRotation)
{
    const Eigen::Quaternionf body = imu_orientation_to_body(kLevelRobotImu);

    EXPECT_NEAR(body.angularDistance(Eigen::Quaternionf::Identity()), 0.0F, kTolerance);
}

// A robot turned 90 deg left must come out turned 90 deg left. Multiplying on
// the wrong side still passes the level test above, but fails this one.
TEST(ImuOrientationToBody, RobotYawComesThrough)
{
    const Eigen::Quaternionf yaw(Eigen::AngleAxisf(kPi / 2.0F, Eigen::Vector3f::UnitZ()));

    const Eigen::Quaternionf body = imu_orientation_to_body(yaw * kLevelRobotImu);

    EXPECT_NEAR(body.angularDistance(yaw), 0.0F, kTolerance);
}

// Bias z is not zero on purpose: corrected z is 0.4, raw z is 0.5. Taking the
// leak from raw z, or adding it before the bias, gives x = 0.075 and fails.
TEST(CalibrateGyro, SubtractsBiasThenAddsLeakFromCorrectedZ)
{
    const GyroCalibration calibration{Eigen::Vector3f(0.01F, 0.02F, 0.1F), 0.15F, -0.28F};

    const Eigen::Vector3f corrected = calibrate_gyro(Eigen::Vector3f(0.01F, 0.02F, 0.5F), calibration);

    EXPECT_NEAR(corrected.x(), 0.06F, 1e-6F);    // 0 + 0.15 * 0.4
    EXPECT_NEAR(corrected.y(), -0.112F, 1e-6F);  // 0 - 0.28 * 0.4
    EXPECT_NEAR(corrected.z(), 0.4F, 1e-6F);
}

TEST(CalibrateAccel, SubtractsBiasOnly)
{
    const Eigen::Vector3f corrected =
        calibrate_accel(Eigen::Vector3f(0.1F, 0.2F, 9.9F), Eigen::Vector3f(0.1F, 0.2F, 0.09F));

    EXPECT_NEAR(corrected.x(), 0.0F, 1e-6F);
    EXPECT_NEAR(corrected.y(), 0.0F, 1e-6F);
    EXPECT_NEAR(corrected.z(), 9.81F, 1e-5F);
}

}  // namespace
}  // namespace go2_preprocessor
