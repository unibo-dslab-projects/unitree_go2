// Unit tests for calibrate(): a synthetic Go2 whose every frame is known must come back, and data
// that is wrong or missing must fail a check or throw.
#include "go2_calibration/calibrate.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_calibration {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kMs = 1'000'000;  // nanoseconds

double radians(double degrees)
{
    return degrees * kPi / 180.0;
}

// Our Go2's lidar: R_y(165 deg) R_z(-123 deg), 0.282 m ahead of the body origin.
Eigen::Matrix3d go2_lidar_to_body()
{
    return (Eigen::AngleAxisd(radians(165.0), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(radians(-123.0), Eigen::Vector3d::UnitZ()))
        .toRotationMatrix();
}

Eigen::Vector3d lidar_position()
{
    return Eigen::Vector3d(0.282, 0.0, 0.0);
}

// On purpose not the lidar frame: the calibration must not assume where the IMU sits.
Eigen::Matrix3d imu_to_body()
{
    return Eigen::AngleAxisd(radians(30.0), Eigen::Vector3d::UnitX()).toRotationMatrix() * go2_lidar_to_body();
}

Eigen::Vector3d body_bias()
{
    return Eigen::Vector3d(0.003, -0.005, 0.0006);  // rad/s, body frame
}

// A smooth turn, always faster than 0.15 rad/s; one_axis keeps only the yaw.
Eigen::Vector3d turn_rate(double seconds, bool one_axis)
{
    const double x = one_axis ? 0.0 : 0.3 * std::sin(2.0 * seconds);
    const double y = one_axis ? 0.0 : 0.2 * std::cos(3.0 * seconds);
    return Eigen::Vector3d(x, y, 1.0 + 0.5 * std::sin(seconds));
}

// Everything a Go2 standing still and then turning gives the buffer, with every lidar range
// `range_offset` metres too long.
CalibrationData go2_data(double range_offset = 0.0, bool one_axis = false)
{
    CalibrationData data;
    const Eigen::Matrix3d lidar_to_body = go2_lidar_to_body();
    const Eigen::Matrix3d body_to_imu = imu_to_body().transpose();

    // A 4 x 4 m floor 0.39 m below the lidar; the firmware copies each seen point into the body frame.
    for (int i = -20; i <= 20; ++i) {
        for (int j = -20; j <= 20; ++j) {
            const Eigen::Vector3d truth = lidar_to_body.transpose() * Eigen::Vector3d(0.1 * i, 0.1 * j, -0.39);
            const Eigen::Vector3d seen = truth * (1.0 + range_offset / truth.norm());
            data.lidar_points.push_back(seen);
            data.body_points.push_back(lidar_to_body * seen + lidar_position());
            data.floor_points.push_back(seen);
        }
    }
    data.scans = 1;

    // Still: the bias with a little noise, and gravity straight down in the body frame.
    for (int i = 0; i < 1000; ++i) {
        const double noise = i % 2 == 0 ? 0.001 : -0.001;
        data.still_gyro.push_back(body_to_imu * (body_bias() + Eigen::Vector3d::Constant(noise)));
        data.still_accel.push_back(body_to_imu * Eigen::Vector3d(0.0, 0.0, 9.81));
    }

    // Turning: the lidar IMU at 250 Hz with its bias, the body IMU at 300 Hz and 10 ms late.
    for (std::int64_t t = 0; t < 4000 * kMs; t += 4 * kMs) {
        const double seconds = static_cast<double>(t) / 1e9;
        data.moving_lidar_gyro.push_back(TimedVector{t, body_to_imu * (turn_rate(seconds, one_axis) + body_bias())});
    }
    for (std::int64_t t = 0; t < 4000 * kMs; t += 3'333'333) {
        const double seconds = static_cast<double>(t) / 1e9;
        data.moving_body_gyro.push_back(TimedVector{t + 10 * kMs, turn_rate(seconds, one_axis)});
    }
    return data;
}

Check check_named(const Calibration &calibration, const std::string &name)
{
    for (const Check &check : calibration.checks) {
        if (check.name == name) {
            return check;
        }
    }
    ADD_FAILURE() << "no check named " << name;
    return Check{};
}

TEST(Calibrate, RecoversEveryFrameOfASyntheticGo2)
{
    const Calibration c = calibrate(go2_data());

    EXPECT_LT(angle_between_deg(c.lidar_to_body.rotation, go2_lidar_to_body()), 1e-4);
    EXPECT_LT((c.lidar_to_body.translation - lidar_position()).norm(), 1e-9);
    EXPECT_LT(c.floor_tilt_deg, 1e-3);
    EXPECT_NEAR(c.range.offset, 0.0, 1e-6);
    EXPECT_NEAR(c.range.distance, 0.39, 1e-6);
    EXPECT_LT(angle_between_deg(c.imu_to_body.fit.rotation, imu_to_body()), 0.5);
    EXPECT_EQ(c.imu_to_body.lag_ns, 10 * kMs);
    EXPECT_NEAR(c.imu_to_lidar_deg, 30.0, 0.5);
    EXPECT_LT((c.gyro_bias.mean - body_bias()).norm(), 1e-3);
    EXPECT_LT(c.gravity_tilt_deg, 0.5);
    EXPECT_TRUE(c.passed());
}

// The turn itself moves x and y along with z (sin 2t does not average to 0 over 4 s), so the leak
// is not 0: it must be what the true body rates give, once rotation and bias are taken out.
TEST(Calibrate, LeakIsWhatTheTrueRatesGive)
{
    const CalibrationData data = go2_data();
    std::vector<Eigen::Vector3d> true_rates;
    for (const TimedVector &rate : data.moving_lidar_gyro) {
        true_rates.push_back(turn_rate(static_cast<double>(rate.time_ns) / 1e9, false));
    }
    const GyroLeak expected = fit_leak(true_rates);

    const Calibration c = calibrate(data);

    EXPECT_NEAR(c.leak.z_to_x, expected.z_to_x, 0.005);
    EXPECT_NEAR(c.leak.z_to_y, expected.z_to_y, 0.005);
}

// The firmware copies every point, offset included, so the mount fit stays exact while the floor
// bends.
TEST(Calibrate, MeasuresTheRangeOffsetFromTheFloor)
{
    const Calibration c = calibrate(go2_data(0.12));

    EXPECT_NEAR(c.range.offset, 0.12, 0.01);
    EXPECT_NEAR(c.range.distance, 0.39, 0.01);
    EXPECT_LT(angle_between_deg(c.lidar_to_body.rotation, go2_lidar_to_body()), 1e-4);
}

TEST(Calibrate, FailsTheSpreadCheckForTurnsAboutOneAxis)
{
    const Calibration c = calibrate(go2_data(0.0, true));

    EXPECT_FALSE(check_named(c, "turn axis spread").passed());
    EXPECT_FALSE(c.passed());
}

TEST(Calibrate, FailsTheNoiseCheckWhenTheRobotWasNotStill)
{
    CalibrationData data = go2_data();
    for (std::size_t i = 0; i < data.still_gyro.size(); ++i) {
        data.still_gyro[i] += Eigen::Vector3d::Constant(i % 2 == 0 ? 0.05 : -0.05);
    }

    const Calibration c = calibrate(data);

    EXPECT_FALSE(check_named(c, "still gyroscope noise, rad/s").passed());
    EXPECT_FALSE(c.passed());
}

TEST(Calibrate, ThrowsWhenDataIsMissing)
{
    CalibrationData no_points = go2_data();
    no_points.lidar_points.clear();
    no_points.body_points.clear();
    EXPECT_THROW(calibrate(no_points), std::runtime_error);

    CalibrationData no_still = go2_data();
    no_still.still_gyro.clear();
    no_still.still_accel.clear();
    EXPECT_THROW(calibrate(no_still), std::runtime_error);

    CalibrationData no_turning = go2_data();
    no_turning.moving_lidar_gyro.clear();
    EXPECT_THROW(calibrate(no_turning), std::runtime_error);

    // The same floor three times as far: outside the 0.2 to 0.7 m a standing Go2 can have.
    CalibrationData no_floor = go2_data();
    for (Eigen::Vector3d &point : no_floor.floor_points) {
        point *= 3.0;
    }
    EXPECT_THROW(calibrate(no_floor), std::runtime_error);
}

TEST(Check, PassesOnTheRightSideOfTheLimit)
{
    EXPECT_TRUE((Check{"at least", 2.0, 1.0, true}).passed());
    EXPECT_FALSE((Check{"at least", 0.5, 1.0, true}).passed());
    EXPECT_TRUE((Check{"at most", 0.5, 1.0, false}).passed());
    EXPECT_FALSE((Check{"at most", 2.0, 1.0, false}).passed());
    EXPECT_TRUE((Check{"on the limit", 1.0, 1.0, false}).passed());
}

}  // namespace
}  // namespace go2_calibration
