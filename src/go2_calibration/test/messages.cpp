// Unit tests for the message conversions: clouds to keyed points, stamps, the body gyroscope and
// the sport requests.
#include "go2_calibration/messages.hpp"

#include "go2_cloud.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace go2_calibration {
namespace {

using test::Go2Point;
using test::go2_cloud;

TEST(ToScanPoints, ReadsPositionsAndKeys)
{
    const auto points = to_scan_points(go2_cloud({{1.0F, 2.0F, 3.0F, 50.0F, 7, 0.01F}, {4.0F, 5.0F, 6.0F, 60.0F, 8, 0.02F}}));

    ASSERT_EQ(points.size(), 2U);
    EXPECT_TRUE(points[0].position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
    EXPECT_TRUE(points[1].position.isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
    EXPECT_EQ(std::get<2>(points[0].key), 7);
    EXPECT_NE(points[0].key, points[1].key);
}

// The firmware's copy keeps time, intensity and ring: equal fields give equal keys.
TEST(ToScanPoints, EqualFieldsGiveEqualKeys)
{
    const auto lidar = to_scan_points(go2_cloud({{1.0F, 2.0F, 3.0F, 50.0F, 7, 0.01F}}));
    const auto body = to_scan_points(go2_cloud({{9.0F, 9.0F, 9.0F, 50.0F, 7, 0.01F}}));

    EXPECT_EQ(lidar[0].key, body[0].key);
}

TEST(ToScanPoints, DropsPointsThatAreNotFinite)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();

    const auto points = to_scan_points(go2_cloud({{nan, 0.0F, 0.0F, 1.0F, 1, 0.0F}, {1.0F, 0.0F, 0.0F, 1.0F, 1, 0.0F}}));

    ASSERT_EQ(points.size(), 1U);
    EXPECT_EQ(points[0].position.x(), 1.0);
}

TEST(ToScanPoints, ThrowsWithoutTheRingField)
{
    EXPECT_THROW(to_scan_points(go2_cloud({Go2Point{}}, false)), std::runtime_error);
}

TEST(ToScanPoints, EmptyCloudGivesNoPoints)
{
    EXPECT_TRUE(to_scan_points(go2_cloud({})).empty());
}

TEST(ToNanoseconds, JoinsSecondsAndNanoseconds)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = 2;
    stamp.nanosec = 5;

    EXPECT_EQ(to_nanoseconds(stamp), 2'000'000'005);
}

TEST(BodyGyro, ReadsTheImuStateGyroscope)
{
    unitree_go::msg::SportModeState state;
    state.imu_state.gyroscope = {0.5F, -0.25F, 1.0F};

    EXPECT_TRUE(body_gyro(state).isApprox(Eigen::Vector3d(0.5, -0.25, 1.0)));
}

TEST(BodyAccel, ReadsTheImuStateAccelerometer)
{
    unitree_go::msg::SportModeState state;
    state.imu_state.accelerometer = {-0.25F, 0.5F, 9.5F};

    EXPECT_TRUE(body_accel(state).isApprox(Eigen::Vector3d(-0.25, 0.5, 9.5)));
}

TEST(ToRequest, StopMoveAndBalanceStandCarryNoParameter)
{
    const auto stop = to_request(Command{});
    EXPECT_EQ(stop.header.identity.api_id, kStopMoveApiId);
    EXPECT_EQ(stop.parameter, "");

    const auto balance = to_request(Command{Action::balance_stand});
    EXPECT_EQ(balance.header.identity.api_id, kBalanceStandApiId);
    EXPECT_EQ(balance.parameter, "");
}

// As Unitree's SportClient: {"x":...,"y":...,"z":...}, with a decimal point whatever the locale.
TEST(ToRequest, MoveAndEulerCarryXyzAsJson)
{
    const auto move = to_request(Command{Action::move, 0.0, 0.0, 1.5});
    EXPECT_EQ(move.header.identity.api_id, kMoveApiId);
    EXPECT_EQ(move.parameter, R"({"x":0,"y":0,"z":1.5})");

    const auto euler = to_request(Command{Action::euler, -0.2, 0.1, 0.0});
    EXPECT_EQ(euler.header.identity.api_id, kEulerApiId);
    EXPECT_EQ(euler.parameter, R"({"x":-0.2,"y":0.1,"z":0})");
}

}  // namespace
}  // namespace go2_calibration
