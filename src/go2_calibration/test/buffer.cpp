// Unit tests for CalibrationBuffer: where each sample goes by phase, and how a scan pairs with its
// copy in /utlidar/cloud_base.
#include "go2_calibration/buffer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace go2_calibration {
namespace {

ScanPoint point(std::uint32_t time_bits, double x)
{
    return ScanPoint{PointKey{time_bits, 0U, 1}, Eigen::Vector3d(x, 0.0, 0.0)};
}

TEST(CalibrationBuffer, SortsImuSamplesByPhase)
{
    CalibrationBuffer buffer;
    const Eigen::Vector3d gyro(0.1, 0.2, 0.3);
    const Eigen::Vector3d accel(0.0, 0.0, 9.8);

    buffer.add_lidar_imu(0, gyro, accel);  // no phase yet: dropped
    buffer.set_phase("still");
    buffer.add_lidar_imu(1, gyro, accel);
    buffer.add_body_imu(1, gyro, accel);
    buffer.set_phase("spin_left");
    buffer.add_lidar_imu(2, gyro, accel);
    buffer.add_body_imu(3, gyro, accel);  // moving: the body accelerometer is not kept
    buffer.set_phase("pause");
    buffer.add_lidar_imu(4, gyro, accel);
    buffer.add_body_imu(4, gyro, accel);

    const CalibrationData &data = buffer.data();
    EXPECT_EQ(data.still_gyro.size(), 1U);
    EXPECT_EQ(data.still_accel.size(), 1U);
    EXPECT_EQ(data.still_body_gyro.size(), 1U);
    EXPECT_EQ(data.still_body_accel.size(), 1U);
    ASSERT_EQ(data.moving_lidar_gyro.size(), 1U);
    EXPECT_EQ(data.moving_lidar_gyro[0].time_ns, 2);
    ASSERT_EQ(data.moving_body_gyro.size(), 1U);
    EXPECT_EQ(data.moving_body_gyro[0].time_ns, 3);
}

// The firmware drops points from the copy and changes their order: the key pairs them.
TEST(CalibrationBuffer, PairsAScanWithItsCopyByKey)
{
    CalibrationBuffer buffer;
    buffer.set_phase("still");

    buffer.add_scan(100, {point(1, 1.0), point(2, 2.0), point(3, 3.0)}, false);
    buffer.add_scan(100, {point(3, 30.0), point(1, 10.0)}, true);

    const CalibrationData &data = buffer.data();
    EXPECT_EQ(data.scans, 1U);
    ASSERT_EQ(data.lidar_points.size(), 2U);
    EXPECT_EQ(data.lidar_points[0].x(), 3.0);
    EXPECT_EQ(data.body_points[0].x(), 30.0);
    EXPECT_EQ(data.lidar_points[1].x(), 1.0);
    EXPECT_EQ(data.body_points[1].x(), 10.0);
    EXPECT_EQ(data.floor_points.size(), 3U);  // every point of the lidar scan
}

TEST(CalibrationBuffer, TheCopyMayArriveFirst)
{
    CalibrationBuffer buffer;
    buffer.set_phase("still");

    buffer.add_scan(100, {point(1, 10.0)}, true);
    buffer.add_scan(100, {point(1, 1.0)}, false);

    ASSERT_EQ(buffer.data().lidar_points.size(), 1U);
    EXPECT_EQ(buffer.data().lidar_points[0].x(), 1.0);
    EXPECT_EQ(buffer.data().body_points[0].x(), 10.0);
}

TEST(CalibrationBuffer, SkipsKeysSeenTwiceInAScan)
{
    CalibrationBuffer buffer;
    buffer.set_phase("still");

    buffer.add_scan(100, {point(1, 1.0), point(1, 1.5), point(2, 2.0)}, false);
    buffer.add_scan(100, {point(1, 10.0), point(2, 20.0)}, true);

    ASSERT_EQ(buffer.data().lidar_points.size(), 1U);
    EXPECT_EQ(buffer.data().lidar_points[0].x(), 2.0);
}

TEST(CalibrationBuffer, PairsOnlyScansWithTheSameStamp)
{
    CalibrationBuffer buffer;
    buffer.set_phase("still");

    buffer.add_scan(100, {point(1, 1.0)}, false);
    buffer.add_scan(200, {point(1, 10.0)}, true);

    EXPECT_EQ(buffer.data().scans, 0U);
}

TEST(CalibrationBuffer, WantsScansOnlyWhileStillAndBelowTheLimit)
{
    CalibrationBuffer buffer(1);
    EXPECT_FALSE(buffer.wants_scans());  // no phase yet
    buffer.set_phase("spin_left");
    EXPECT_FALSE(buffer.wants_scans());
    buffer.set_phase("still");
    EXPECT_TRUE(buffer.wants_scans());

    buffer.add_scan(100, {point(1, 1.0)}, false);
    buffer.add_scan(100, {point(1, 10.0)}, true);
    EXPECT_FALSE(buffer.wants_scans());

    buffer.add_scan(200, {point(1, 1.0)}, false);
    buffer.add_scan(200, {point(1, 10.0)}, true);
    EXPECT_EQ(buffer.data().scans, 1U);
}

// The live source repeats the phase at 50 Hz: a repeat must not drop a scan waiting for its copy.
TEST(CalibrationBuffer, ARepeatedPhaseKeepsAWaitingScan)
{
    CalibrationBuffer buffer;
    buffer.set_phase("still");
    buffer.add_scan(100, {point(1, 1.0)}, false);
    buffer.set_phase("still");
    buffer.add_scan(100, {point(1, 10.0)}, true);

    EXPECT_EQ(buffer.data().scans, 1U);
}

TEST(CalibrationBuffer, APhaseChangeDropsAWaitingScan)
{
    CalibrationBuffer buffer;
    buffer.set_phase("still");
    buffer.add_scan(100, {point(1, 1.0)}, false);
    buffer.set_phase("pause");
    buffer.set_phase("still");
    buffer.add_scan(100, {point(1, 10.0)}, true);

    EXPECT_EQ(buffer.data().scans, 0U);
}

}  // namespace
}  // namespace go2_calibration
