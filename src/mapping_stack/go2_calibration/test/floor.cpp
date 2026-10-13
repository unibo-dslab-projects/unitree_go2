// Unit tests for find_floor(), fit_range_offset() and tilt_from_up_deg().
#include "go2_calibration/floor.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_calibration {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kAngleToleranceDeg = 1e-4;  // arc cosine of rounding, see test/fitting.cpp

Eigen::Matrix3d go2_lidar_to_body()
{
    return (Eigen::AngleAxisd(165.0 * kPi / 180.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-123.0 * kPi / 180.0, Eigen::Vector3d::UnitZ()))
        .toRotationMatrix();
}

// A room built in the body frame and seen from the tilted lidar (lidar = R^T * body): a 4 x 4 m
// floor 0.39 m below the lidar (41 x 41 points) and, if asked, a wall 1.5 m ahead.
std::vector<Eigen::Vector3d> room_in_lidar_frame(bool with_floor)
{
    const Eigen::Matrix3d body_to_lidar = go2_lidar_to_body().transpose();
    std::vector<Eigen::Vector3d> points;
    for (int i = -20; i <= 20; ++i) {
        for (int j = -20; j <= 20; ++j) {
            if (with_floor) {
                points.push_back(body_to_lidar * Eigen::Vector3d(0.1 * i, 0.1 * j, -0.39));
            }
        }
    }
    for (int j = -20; j <= 20; ++j) {
        for (int k = -3; k <= 10; ++k) {
            points.push_back(body_to_lidar * Eigen::Vector3d(1.5, 0.1 * j, 0.1 * k));
        }
    }
    return points;
}

TEST(FindFloor, FindsTheFloorUnderTheTiltedLidar)
{
    const auto floor = find_floor(room_in_lidar_frame(true));

    ASSERT_TRUE(floor.has_value());
    EXPECT_NEAR(floor->distance, 0.39, 1e-9);
    EXPECT_EQ(floor->inliers, std::size_t{41 * 41});
    // "Up" found in the lidar frame must be +z once turned into the body frame.
    EXPECT_LT(tilt_from_up_deg(go2_lidar_to_body() * floor->normal), kAngleToleranceDeg);
}

// The wall is the only plane, but 1.5 m away: too far to be the floor under the robot.
TEST(FindFloor, IgnoresPlanesOutsideTheDistanceRange)
{
    EXPECT_FALSE(find_floor(room_in_lidar_frame(false)).has_value());
}

TEST(FindFloor, NeedsThreePoints)
{
    EXPECT_FALSE(find_floor({Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()}).has_value());
}

// The 4 x 4 m floor of room_in_lidar_frame() as a lidar sees it whose every range reads `offset`
// metres too long: each point slides along its own ray.
std::vector<Eigen::Vector3d> floor_with_range_offset(double offset)
{
    const Eigen::Matrix3d body_to_lidar = go2_lidar_to_body().transpose();
    std::vector<Eigen::Vector3d> points;
    for (int i = -20; i <= 20; ++i) {
        for (int j = -20; j <= 20; ++j) {
            const Eigen::Vector3d point = body_to_lidar * Eigen::Vector3d(0.1 * i, 0.1 * j, -0.39);
            points.push_back(point * (1.0 + offset / point.norm()));
        }
    }
    return points;
}

// The floor plane, "up" towards the lidar, somewhere inside the bowl the offset makes.
Plane floor_plane(double distance)
{
    Plane plane;
    plane.normal = go2_lidar_to_body().transpose() * Eigen::Vector3d::UnitZ();
    plane.distance = distance;
    return plane;
}

TEST(FitRangeOffset, RecoversTheOffsetAndTheTrueDistance)
{
    const RangeOffset range = fit_range_offset(floor_with_range_offset(0.12), floor_plane(0.45), 0.2);

    EXPECT_NEAR(range.offset, 0.12, 1e-9);
    EXPECT_NEAR(range.distance, 0.39, 1e-9);
    EXPECT_NEAR(range.residual, 0.0, 1e-9);
    EXPECT_EQ(range.points, std::size_t{41 * 41});
}

TEST(FitRangeOffset, ZeroForAnExactLidar)
{
    const RangeOffset range = fit_range_offset(floor_with_range_offset(0.0), floor_plane(0.39));

    EXPECT_NEAR(range.offset, 0.0, 1e-9);
    EXPECT_NEAR(range.distance, 0.39, 1e-9);
}

// The bowl spans 0.41 to 0.51 m; the wall of room_in_lidar_frame() comes no closer to the floor
// plane than 0.30 m from the lidar, outside the 0.35 to 0.55 m band, so it must not count.
TEST(FitRangeOffset, IgnoresPointsOutsideTheBand)
{
    std::vector<Eigen::Vector3d> points = floor_with_range_offset(0.12);
    const std::vector<Eigen::Vector3d> wall = room_in_lidar_frame(false);
    points.insert(points.end(), wall.begin(), wall.end());

    const RangeOffset range = fit_range_offset(points, floor_plane(0.45), 0.1);

    EXPECT_NEAR(range.offset, 0.12, 1e-9);
    EXPECT_EQ(range.points, std::size_t{41 * 41});
}

TEST(FitRangeOffset, NeedsPointsSeenAtDifferentAngles)
{
    const Eigen::Vector3d below = go2_lidar_to_body().transpose() * Eigen::Vector3d(0.0, 0.0, -0.39);

    EXPECT_THROW(fit_range_offset({below, below, below}, floor_plane(0.39)), std::invalid_argument);
    EXPECT_THROW(fit_range_offset({}, floor_plane(0.39)), std::invalid_argument);
}

TEST(TiltFromUp, MeasuresTheAngleToPlusZ)
{
    EXPECT_NEAR(tilt_from_up_deg(Eigen::Vector3d::UnitZ()), 0.0, kAngleToleranceDeg);
    EXPECT_NEAR(tilt_from_up_deg(Eigen::Vector3d::UnitX()), 90.0, 1e-9);
    EXPECT_NEAR(tilt_from_up_deg(-Eigen::Vector3d::UnitZ()), 180.0, kAngleToleranceDeg);
    EXPECT_NEAR(tilt_from_up_deg(Eigen::Vector3d(0.0, 2.0, 2.0)), 45.0, 1e-9);  // length does not matter
}

}  // namespace
}  // namespace go2_calibration
