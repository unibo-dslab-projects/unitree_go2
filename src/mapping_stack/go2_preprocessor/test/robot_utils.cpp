// Unit tests for lidar_to_body(), remove_robot_points() and preprocess_cloud().
#include "go2_preprocessor/robot_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace go2_preprocessor {
namespace {
constexpr float kTolerance = 1e-3F;
// A wall 2 m straight ahead of the robot.
constexpr Point kRawWallAhead{-1.943F, 0.0F, 0.476F, 1.0F, 0, 0.0F};
// Body point (-0.4, 0.1, -0.25), in the middle of the robot box, taken back
// into the lidar frame. Outside the box if read in the lidar frame.
constexpr Point kRawRobotBack{0.4391F, 0.1F, 0.0920F, 2.0F, 0, 0.0F};
// Inside the box if read in the lidar frame, but at body (0.31, 0, 0.35),
// clear of the robot.
constexpr Point kRawBoxDecoy{-0.4F, 0.0F, -0.3F, 3.0F, 0, 0.0F};

TEST(LidarToBody, WallStraightAheadEndsUpTwoMetresAhead)
{
    PointCloud cloud;
    cloud.frame_id = "utlidar_lidar";
    cloud.points.push_back(kRawWallAhead);

    lidar_to_body(cloud);

    ASSERT_EQ(cloud.points.size(), 1U);
    EXPECT_NEAR(cloud.points[0].x, 2.0F, kTolerance);
    EXPECT_NEAR(cloud.points[0].y, 0.0F, kTolerance);
    EXPECT_NEAR(cloud.points[0].z, 0.0F, kTolerance);
    EXPECT_EQ(cloud.frame_id, "body");
}

// The body origin sits 4.68 cm above the lidar centre (CMU convention), so the
// lidar centre ends up just below it. A shift with the wrong sign puts it above.
TEST(LidarToBody, LidarCentreEndsUpBelowTheBodyOrigin)
{
    PointCloud cloud;
    cloud.points.push_back(Point{0.0F, 0.0F, 0.0F, 0.0F, 0, 0.0F});

    lidar_to_body(cloud);

    ASSERT_EQ(cloud.points.size(), 1U);
    EXPECT_FLOAT_EQ(cloud.points[0].x, 0.0F);
    EXPECT_FLOAT_EQ(cloud.points[0].y, 0.0F);
    EXPECT_FLOAT_EQ(cloud.points[0].z, -0.046825F);
}

// Changing frame must not drop points (filtering is a separate step), nor touch
// the per-point fields Point-LIO reads, nor the cloud's stamp.
TEST(LidarToBody, KeepsEveryPointItsOtherFieldsAndTheStamp)
{
    PointCloud cloud;
    cloud.stamp_ns = 1'700'000'000'123'456'789;
    cloud.points.push_back(Point{-1.943F, 0.0F, 0.476F, 40.0F, 5, 0.01F});
    cloud.points.push_back(kRawRobotBack);

    lidar_to_body(cloud);

    EXPECT_EQ(cloud.stamp_ns, 1'700'000'000'123'456'789);
    ASSERT_EQ(cloud.points.size(), 2U);
    EXPECT_FLOAT_EQ(cloud.points[0].intensity, 40.0F);
    EXPECT_EQ(cloud.points[0].ring, 5U);
    EXPECT_FLOAT_EQ(cloud.points[0].time, 0.01F);
}

// From -1 to 1 on every axis, so it is easy to see which points fall inside.
constexpr Box kUnitBox{-1.0F, 1.0F, -1.0F, 1.0F, -1.0F, 1.0F};

TEST(RemoveRobotPoints, DropsOnlyPointsInsideAndKeepsTheOrder)
{
    const std::vector<Point> points{
        Point{2.0F, 0.0F, 0.0F, 1.0F, 0, 0.0F},    // outside
        Point{0.0F, 0.0F, 0.0F, 2.0F, 0, 0.0F},    // inside
        Point{0.0F, 0.0F, 5.0F, 3.0F, 0, 0.0F},    // outside
        Point{0.5F, -0.5F, 0.5F, 4.0F, 0, 0.0F},   // inside
        Point{-3.0F, 0.0F, 0.0F, 5.0F, 0, 0.0F},   // outside
    };

    const std::vector<Point> kept = remove_robot_points(points, kUnitBox);

    ASSERT_EQ(kept.size(), 3U);
    EXPECT_FLOAT_EQ(kept[0].intensity, 1.0F);
    EXPECT_FLOAT_EQ(kept[1].intensity, 3.0F);
    EXPECT_FLOAT_EQ(kept[2].intensity, 5.0F);
}

TEST(RemoveRobotPoints, EmptyInputGivesEmptyOutput)
{
    EXPECT_TRUE(remove_robot_points({}, kUnitBox).empty());
}

// CMU's robot box, the node's default. The raw points above were chosen for it.
constexpr Box kGo2RobotBody{-0.7F, -0.1F, -0.3F, 0.3F, -0.646825F, -0.046825F};

// The robot box is defined in the body frame, so the filter must run after the
// turn. Swapping the order would keep the robot's back and drop the decoy.
TEST(PreprocessCloud, RemovesTheRobotsBackOnlyAfterTheTurn)
{
    PointCloud cloud;
    cloud.frame_id = "utlidar_lidar";
    cloud.points = {kRawWallAhead, kRawRobotBack, kRawBoxDecoy};

    preprocess_cloud(cloud, kGo2RobotBody);

    ASSERT_EQ(cloud.points.size(), 2U);
    EXPECT_FLOAT_EQ(cloud.points[0].intensity, 1.0F);  // wall
    EXPECT_FLOAT_EQ(cloud.points[1].intensity, 3.0F);  // decoy
    EXPECT_EQ(cloud.frame_id, "body");
}

TEST(PreprocessCloud, EmptyCloudStaysEmpty)
{
    PointCloud cloud;

    preprocess_cloud(cloud, kGo2RobotBody);

    EXPECT_TRUE(cloud.points.empty());
    EXPECT_EQ(cloud.frame_id, "body");
}

}  // namespace
}  // namespace go2_preprocessor
