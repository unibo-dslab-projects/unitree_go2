#include "go2_preprocessor/convertor.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace go2_preprocessor {
namespace {

using sensor_msgs::msg::PointField;

// Shared setup: a two-point cloud where every field has a different value, so a
// value copied into the wrong field shows up as a wrong number.
class ConvertorTest : public ::testing::Test {
protected:
    ConvertorTest()
    {
        cloud_.frame_id = "utlidar_lidar";
        cloud_.stamp_ns = 1'700'000'000'123'456'789;
        cloud_.points.push_back(Point{1.0F, 2.0F, 3.0F, 40.0F, 5, 0.01F});
        cloud_.points.push_back(Point{-1.5F, 0.25F, -3.75F, 90.0F, 17, 0.06F});
    }

    PointCloud cloud_;
};

TEST_F(ConvertorTest, RoundTripKeepsEveryPointAndField)
{
    const PointCloud result = from_message(to_message(cloud_));

    EXPECT_EQ(result.frame_id, cloud_.frame_id);
    EXPECT_EQ(result.stamp_ns, cloud_.stamp_ns);
    ASSERT_EQ(result.points.size(), cloud_.points.size());

    for (std::size_t i = 0; i < cloud_.points.size(); ++i) {
        // Adds "point i" to any failure message below, so you know which one broke.
        SCOPED_TRACE("point " + std::to_string(i));

        // The values are only copied, never computed, so they must come back
        // unchanged: EXPECT_FLOAT_EQ tolerates only last-bit rounding.
        EXPECT_FLOAT_EQ(result.points[i].x, cloud_.points[i].x);
        EXPECT_FLOAT_EQ(result.points[i].y, cloud_.points[i].y);
        EXPECT_FLOAT_EQ(result.points[i].z, cloud_.points[i].z);
        EXPECT_FLOAT_EQ(result.points[i].intensity, cloud_.points[i].intensity);
        EXPECT_EQ(result.points[i].ring, cloud_.points[i].ring);
        EXPECT_FLOAT_EQ(result.points[i].time, cloud_.points[i].time);
    }
}

// The layout measured on the real Go2 /utlidar/cloud: Point-LIO and RViz read
// fields by name and offset, so the message must match it exactly.
TEST_F(ConvertorTest, MessageHasTheGo2Layout)
{
    const sensor_msgs::msg::PointCloud2 message = to_message(cloud_);

    EXPECT_EQ(message.height, 1U);
    EXPECT_EQ(message.width, 2U);
    EXPECT_EQ(message.point_step, 32U);
    EXPECT_EQ(message.row_step, 64U);
    EXPECT_EQ(message.data.size(), 64U);
    EXPECT_FALSE(message.is_bigendian);
    EXPECT_TRUE(message.is_dense);

    // A table-driven check: one row per expected field, one loop for all rows.
    struct ExpectedField {
        std::string name;
        std::uint32_t offset;
        std::uint8_t datatype;
    };
    const ExpectedField expected[] = {
        {"x", 0, PointField::FLOAT32},
        {"y", 4, PointField::FLOAT32},
        {"z", 8, PointField::FLOAT32},
        {"intensity", 16, PointField::FLOAT32},
        {"ring", 20, PointField::UINT16},
        {"time", 24, PointField::FLOAT32},
    };

    ASSERT_EQ(message.fields.size(), std::size(expected));
    for (std::size_t i = 0; i < message.fields.size(); ++i) {
        SCOPED_TRACE("field " + expected[i].name);
        EXPECT_EQ(message.fields[i].name, expected[i].name);
        EXPECT_EQ(message.fields[i].offset, expected[i].offset);
        EXPECT_EQ(message.fields[i].datatype, expected[i].datatype);
        EXPECT_EQ(message.fields[i].count, 1U);
    }
}

TEST_F(ConvertorTest, StampIsSplitIntoSecondsAndNanoseconds)
{
    const sensor_msgs::msg::PointCloud2 message = to_message(cloud_);

    EXPECT_EQ(message.header.stamp.sec, 1'700'000'000);
    EXPECT_EQ(message.header.stamp.nanosec, 123'456'789U);
}

TEST(ToMessage, PointWithNaNMarksMessageNotDense)
{
    PointCloud cloud;
    cloud.points.push_back(Point{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F, 0, 0.0F});

    EXPECT_FALSE(to_message(cloud).is_dense);
}

TEST(FromMessage, EmptyMessageGivesEmptyCloudWithItsFrame)
{
    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = "body";

    const PointCloud cloud = from_message(message);

    EXPECT_TRUE(cloud.points.empty());
    EXPECT_EQ(cloud.frame_id, "body");
}

// EXPECT_THROW(statement, exception type) passes only when the statement throws
// that type of exception.
TEST_F(ConvertorTest, DataShorterThanDeclaredThrows)
{
    sensor_msgs::msg::PointCloud2 message = to_message(cloud_);
    message.data.pop_back();  // one byte missing

    EXPECT_THROW(from_message(message), std::runtime_error);
}

TEST_F(ConvertorTest, MissingFieldThrows)
{
    sensor_msgs::msg::PointCloud2 message = to_message(cloud_);
    message.fields.pop_back();  // drops "time"

    EXPECT_THROW(from_message(message), std::runtime_error);
}

}  // namespace
}  // namespace go2_preprocessor
