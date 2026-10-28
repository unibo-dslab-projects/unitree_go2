// Unit tests for to_parameter_file() and describe(): the layout go2_preprocessor and the ROS 2
// parameter parser expect, and a readable PASS / FAIL per check.
#include "go2_calibration/report.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include <Eigen/Core>

namespace go2_calibration {
namespace {

const std::string kCreated = "2026-10-13 18:00:00 CEST";

bool contains(const std::string &text, const std::string &part)
{
    return text.find(part) != std::string::npos;
}

std::string joined(const std::vector<std::string> &lines)
{
    std::string text;
    for (const std::string &line : lines) {
        text += line + '\n';
    }
    return text;
}

// A quarter turn around z, written out so that every entry is exact.
Eigen::Matrix3d quarter_turn()
{
    Eigen::Matrix3d rotation;
    rotation << 0.0, -1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 0.0, 1.0;
    return rotation;
}

Calibration example()
{
    Calibration c;
    c.lidar_to_body.rotation = quarter_turn();
    c.lidar_to_body.translation = Eigen::Vector3d(0.282, 0.0, 0.0);
    c.lidar_to_body.samples = 151577;
    c.scans = 100;
    c.range.offset = 0.128;
    c.range.distance = 0.316;
    c.imu_to_body.fit.rotation = quarter_turn();
    c.body_gyro_bias.mean = Eigen::Vector3d(0.00001, -0.0000025, 0.0017);
    c.body_accel.mean = Eigen::Vector3d(0.0, 0.0, 9.42);
    c.accel_scale = 9.81 / 9.42;
    c.checks = {Check{"matched lidar points", 151577.0, 1000.0, true}, Check{"gyro_leak left", 0.2, 0.1, false}};
    return c;
}

TEST(ToParameterFile, WritesEveryParameterUnderTheNode)
{
    const std::string text = to_parameter_file(example(), "go2_preprocessor", "a live run", kCreated);

    EXPECT_TRUE(contains(text, "go2_preprocessor:\n  ros__parameters:\n    lidar_to_body:\n"));
    EXPECT_TRUE(contains(text, "      translation: [0.282000, 0.000000, 0.000000]\n"));
    EXPECT_TRUE(contains(text, "      accel_scale: 1.041401\n"));
    EXPECT_TRUE(contains(text, "    range_offset: 0.1280\n"));
}

// go2_preprocessor reads only the body IMU: the lidar IMU results must not look like parameters.
TEST(ToParameterFile, KeepsTheLidarImuInTheComments)
{
    const std::string text = to_parameter_file(example(), "go2_preprocessor", "a live run", kCreated);

    EXPECT_TRUE(contains(text, "# imu_to_body: "));
    EXPECT_FALSE(contains(text, "\n    imu_to_body:"));
    EXPECT_FALSE(contains(text, "\n    gyro_leak:"));
}

// Row-major, with a decimal point on every value: "1" would parse as an integer, and an array
// must not mix integers and doubles.
TEST(ToParameterFile, WritesRotationsRowMajorAsDoubles)
{
    const std::string text = to_parameter_file(example(), "go2_preprocessor", "a live run", kCreated);

    EXPECT_TRUE(contains(
        text,
        "      rotation: [0.000000000, -1.000000000, 0.000000000, 1.000000000, 0.000000000, 0.000000000, "
        "0.000000000, 0.000000000, 1.000000000]\n"));
}

// A bias of 1e-5 must not come out as "1e-05", which the parser would read as a string.
TEST(ToParameterFile, WritesSmallValuesWithoutExponent)
{
    const std::string text = to_parameter_file(example(), "go2_preprocessor", "a live run", kCreated);

    EXPECT_TRUE(contains(
        text, "    body_imu:\n      gyro_bias:\n        x: 0.000010000\n        y: -0.000002500\n        z: 0.001700000\n"));
}

TEST(ToParameterFile, EveryLineBeforeTheNodeIsAComment)
{
    const std::string text = to_parameter_file(example(), "go2_preprocessor", "bags/calibration_01", kCreated);
    EXPECT_TRUE(contains(text, "# from bags/calibration_01.\n"));

    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line) && line != "go2_preprocessor:") {
        EXPECT_EQ(line.rfind('#', 0), 0U) << line;
    }
    EXPECT_EQ(line, "go2_preprocessor:");
}

TEST(Describe, MarksEveryCheckPassOrFail)
{
    const std::string text = joined(describe(example()));

    EXPECT_TRUE(contains(text, "  PASS matched lidar points: 151577 (at least 1000)\n"));
    EXPECT_TRUE(contains(text, "  FAIL gyro_leak left: 0.2 (at most 0.1)\n"));
    EXPECT_TRUE(contains(text, "range_offset: 0.128 m"));
}

TEST(Describe, ShowsTheBodyAccelerometer)
{
    const std::string text = joined(describe(example()));

    EXPECT_TRUE(contains(text, "|a| 9.420, accel_scale 1.0414, gravity 0.00 deg from +z"));
}

}  // namespace
}  // namespace go2_calibration
