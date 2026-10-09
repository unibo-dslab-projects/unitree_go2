// Unit tests for Point::rotate_y_axis() and Point::shift_z_axis().
#include "go2_preprocessor/schemas.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace go2_preprocessor {
namespace {

constexpr float kPi = 3.14159265F;
// Float sin/cos of exact angles are off in the last digits (cos(pi/2) is about
// -4e-8, not 0), so compare computed coordinates within a small margin.
constexpr float kTolerance = 1e-6F;

// Right-hand rule around +y: a quarter turn carries +x down to -z.
TEST(RotateYAxis, QuarterTurnMovesXOntoMinusZ)
{
    Point point{1.0F, 0.0F, 0.0F, 0.0F, 0, 0.0F};

    point.rotate_y_axis(kPi / 2.0F);

    EXPECT_NEAR(point.x, 0.0F, kTolerance);
    EXPECT_NEAR(point.y, 0.0F, kTolerance);
    EXPECT_NEAR(point.z, -1.0F, kTolerance);
}

TEST(RotateYAxis, QuarterTurnMovesZOntoPlusX)
{
    Point point{0.0F, 0.0F, 1.0F, 0.0F, 0, 0.0F};

    point.rotate_y_axis(kPi / 2.0F);

    EXPECT_NEAR(point.x, 1.0F, kTolerance);
    EXPECT_NEAR(point.z, 0.0F, kTolerance);
}

TEST(RotateYAxis, ZeroAngleChangesNothing)
{
    Point point{1.5F, -2.0F, 3.25F, 0.0F, 0, 0.0F};

    point.rotate_y_axis(0.0F);

    // cos(0) and sin(0) are exact, so the values must be unchanged.
    EXPECT_FLOAT_EQ(point.x, 1.5F);
    EXPECT_FLOAT_EQ(point.y, -2.0F);
    EXPECT_FLOAT_EQ(point.z, 3.25F);
}

// A rotation turns a point around the axis without moving it closer or farther:
// its distance from the y axis must stay the same for any angle.
TEST(RotateYAxis, KeepsDistanceFromTheAxis)
{
    Point point{3.0F, 7.0F, 4.0F, 0.0F, 0, 0.0F};  // distance from the y axis: 5

    point.rotate_y_axis(0.7F);

    EXPECT_NEAR(std::hypot(point.x, point.z), 5.0F, 1e-5F);
    EXPECT_FLOAT_EQ(point.y, 7.0F);
}

TEST(RotateYAxis, LeavesTheOtherFieldsAlone)
{
    Point point{1.0F, 2.0F, 3.0F, 40.0F, 5, 0.06F};

    point.rotate_y_axis(1.0F);

    EXPECT_FLOAT_EQ(point.intensity, 40.0F);
    EXPECT_EQ(point.ring, 5U);
    EXPECT_FLOAT_EQ(point.time, 0.06F);
}

TEST(ShiftZAxis, AddsTheShiftToZOnly)
{
    Point point{1.0F, 2.0F, 3.0F, 40.0F, 5, 0.06F};

    point.shift_z_axis(-0.5F);

    EXPECT_FLOAT_EQ(point.x, 1.0F);
    EXPECT_FLOAT_EQ(point.y, 2.0F);
    EXPECT_FLOAT_EQ(point.z, 2.5F);
}

}  // namespace
}  // namespace go2_preprocessor
