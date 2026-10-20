// Unit tests for Box::contains().
#include "go2_preprocessor/schemas.hpp"

#include <gtest/gtest.h>

namespace go2_preprocessor {
namespace {

// From -1 to 1 on every axis, so it is easy to see which points fall inside.
constexpr Box kUnitBox{-1.0F, 1.0F, -1.0F, 1.0F, -1.0F, 1.0F};

TEST(BoxContains, PointInTheMiddleIsInside)
{
    EXPECT_TRUE(kUnitBox.contains(Point{0.5F, -0.5F, 0.0F, 0.0F, 0, 0.0F}));
}

// Being out on a single axis is enough: catches || written instead of &&.
TEST(BoxContains, PointOutOnOneAxisIsOutside)
{
    EXPECT_FALSE(kUnitBox.contains(Point{2.0F, 0.0F, 0.0F, 0.0F, 0, 0.0F}));
    EXPECT_FALSE(kUnitBox.contains(Point{0.0F, -2.0F, 0.0F, 0.0F, 0, 0.0F}));
    EXPECT_FALSE(kUnitBox.contains(Point{0.0F, 0.0F, 2.0F, 0.0F, 0, 0.0F}));
}

// Comparisons are strict, as in CMU: a point exactly on a face is outside.
// One point per face, so each of the six comparisons is checked once.
TEST(BoxContains, PointOnAFaceIsOutside)
{
    EXPECT_FALSE(kUnitBox.contains(Point{-1.0F, 0.0F, 0.0F, 0.0F, 0, 0.0F}));
    EXPECT_FALSE(kUnitBox.contains(Point{1.0F, 0.0F, 0.0F, 0.0F, 0, 0.0F}));
    EXPECT_FALSE(kUnitBox.contains(Point{0.0F, -1.0F, 0.0F, 0.0F, 0, 0.0F}));
    EXPECT_FALSE(kUnitBox.contains(Point{0.0F, 1.0F, 0.0F, 0.0F, 0, 0.0F}));
    EXPECT_FALSE(kUnitBox.contains(Point{0.0F, 0.0F, -1.0F, 0.0F, 0, 0.0F}));
    EXPECT_FALSE(kUnitBox.contains(Point{0.0F, 0.0F, 1.0F, 0.0F, 0, 0.0F}));
}

}  // namespace
}  // namespace go2_preprocessor
