// Unit tests for pair_by_time(), fit_imu_rotation(), axis_spread(), mean_and_std() and fit_leak().
#include "go2_calibration/imu_fit.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_calibration {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kMs = 1'000'000;  // nanoseconds

TimedVector at_ms(std::int64_t time_ms, double x)
{
    return TimedVector{time_ms * kMs, Eigen::Vector3d(x, 0.0, 0.0)};
}

TEST(PairByTime, PairsTheNearestSampleWithinTheGap)
{
    const std::vector<TimedVector> first = {at_ms(0, 1.0), at_ms(10, 2.0), at_ms(20, 3.0)};
    const std::vector<TimedVector> second = {at_ms(1, 10.0), at_ms(9, 20.0), at_ms(31, 30.0)};

    // 0 -> 1 ms and 10 -> 9 ms are close enough; 20 has nothing within 2 ms.
    const VectorPairs pairs = pair_by_time(first, second, 0, 2 * kMs);

    ASSERT_EQ(pairs.first.size(), 2U);
    EXPECT_EQ(pairs.first[0].x(), 1.0);
    EXPECT_EQ(pairs.second[0].x(), 10.0);
    EXPECT_EQ(pairs.first[1].x(), 2.0);
    EXPECT_EQ(pairs.second[1].x(), 20.0);
}

// A body sample that arrives 10 ms later pairs only once the lag says so.
TEST(PairByTime, ShiftsTheTargetByTheLag)
{
    const std::vector<TimedVector> first = {at_ms(0, 1.0)};
    const std::vector<TimedVector> second = {at_ms(10, 5.0)};

    EXPECT_EQ(pair_by_time(first, second, 0, 2 * kMs).first.size(), 0U);
    EXPECT_EQ(pair_by_time(first, second, 10 * kMs, 2 * kMs).first.size(), 1U);
}

// A smooth turn, always faster than the 0.15 rad/s threshold.
Eigen::Vector3d turn_rate(double seconds)
{
    return {0.3 * std::sin(2.0 * seconds), 0.2 * std::cos(3.0 * seconds), 1.0 + 0.5 * std::sin(seconds)};
}

// The lidar IMU (250 Hz) reports the turn in its own frame; /sportmodestate (300 Hz) reports it
// in the body frame, 10 ms late. Both rotation and lag must come back.
TEST(FitImuRotation, RecoversTheRotationAndTheLag)
{
    const Eigen::Matrix3d imu_to_body =
        (Eigen::AngleAxisd(165.0 * kPi / 180.0, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(-123.0 * kPi / 180.0, Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();

    std::vector<TimedVector> imu;
    for (std::int64_t t = 0; t < 4000 * kMs; t += 4 * kMs) {
        imu.push_back(TimedVector{t, imu_to_body.transpose() * turn_rate(static_cast<double>(t) / 1e9)});
    }
    std::vector<TimedVector> body;
    for (std::int64_t t = 0; t < 4000 * kMs; t += 3'333'333) {
        body.push_back(TimedVector{t + 10 * kMs, turn_rate(static_cast<double>(t) / 1e9)});
    }

    const auto fit = fit_imu_rotation(imu, body);

    ASSERT_TRUE(fit.has_value());
    EXPECT_EQ(fit->lag_ns, 10 * kMs);
    EXPECT_LT(angle_between_deg(fit->fit.rotation, imu_to_body), 0.2);
}

TEST(FitImuRotation, EmptyWhenTheRobotDoesNotTurn)
{
    const std::vector<TimedVector> still = {at_ms(0, 0.01), at_ms(4, 0.01), at_ms(8, 0.01), at_ms(12, 0.01)};

    EXPECT_FALSE(fit_imu_rotation(still, still).has_value());
}

TEST(FitImuRotation, RejectsANonPositiveLagStep)
{
    ImuRotationSearch search;
    search.lag_step_ns = 0;

    EXPECT_THROW(fit_imu_rotation({}, {}, search), std::invalid_argument);
}

std::vector<TimedVector> rates(const std::vector<Eigen::Vector3d> &values)
{
    std::vector<TimedVector> result;
    for (const Eigen::Vector3d &value : values) {
        result.push_back(TimedVector{0, value});
    }
    return result;
}

// Turns about one axis leave the rotation about it undetermined: the spread is 0.
TEST(AxisSpread, ZeroForTurnsAboutOneAxis)
{
    EXPECT_NEAR(axis_spread(rates({{0.0, 0.0, 0.5}, {0.0, 0.0, -1.0}, {0.0, 0.0, 1.5}})), 0.0, 1e-12);
}

TEST(AxisSpread, OneForTwoAxesTurnedAsMuch)
{
    EXPECT_NEAR(axis_spread(rates({{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}})), 1.0, 1e-12);
}

// Rates below min_rate (0.15 rad/s) are mostly bias and noise, so they do not widen the spread.
TEST(AxisSpread, IgnoresSlowRates)
{
    EXPECT_NEAR(axis_spread(rates({{0.0, 0.0, 1.0}, {0.1, 0.0, 0.0}})), 0.0, 1e-12);
    EXPECT_EQ(axis_spread({}), 0.0);
}

TEST(MeanAndStd, AveragesEveryAxis)
{
    const VectorStats stats = mean_and_std({{1.0, 2.0, 3.0}, {3.0, 2.0, 1.0}});

    EXPECT_EQ(stats.samples, 2U);
    EXPECT_TRUE(stats.mean.isApprox(Eigen::Vector3d(2.0, 2.0, 2.0)));
    EXPECT_TRUE(stats.std_dev.isApprox(Eigen::Vector3d(1.0, 0.0, 1.0)));
}

TEST(MeanAndStd, EmptyGivesZeros)
{
    const VectorStats stats = mean_and_std({});

    EXPECT_EQ(stats.samples, 0U);
    EXPECT_TRUE(stats.mean.isZero());
}

// x reads -0.12 of the yaw rate and y +0.21: the correction is the opposite, as the preprocessor
// adds it (x += z_to_x * z).
TEST(FitLeak, MeasuresTheShareOfYawOnXAndY)
{
    std::vector<Eigen::Vector3d> rates;
    for (int i = -10; i <= 10; ++i) {
        const double z = 0.1 * i;
        rates.emplace_back(-0.12 * z, 0.21 * z, z);
    }

    const GyroLeak leak = fit_leak(rates);

    EXPECT_NEAR(leak.z_to_x, 0.12, 1e-12);
    EXPECT_NEAR(leak.z_to_y, -0.21, 1e-12);
}

// Below min_z_rate the robot barely turns and x and y are mostly noise: those samples are ignored.
TEST(FitLeak, IgnoresSlowTurns)
{
    const GyroLeak leak = fit_leak({{0.5, -0.5, 0.1}, {0.4, 0.3, -0.2}});

    EXPECT_EQ(leak.z_to_x, 0.0);
    EXPECT_EQ(leak.z_to_y, 0.0);
}

}  // namespace
}  // namespace go2_calibration
