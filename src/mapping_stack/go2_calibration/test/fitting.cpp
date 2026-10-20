// Unit tests for fit_rotation(), fit_rigid() and angle_between_deg().
#include "go2_calibration/fitting.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_calibration {
namespace {

constexpr double kPi = 3.14159265358979323846;
// angle_between_deg() takes an arc cosine, which turns rounding of 1e-16 into about 1e-6 deg.
constexpr double kAngleToleranceDeg = 1e-4;

double radians(double degrees)
{
    return degrees * kPi / 180.0;
}

// The lidar-to-body rotation of our Go2, from the firmware's own transform (2026-09-28 bags):
// a -123 deg turn around z, then 165 deg around y.
Eigen::Matrix3d go2_lidar_to_body()
{
    return (Eigen::AngleAxisd(radians(165.0), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(radians(-123.0), Eigen::Vector3d::UnitZ()))
        .toRotationMatrix();
}

// Spread in every direction, so that no rotation axis is left undetermined.
std::vector<Eigen::Vector3d> spread_vectors()
{
    return {{1.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 0.5}, {1.0, 1.0, 1.0}, {-0.3, 0.8, -1.2}, {2.0, -1.0, 0.4}};
}

std::vector<Eigen::Vector3d> transformed(
    const std::vector<Eigen::Vector3d> &vectors, const Eigen::Matrix3d &rotation,
    const Eigen::Vector3d &translation = Eigen::Vector3d::Zero())
{
    std::vector<Eigen::Vector3d> result;
    for (const Eigen::Vector3d &vector : vectors) {
        result.push_back(rotation * vector + translation);
    }
    return result;
}

TEST(FitRotation, RecoversAKnownRotation)
{
    const std::vector<Eigen::Vector3d> from = spread_vectors();

    const RotationFit fit = fit_rotation(from, transformed(from, go2_lidar_to_body()));

    EXPECT_LT(angle_between_deg(fit.rotation, go2_lidar_to_body()), kAngleToleranceDeg);
    EXPECT_NEAR(fit.rms, 0.0, 1e-12);
    EXPECT_EQ(fit.samples, from.size());
}

// A mirror image (z flipped) is no rotation: the fit must still return a proper rotation
// (determinant +1) and show the mismatch in rms. A plain SVD without the sign fix fails this.
TEST(FitRotation, NeverReturnsAMirrorImage)
{
    const std::vector<Eigen::Vector3d> from = spread_vectors();
    const Eigen::Matrix3d mirror = Eigen::Vector3d(1.0, 1.0, -1.0).asDiagonal();

    const RotationFit fit = fit_rotation(from, transformed(from, mirror));

    EXPECT_NEAR(fit.rotation.determinant(), 1.0, 1e-9);
    EXPECT_GT(fit.rms, 0.1);
}

TEST(FitRotation, RejectsUnequalOrTooFewPairs)
{
    const std::vector<Eigen::Vector3d> two = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    const std::vector<Eigen::Vector3d> three = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

    EXPECT_THROW(fit_rotation(two, two), std::invalid_argument);
    EXPECT_THROW(fit_rotation(three, two), std::invalid_argument);
    EXPECT_THROW(fit_rigid(three, two), std::invalid_argument);
}

// The firmware's transform: the rotation, and the lidar 0.282 m ahead of the body origin.
TEST(FitRigid, RecoversRotationAndTranslation)
{
    const std::vector<Eigen::Vector3d> from = spread_vectors();
    const Eigen::Vector3d translation(0.282, 0.0, 0.0);

    const RigidFit fit = fit_rigid(from, transformed(from, go2_lidar_to_body(), translation));

    EXPECT_LT(angle_between_deg(fit.rotation, go2_lidar_to_body()), kAngleToleranceDeg);
    EXPECT_LT((fit.translation - translation).norm(), 1e-9);
    EXPECT_NEAR(fit.rms, 0.0, 1e-9);
    EXPECT_NEAR(fit.max_error, 0.0, 1e-9);
}

TEST(AngleBetween, QuarterTurnIsNinetyDegrees)
{
    const Eigen::Matrix3d quarter = Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    EXPECT_NEAR(angle_between_deg(Eigen::Matrix3d::Identity(), quarter), 90.0, 1e-9);
    EXPECT_NEAR(angle_between_deg(quarter, quarter), 0.0, kAngleToleranceDeg);
}

}  // namespace
}  // namespace go2_calibration
