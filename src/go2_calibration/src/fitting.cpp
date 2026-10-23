#include "go2_calibration/fitting.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/LU>   // determinant()
#include <Eigen/SVD>

namespace go2_calibration {
    namespace {
    constexpr double kPi = 3.14159265358979323846;

    void check_pairs(const std::vector<Eigen::Vector3d> &from, const std::vector<Eigen::Vector3d> &to)
    {
        if (from.size() != to.size()) {
            throw std::invalid_argument("fit: from and to have different sizes");
        }
        if (from.size() < 3) {
            throw std::invalid_argument("fit: needs at least 3 pairs");
        }
    }

    // kabsch: with sum(from * to^T) = U S V^T, the best rotation is V U^T. diag(1, 1, d) flips the
    // last axis when V U^T is a mirror image (determinant -1) instead of a rotation.
    Eigen::Matrix3d best_rotation(const Eigen::Matrix3d &covariance)
    {
        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const double d = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
        return svd.matrixV() * Eigen::Vector3d(1.0, 1.0, d).asDiagonal() * svd.matrixU().transpose();
    }
    }  // namespace

    /// Compute the best rotation fit between two sets of 3D vectors using the Kabsch algorithm.
    RotationFit fit_rotation(const std::vector<Eigen::Vector3d> &from, const std::vector<Eigen::Vector3d> &to)
    {
        check_pairs(from, to);

        // compute covariance matrix
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < from.size(); ++i) {
            covariance += from[i] * to[i].transpose();
        }

        RotationFit fit;
        fit.rotation = best_rotation(covariance);
        fit.samples = from.size();

        double squared_sum = 0.0;
        for (std::size_t i = 0; i < from.size(); ++i) {
            squared_sum += (to[i] - fit.rotation * from[i]).squaredNorm();
        }
        fit.rms = std::sqrt(squared_sum / static_cast<double>(from.size()));

        return fit;
    }

    RigidFit fit_rigid(const std::vector<Eigen::Vector3d> &from, const std::vector<Eigen::Vector3d> &to)
    {
        check_pairs(from, to);
        const double count = static_cast<double>(from.size());

        // the translation only moves the centroids, so the rotation is fitted on centred points.
        // a translation is a rigid body transformation that does not change the orientation (offset).
        Eigen::Vector3d from_centroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d to_centroid = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < from.size(); ++i) {
            from_centroid += from[i];
            to_centroid += to[i];
        }
        from_centroid /= count;
        to_centroid /= count;

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < from.size(); ++i) {
            covariance += (from[i] - from_centroid) * (to[i] - to_centroid).transpose();
        }

        RigidFit fit;
        fit.rotation = best_rotation(covariance);
        fit.translation = to_centroid - fit.rotation * from_centroid;
        fit.samples = from.size();

        double squared_sum = 0.0;
        for (std::size_t i = 0; i < from.size(); ++i) {
            const double error = (to[i] - (fit.rotation * from[i] + fit.translation)).norm();
            squared_sum += error * error;
            fit.max_error = std::max(fit.max_error, error);
        }
        fit.rms = std::sqrt(squared_sum / count);

        return fit;
    }

    double angle_between_deg(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b)
    {
        // a rotation by angle x has trace 1 + 2 cos(x); the clamp absorbs rounding just past +-1.
        const double cosine = std::clamp(((a.transpose() * b).trace() - 1.0) / 2.0, -1.0, 1.0);
        return std::acos(cosine) * 180.0 / kPi;
    }
}
