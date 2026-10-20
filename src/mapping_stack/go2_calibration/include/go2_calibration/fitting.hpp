#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace go2_calibration {
    struct RotationFit {
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        double rms = 0.0;  // root mean square of |to - R * from|, in the unit of the vectors
        std::size_t samples = 0;
    };

    struct RigidFit {
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        double rms = 0.0;        // metres
        double max_error = 0.0;  // metres
        std::size_t samples = 0;
    };

    // Rotation R that best maps every `from` vector onto its `to` vector (to = R * from), for
    // directions such as angular rates: there is no translation, so nothing is centred.
    // Throws std::invalid_argument if the sizes differ or there are fewer than 3 pairs.
    RotationFit fit_rotation(const std::vector<Eigen::Vector3d> &from, const std::vector<Eigen::Vector3d> &to);

    // Rotation and translation that best map every `from` point onto its `to` point
    // (to = R * from + t). Same checks as fit_rotation.
    RigidFit fit_rigid(const std::vector<Eigen::Vector3d> &from, const std::vector<Eigen::Vector3d> &to);

    // Angle of the rotation that takes a onto b, in degrees.
    double angle_between_deg(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b);
}
