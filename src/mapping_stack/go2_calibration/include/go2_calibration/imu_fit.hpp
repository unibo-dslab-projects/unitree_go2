#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "go2_calibration/fitting.hpp"

namespace go2_calibration {
    struct TimedVector {
        std::int64_t time_ns = 0;
        Eigen::Vector3d value = Eigen::Vector3d::Zero();
    };

    struct VectorPairs {
        std::vector<Eigen::Vector3d> first;
        std::vector<Eigen::Vector3d> second;
    };

    // Pairs every sample of `first` with the sample of `second` nearest to its time + lag_ns, when
    // that one is at most max_gap_ns away. Both streams must be sorted by time, as bags are.
    VectorPairs pair_by_time(
        const std::vector<TimedVector> &first, const std::vector<TimedVector> &second, std::int64_t lag_ns,
        std::int64_t max_gap_ns);

    struct ImuRotationSearch {
        double min_rate = 0.15;               // rad/s: slower turns are mostly bias and noise
        std::int64_t max_gap_ns = 3'000'000;  // the two IMUs publish at about 250 and 300 Hz
        std::int64_t max_lag_ns = 40'000'000;
        std::int64_t lag_step_ns = 5'000'000;
    };

    struct ImuRotationFit {
        RotationFit fit;
        std::int64_t lag_ns = 0;  // how much later the body sample of a pair arrived
    };

    // Rotation from the lidar IMU frame to the body frame: fits the lidar IMU gyroscope to the body
    // gyroscope of /sportmodestate over the samples where the robot turns, for every lag from
    // -max_lag_ns to max_lag_ns, and keeps the lag with the smallest error. Empty if no lag gives
    // 3 pairs. Throws std::invalid_argument if lag_step_ns is not positive.
    std::optional<ImuRotationFit> fit_imu_rotation(
        const std::vector<TimedVector> &imu_gyro, const std::vector<TimedVector> &body_gyro,
        const ImuRotationSearch &search = ImuRotationSearch{});

    struct VectorStats {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d std_dev = Eigen::Vector3d::Zero();
        std::size_t samples = 0;
    };

    // How much the turns spread over more than one axis: the second largest over the largest
    // eigenvalue of sum(w w^T) for the rates of at least min_rate. Near 0 when every turn is about
    // one axis, and then the rotation about that axis is left undetermined by fit_imu_rotation().
    // 0 when no rate reaches min_rate.
    double axis_spread(const std::vector<TimedVector> &rates, double min_rate = 0.15);

    VectorStats mean_and_std(const std::vector<Eigen::Vector3d> &values);

    // Named as the go2_preprocessor parameters: it adds x += z_to_x * z and y += z_to_y * z.
    struct GyroLeak {
        double z_to_x = 0.0;
        double z_to_y = 0.0;
    };

    // Share of the yaw rate (z) that still shows up on x and y of body-frame rates, as the
    // correction that removes it. Uses the samples with |z| >= min_z_rate only; zero if none.
    GyroLeak fit_leak(const std::vector<Eigen::Vector3d> &body_rates, double min_z_rate = 0.3);
}
