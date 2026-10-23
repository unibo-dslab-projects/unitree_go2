#include "go2_calibration/imu_fit.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

#include <Eigen/Eigenvalues>

namespace go2_calibration {
    VectorPairs pair_by_time(
        const std::vector<TimedVector> &first, const std::vector<TimedVector> &second, std::int64_t lag_ns,
        std::int64_t max_gap_ns)
    {
        VectorPairs pairs;

        for (const TimedVector &sample : first) {
            const std::int64_t target = sample.time_ns + lag_ns;

            // The first sample of `second` at or after the target; the nearest is it or the one before.
            const auto after = std::lower_bound(
                second.begin(), second.end(), target,
                [](const TimedVector &candidate, std::int64_t time) { return candidate.time_ns < time; });

            auto nearest = second.end();
            std::int64_t gap = max_gap_ns + 1;
            if (after != second.end()) {
                nearest = after;
                gap = after->time_ns - target;
            }
            if (after != second.begin()) {
                const auto before = std::prev(after);
                if (target - before->time_ns < gap) {
                    nearest = before;
                    gap = target - before->time_ns;
                }
            }

            if (nearest != second.end() && gap <= max_gap_ns) {
                pairs.first.push_back(sample.value);
                pairs.second.push_back(nearest->value);
            }
        }

        return pairs;
    }

    std::optional<ImuRotationFit> fit_imu_rotation(
        const std::vector<TimedVector> &imu_gyro, const std::vector<TimedVector> &body_gyro,
        const ImuRotationSearch &search)
    {
        if (search.lag_step_ns <= 0) {
            throw std::invalid_argument("fit_imu_rotation: lag_step_ns must be positive");
        }

        // The rotation turns vectors without changing their length, so the lidar IMU rate alone
        // tells whether the robot turns.
        std::vector<TimedVector> turning;
        std::copy_if(imu_gyro.begin(), imu_gyro.end(), std::back_inserter(turning), [&](const TimedVector &sample) {
            return sample.value.norm() >= search.min_rate;
        });

        std::optional<ImuRotationFit> best;
        for (std::int64_t lag = -search.max_lag_ns; lag <= search.max_lag_ns; lag += search.lag_step_ns) {
            const VectorPairs pairs = pair_by_time(turning, body_gyro, lag, search.max_gap_ns);
            if (pairs.first.size() < 3) {
                continue;
            }
            const RotationFit fit = fit_rotation(pairs.first, pairs.second);
            if (!best || fit.rms < best->fit.rms) {
                best = ImuRotationFit{fit, lag};
            }
        }

        return best;
    }

    double axis_spread(const std::vector<TimedVector> &rates, double min_rate)
    {
        Eigen::Matrix3d sum = Eigen::Matrix3d::Zero();
        for (const TimedVector &rate : rates) {
            if (rate.value.norm() >= min_rate) {
                sum += rate.value * rate.value.transpose();
            }
        }
        // Eigenvalues of a symmetric matrix, in increasing order.
        const Eigen::Vector3d eigenvalues = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(sum).eigenvalues();
        return eigenvalues(2) > 0.0 ? eigenvalues(1) / eigenvalues(2) : 0.0;
    }

    VectorStats mean_and_std(const std::vector<Eigen::Vector3d> &values)
    {
        VectorStats stats;
        stats.samples = values.size();
        if (values.empty()) {
            return stats;
        }
        const double count = static_cast<double>(values.size());

        for (const Eigen::Vector3d &value : values) {
            stats.mean += value;
        }
        stats.mean /= count;

        for (const Eigen::Vector3d &value : values) {
            stats.std_dev += (value - stats.mean).cwiseAbs2();
        }
        stats.std_dev = (stats.std_dev / count).cwiseSqrt();

        return stats;
    }

    GyroLeak fit_leak(const std::vector<Eigen::Vector3d> &body_rates, double min_z_rate)
    {
        // Least squares for x = -z_to_x * z gives z_to_x = -sum(x z) / sum(z z); the same for y.
        double xz = 0.0;
        double yz = 0.0;
        double zz = 0.0;
        for (const Eigen::Vector3d &rate : body_rates) {
            if (std::abs(rate.z()) < min_z_rate) {
                continue;
            }
            xz += rate.x() * rate.z();
            yz += rate.y() * rate.z();
            zz += rate.z() * rate.z();
        }

        GyroLeak leak;
        if (zz > 0.0) {
            leak.z_to_x = -xz / zz;
            leak.z_to_y = -yz / zz;
        }
        return leak;
    }
}
