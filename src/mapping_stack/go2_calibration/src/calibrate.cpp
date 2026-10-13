#include "go2_calibration/calibrate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2_calibration {
    namespace {
    Check at_least(const std::string &name, double value, double limit)
    {
        return Check{name, value, limit, true};
    }

    Check at_most(const std::string &name, double value, double limit)
    {
        return Check{name, value, limit, false};
    }
    }  // namespace

    bool Check::passed() const
    {
        return at_least ? value >= limit : value <= limit;
    }

    bool Calibration::passed() const
    {
        return std::all_of(checks.begin(), checks.end(), [](const Check &check) { return check.passed(); });
    }

    Calibration calibrate(const CalibrationData &data, const Limits &limits)
    {
        if (data.lidar_points.size() < 3) {
            throw std::runtime_error("no /utlidar/cloud point matched /utlidar/cloud_base while the robot stood still");
        }
        if (data.still_gyro.empty()) {
            throw std::runtime_error("no /utlidar/imu sample while the robot stood still");
        }

        Calibration result;

        // 1. lidar mount, and the floor as an independent check of it.
        result.lidar_to_body = fit_rigid(data.lidar_points, data.body_points);
        result.scans = data.scans;
        const auto floor = find_floor(data.floor_points);
        if (!floor) {
            throw std::runtime_error("no floor 0.2 to 0.7 m from the lidar: stand the robot on a flat floor");
        }
        result.floor = *floor;
        result.floor_tilt_deg = tilt_from_up_deg(result.lidar_to_body.rotation * floor->normal);
        result.range = fit_range_offset(data.floor_points, *floor);

        // 2. Lidar IMU rotation, from the turns and tilts.
        const auto rotation = fit_imu_rotation(data.moving_lidar_gyro, data.moving_body_gyro);
        if (!rotation) {
            throw std::runtime_error("too little turning to fit the IMU rotation (needs the gyroscope above 0.15 rad/s)");
        }
        result.imu_to_body = *rotation;
        const Eigen::Matrix3d &imu_to_body = rotation->fit.rotation;
        result.axis_spread = axis_spread(data.moving_lidar_gyro);
        result.imu_to_lidar_deg = angle_between_deg(imu_to_body, result.lidar_to_body.rotation);

        // 3. Bias and gravity, robot still, in the body frame.
        std::vector<Eigen::Vector3d> still_rates;
        for (const Eigen::Vector3d &rate : data.still_gyro) {
            still_rates.push_back(imu_to_body * rate);
        }
        result.gyro_bias = mean_and_std(still_rates);
        result.gravity_tilt_deg = tilt_from_up_deg(imu_to_body * mean_and_std(data.still_accel).mean);

        // 4. Cross-talk left once rotation and bias are taken out.
        std::vector<Eigen::Vector3d> moving_rates;
        for (const TimedVector &rate : data.moving_lidar_gyro) {
            moving_rates.push_back(imu_to_body * rate.value - result.gyro_bias.mean);
        }
        result.leak = fit_leak(moving_rates);

        // 5. Checks.
        result.checks = {
            at_least("matched lidar points", static_cast<double>(result.lidar_to_body.samples), limits.min_matched_points),
            at_most("lidar_to_body error rms, mm", result.lidar_to_body.rms * 1000.0, limits.max_mount_error_mm),
            at_most("floor tilt after lidar_to_body, deg", result.floor_tilt_deg, limits.max_floor_tilt_deg),
            at_most("imu_to_body error rms, rad/s", result.imu_to_body.fit.rms, limits.max_gyro_error),
            at_least("turn axis spread", result.axis_spread, limits.min_axis_spread),
            at_most("still gyroscope noise, rad/s", result.gyro_bias.std_dev.maxCoeff(), limits.max_still_noise),
            at_most(
                "gyro_leak left", std::max(std::abs(result.leak.z_to_x), std::abs(result.leak.z_to_y)), limits.max_leak),
        };
        return result;
    }
}
