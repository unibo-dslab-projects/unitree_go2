#pragma once

#include "go2_calibration/buffer.hpp"
#include "go2_calibration/fitting.hpp"
#include "go2_calibration/floor.hpp"
#include "go2_calibration/imu_fit.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace go2_calibration {
    // quality limits. A result outside its limit fails its check, and a failed calibration writes
    // no parameter file. Defaults: our Go2 on 2026-10-13 (value in brackets) with a wide margin.
    struct Limits {
        double min_matched_points = 1000.0;  // lidar points paired with their copy (151577)
        double max_mount_error_mm = 1.0;     // lidar_to_body error rms: the firmware's own copy (0.000)
        double max_floor_tilt_deg = 2.0;     // floor normal after lidar_to_body against +z (0.75)
        double max_gyro_error = 0.25;        // imu_to_body error rms, rad/s (0.12)
        double min_axis_spread = 0.01;       // turns about more than one axis (0.040)
        double max_still_noise = 0.02;       // largest std of the still gyroscope, rad/s (0.007)
        double max_leak = 0.1;               // largest |gyro_leak| left after imu_to_body (0.034)
    };

    // one quality check: value against limit.
    struct Check {
        std::string name;
        double value = 0.0;
        double limit = 0.0;
        bool at_least = false;  // true: value >= limit passes; false: value <= limit passes

        bool passed() const;
    };

    struct Calibration {
        // lidar mount, robot still: /utlidar/cloud against /utlidar/cloud_base. p_body = R p_lidar + t.
        RigidFit lidar_to_body;
        std::size_t scans = 0;
        Plane floor;                  // lidar frame, found with the range offset still in it
        double floor_tilt_deg = 0.0;  // floor normal after lidar_to_body, against +z
        RangeOffset range;            // lidar range error, from the bending of the floor

        // lidar IMU, robot turning and tilting: its gyroscope against the body gyroscope.
        ImuRotationFit imu_to_body;
        double axis_spread = 0.0;
        double imu_to_lidar_deg = 0.0;  // angle between imu_to_body and lidar_to_body

        VectorStats gyro_bias;          // body frame, robot still
        GyroLeak leak;                  // what imu_to_body and the bias leave of the yaw rate on x, y
        double gravity_tilt_deg = 0.0;  // lidar accelerometer after imu_to_body, against +z

        std::vector<Check> checks;

        bool passed() const;  // every check passed
    };

    /// The whole calibration from the buffer's data: lidar mount and floor, range offset, IMU
    /// rotation, bias and leak, then the checks against `limits`. Throws std::runtime_error naming
    /// what is missing: no matched lidar point, no floor, no turning, no still gyroscope sample.
    Calibration calibrate(const CalibrationData &data, const Limits &limits = Limits{});
}
