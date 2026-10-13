#pragma once

#include "go2_calibration/imu_fit.hpp"
#include "go2_calibration/sequence.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>

namespace go2_calibration {
    // time, intensity and ring of a lidar point, used as key for lidar-body match
    using PointKey = std::tuple<std::uint32_t, std::uint32_t, std::uint16_t>;

    struct ScanPoint {
        PointKey key;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
    };

    struct CalibrationData {
        std::vector<Eigen::Vector3d> still_gyro;     // lidar IMU, lidar IMU frame, rad/s
        std::vector<Eigen::Vector3d> still_accel;    // lidar IMU, m/s^2
        std::vector<Eigen::Vector3d> lidar_points;   // /utlidar/cloud points that have a copy, lidar frame
        std::vector<Eigen::Vector3d> body_points;    // their copies in /utlidar/cloud_base, body frame
        std::vector<Eigen::Vector3d> floor_points;   // every point of those scans, lidar frame -> needed to find the floor
        std::size_t scans = 0;                       // scans matched with their copy
        std::vector<TimedVector> moving_lidar_gyro;  // lidar IMU frame
        std::vector<TimedVector> moving_body_gyro;   // /sportmodestate, body frame
    };

    class CalibrationBuffer {
    public:
        // keeps at most max_scans matched scans
        explicit CalibrationBuffer(std::size_t max_scans = 100);
        // the live source repeats the current phase at 50 Hz: only a change of name counts.
        void set_phase(const std::string &phase_name);

        // /utlidar/imu. time_ns: when the computer received it, the only clock both IMUs share.
        void add_lidar_imu(std::int64_t time_ns, const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel);
        // /sportmodestate imu_state gyroscope, same clock.
        void add_body_gyro(std::int64_t time_ns, const Eigen::Vector3d &gyro);
        // a scan of /utlidar/cloud (body_frame false) or of /utlidar/cloud_base (true). The two
        // copies of a scan share stamp_ns; the second one to arrive matches them.
        void add_scan(std::int64_t stamp_ns, const std::vector<ScanPoint> &points, bool body_frame);

        // True while add_scan() would keep a scan: lets a source skip converting the rest.
        bool wants_scans() const;

        const CalibrationData &data() const;

    private:
        void match(const std::vector<ScanPoint> &lidar, const std::vector<ScanPoint> &body);

        std::size_t max_scans_;
        std::string phase_;
        Use use_ = Use::none;
        CalibrationData data_;
        // Scans waiting for their copy, by stamp; cleared on every phase change.
        std::map<std::int64_t, std::vector<ScanPoint>> waiting_lidar_;
        std::map<std::int64_t, std::vector<ScanPoint>> waiting_body_;
    };
}
