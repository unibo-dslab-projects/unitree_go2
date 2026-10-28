#include "go2_calibration/buffer.hpp"

#include <cstddef>

namespace go2_calibration {
    CalibrationBuffer::CalibrationBuffer(std::size_t max_scans) : max_scans_(max_scans) {}

    void CalibrationBuffer::set_phase(const std::string &phase_name)
    {
        if (phase_name == phase_) {
            return;
        }
        phase_ = phase_name;
        use_ = use_of(phase_name);
        // A scan whose copy has not arrived by now never will be matched.
        waiting_lidar_.clear();
        waiting_body_.clear();
    }

    void CalibrationBuffer::add_lidar_imu(std::int64_t time_ns, const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel)
    {
        if (use_ == Use::still) {
            data_.still_gyro.push_back(gyro);
            data_.still_accel.push_back(accel);
        } else if (use_ == Use::moving) {
            data_.moving_lidar_gyro.push_back(TimedVector{time_ns, gyro});
        }
    }

    void CalibrationBuffer::add_body_imu(std::int64_t time_ns, const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel)
    {
        if (use_ == Use::still) {
            data_.still_body_gyro.push_back(gyro);
            data_.still_body_accel.push_back(accel);
        } else if (use_ == Use::moving) {
            data_.moving_body_gyro.push_back(TimedVector{time_ns, gyro});
        }
    }

    void CalibrationBuffer::add_scan(std::int64_t stamp_ns, const std::vector<ScanPoint> &points, bool body_frame)
    {
        if (!wants_scans()) {
            return;
        }

        // the first copy of a scan waits here for the other one.
        auto &own = body_frame ? waiting_body_ : waiting_lidar_;
        auto &other = body_frame ? waiting_lidar_ : waiting_body_;
        const auto partner = other.find(stamp_ns);
        if (partner == other.end()) {
            own[stamp_ns] = points;
            return;
        }

        match(body_frame ? partner->second : points, body_frame ? points : partner->second);
        other.erase(partner);
        ++data_.scans;
    }

    /// Returns if the buffer wants more scans, i.e. it is still calibrating.
    bool CalibrationBuffer::wants_scans() const
    {
        return use_ == Use::still && data_.scans < max_scans_;
    }

    const CalibrationData &CalibrationBuffer::data() const
    {
        return data_;
    }

    void CalibrationBuffer::match(const std::vector<ScanPoint> &lidar, const std::vector<ScanPoint> &body)
    {
        // where each key sits in the lidar scan; -1 marks a key seen twice, which cannot be matched.
        std::map<PointKey, std::ptrdiff_t> index;
        for (std::size_t i = 0; i < lidar.size(); ++i) {
            data_.floor_points.push_back(lidar[i].position);
            const auto [entry, inserted] = index.emplace(lidar[i].key, static_cast<std::ptrdiff_t>(i));
            if (!inserted) {
                entry->second = -1;
            }
        }

        for (const ScanPoint &point : body) {
            const auto entry = index.find(point.key);
            if (entry == index.end() || entry->second < 0) {
                continue;
            }
            data_.lidar_points.push_back(lidar[static_cast<std::size_t>(entry->second)].position);
            data_.body_points.push_back(point.position);
        }
    }
}
