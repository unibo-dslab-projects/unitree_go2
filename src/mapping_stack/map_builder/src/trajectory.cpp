#include "map_builder/trajectory.hpp"

#include <iomanip>

namespace map_builder {
    namespace {
        constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;
    } // namespace

    bool Trajectory::add(const Pose &pose)
    {
        if (!poses_.empty() && pose.stamp_ns <= poses_.back().stamp_ns) {
            return false;
        }
        poses_.push_back(pose);
        return true;
    }

    const std::vector<Pose> &Trajectory::poses() const
    {
        return poses_;
    }

    void Trajectory::write_tum(std::ostream &out) const
    {
        out << std::fixed << std::setprecision(6);
        for (const auto &pose : poses_) {
            out << pose.stamp_ns / kNanosecondsPerSecond << "." << std::setw(9) << std::setfill('0')
                << pose.stamp_ns % kNanosecondsPerSecond << std::setfill(' ') << ' '
                << pose.x << " " << pose.y << " " << pose.z << " "
                << pose.qx << " " << pose.qy << " " << pose.qz << " " << pose.qw << "\n";
        }
    }
} // namespace map_builder
