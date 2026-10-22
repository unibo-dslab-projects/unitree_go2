#pragma once

#include <cstdint>
#include <ostream>
#include <vector>

namespace map_builder {
    struct Pose {
        std::int64_t stamp_ns = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
    };

    class Trajectory {
    public:
        bool add(const Pose &pose);
        const std::vector<Pose> &poses() const;
        void write_tum(std::ostream &out) const;

    private:
        std::vector<Pose> poses_;
    };
} // namespace map_builder
