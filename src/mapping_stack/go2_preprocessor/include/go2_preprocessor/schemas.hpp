#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace go2_preprocessor {

    struct Point {
        float x = 0.0F;
        float y = 0.0F;
        float z = 0.0F;
        float intensity = 0.0F;
        std::uint16_t ring = 0;
        float time = 0.0F;

        void rotate_y_axis(float angle);
        void shift_z_axis(float shift);
    };

    struct PointCloud {
        std::string frame_id;
        std::int64_t stamp_ns = 0;
        std::vector<Point> points;
    };

}
