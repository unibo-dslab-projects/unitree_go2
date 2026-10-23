#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace point_cloud {

// One lidar return, with the same six values the Go2 publishes for each point.
struct Point {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    std::uint16_t ring = 0;
    float time = 0.0F;
};

struct PointCloud {
    std::string frame_id;
    // Nanoseconds since the Unix epoch.
    std::int64_t stamp_ns = 0;
    std::vector<Point> points;
};

}  // namespace point_cloud
