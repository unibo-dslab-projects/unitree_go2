#pragma once

// Test helper: a cloud with the Go2 L1 layout, 32 bytes a point: x, y, z (float32) at 0, 4, 8,
// intensity (float32) at 16, ring (uint16) at 20, time (float32) at 24.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace go2_calibration::test {
struct Go2Point {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    std::uint16_t ring = 0;
    float time = 0.0F;
};

inline sensor_msgs::msg::PointCloud2 go2_cloud(const std::vector<Go2Point> &points, bool with_ring = true)
{
    using sensor_msgs::msg::PointField;
    const auto field = [](const std::string &name, std::uint32_t offset, std::uint8_t datatype) {
        PointField result;
        result.name = name;
        result.offset = offset;
        result.datatype = datatype;
        result.count = 1;
        return result;
    };

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.fields = {
        field("x", 0, PointField::FLOAT32), field("y", 4, PointField::FLOAT32), field("z", 8, PointField::FLOAT32),
        field("intensity", 16, PointField::FLOAT32), field("time", 24, PointField::FLOAT32)};
    if (with_ring) {
        cloud.fields.push_back(field("ring", 20, PointField::UINT16));
    }
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.point_step = 32;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = true;
    cloud.data.assign(cloud.row_step, 0);

    for (std::size_t i = 0; i < points.size(); ++i) {
        std::uint8_t *bytes = &cloud.data[i * cloud.point_step];
        std::memcpy(bytes + 0, &points[i].x, 4);
        std::memcpy(bytes + 4, &points[i].y, 4);
        std::memcpy(bytes + 8, &points[i].z, 4);
        std::memcpy(bytes + 16, &points[i].intensity, 4);
        std::memcpy(bytes + 20, &points[i].ring, 2);
        std::memcpy(bytes + 24, &points[i].time, 4);
    }
    return cloud;
}
}  // namespace go2_calibration::test
