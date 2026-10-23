#include "point_cloud/cloud_conversion.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace point_cloud {

namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;
constexpr std::uint32_t kPointStep = 32;

sensor_msgs::msg::PointField make_field(const std::string &name, std::uint32_t offset, std::uint8_t datatype)
{
    sensor_msgs::msg::PointField field;

    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;

    return field;
}

bool is_finite(const Point &point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

}  // namespace

PointCloud from_message(const sensor_msgs::msg::PointCloud2 &message)
{
    PointCloud cloud;

    cloud.frame_id = message.header.frame_id;
    cloud.stamp_ns = message.header.stamp.sec * kNanosecondsPerSecond + message.header.stamp.nanosec;

    const std::size_t point_count = static_cast<std::size_t>(message.width) * message.height;

    if (point_count == 0) {
        return cloud;
    }

    if (message.data.size() < point_count * message.point_step) {
        throw std::runtime_error("cloud data is shorter than width x height x point_step");
    }

    sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
    sensor_msgs::PointCloud2ConstIterator<float> intensity(message, "intensity");
    sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(message, "ring");
    sensor_msgs::PointCloud2ConstIterator<float> time(message, "time");

    cloud.points.reserve(point_count);

    for (std::size_t i = 0; i < point_count; ++i) {
        cloud.points.push_back(Point{*x, *y, *z, *intensity, *ring, *time});

        ++x;
        ++y;
        ++z;
        ++intensity;
        ++ring;
        ++time;
    }

    return cloud;
}

sensor_msgs::msg::PointCloud2 to_message(const PointCloud &cloud)
{
    using sensor_msgs::msg::PointField;

    sensor_msgs::msg::PointCloud2 message;

    message.header.frame_id = cloud.frame_id;
    message.header.stamp.sec = static_cast<std::int32_t>(cloud.stamp_ns / kNanosecondsPerSecond);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(cloud.stamp_ns % kNanosecondsPerSecond);

    message.height = 1;
    message.width = static_cast<std::uint32_t>(cloud.points.size());
    message.fields = {
        make_field("x", 0, PointField::FLOAT32),
        make_field("y", 4, PointField::FLOAT32),
        make_field("z", 8, PointField::FLOAT32),
        make_field("intensity", 16, PointField::FLOAT32),
        make_field("ring", 20, PointField::UINT16),
        make_field("time", 24, PointField::FLOAT32),
    };
    message.is_bigendian = false;
    message.point_step = kPointStep;
    message.row_step = message.point_step * message.width;
    message.is_dense = true;
    message.data.resize(message.row_step);

    if (cloud.points.empty()) {
        return message;
    }

    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint16_t> ring(message, "ring");
    sensor_msgs::PointCloud2Iterator<float> time(message, "time");

    for (const auto &point : cloud.points) {
        *x = point.x;
        *y = point.y;
        *z = point.z;
        *intensity = point.intensity;
        *ring = point.ring;
        *time = point.time;

        if (!is_finite(point)) {
            message.is_dense = false;
        }

        ++x;
        ++y;
        ++z;
        ++intensity;
        ++ring;
        ++time;
    }

    return message;
}

}  // namespace point_cloud
