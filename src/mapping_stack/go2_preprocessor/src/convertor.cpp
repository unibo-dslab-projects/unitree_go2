#include "go2_preprocessor/schemas.hpp"
#include "go2_preprocessor/convertor.hpp"

#include <cmath>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/detail/point_field__struct.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace go2_preprocessor {
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

        // Copy ROS header metadata (frame ID and timestamp converted to nanoseconds)
        cloud.frame_id = message.header.frame_id;
        cloud.stamp_ns = message.header.stamp.sec * kNanosecondsPerSecond + message.header.stamp.nanosec;

        const std::size_t point_count = static_cast<std::size_t>(message.width) * message.height;

        // Return empty cloud if there are no points
        if (point_count == 0) {
            return cloud;
        }

        // Validate buffer size to prevent out-of-bounds reads
        if (message.data.size() < point_count * message.point_step) {
            throw std::runtime_error("cloud data is shorter than width x height x point_step");
        }

        // Initialize ROS iterators to safely read individual point fields from raw binary data
        sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
        sensor_msgs::PointCloud2ConstIterator<float> intensity(message, "intensity");
        sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(message, "ring");
        sensor_msgs::PointCloud2ConstIterator<float> time(message, "time");

        // Reserve memory to avoid repeated allocations
        cloud.points.reserve(point_count);

        // Iterate through all points, extract field values, and advance iterators
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

        // Populate ROS header metadata from custom cloud structure
        message.header.frame_id = cloud.frame_id;
        message.header.stamp.sec = static_cast<std::int32_t>(cloud.stamp_ns / kNanosecondsPerSecond);
        message.header.stamp.nanosec = static_cast<std::uint32_t>(cloud.stamp_ns % kNanosecondsPerSecond);

        // Configure PointCloud2 layout dimensions and field layout with exact byte offsets
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
        message.is_dense = true;  // Assumed dense initially; set to false if NaN/Inf found
        message.data.resize(message.row_step);

        // Return empty message frame if no points exist
        if (cloud.points.empty()) {
            return message;
        }

        // Initialize ROS iterators to write points into the raw binary data buffer
        sensor_msgs::PointCloud2Iterator<float> x(message, "x");
        sensor_msgs::PointCloud2Iterator<float> y(message, "y");
        sensor_msgs::PointCloud2Iterator<float> z(message, "z");
        sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
        sensor_msgs::PointCloud2Iterator<std::uint16_t> ring(message, "ring");
        sensor_msgs::PointCloud2Iterator<float> time(message, "time");

        // Write each point's data and check for non-finite coordinates
        for (const auto &point : cloud.points) {
            *x = point.x;
            *y = point.y;
            *z = point.z;
            *intensity = point.intensity;
            *ring = point.ring;
            *time = point.time;

            // Mark cloud as non-dense if any coordinate contains NaN or Inf
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
}
