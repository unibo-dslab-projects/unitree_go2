#include "map_builder/cloud_msg.hpp"

#include <stdexcept>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace map_builder {
    std::vector<Point> from_message(const sensor_msgs::msg::PointCloud2 &message) {
        std::vector<Point> points;
        const std::size_t point_count = static_cast<std::size_t>(message.width) * message.height;

        if (point_count == 0) {
            return points;
        }

        if (message.data.size() < point_count * message.point_step) {
            throw std::runtime_error("cloud data is shorter than width * height * point_step");
        }

        points.reserve(point_count);

        sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
        sensor_msgs::PointCloud2ConstIterator<float> intensity(message, "intensity");

        for (std::size_t i = 0; i < point_count; ++i) {
            points.push_back(Point{*x, *y, *z, *intensity});
            ++x;
            ++y;
            ++z;
            ++intensity;
        }

        return points;
    }

    sensor_msgs::msg::PointCloud2 to_message(const std::vector<Point> &points, const std_msgs::msg::Header &header) {
        using sensor_msgs::msg::PointField;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = header;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4,
            "x", 1, PointField::FLOAT32,
            "y", 1, PointField::FLOAT32,
            "z", 1, PointField::FLOAT32,
            "intensity", 1, PointField::FLOAT32
        );
        modifier.resize(points.size());
        cloud.is_dense = true;

        if (points.empty()) {
            return cloud;
        }

        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");

        for (const auto &point : points) {
            *x = point.x;
            *y = point.y;
            *z = point.z;
            *intensity = point.intensity;
            ++x;
            ++y;
            ++z;
            ++intensity;
        }

        return cloud;
    }
}
