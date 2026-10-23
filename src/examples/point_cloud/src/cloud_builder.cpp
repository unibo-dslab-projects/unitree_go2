#include "point_cloud/cloud_builder.hpp"
#include "point_cloud/cloud_conversion.hpp"
#include "point_cloud/point_cloud.hpp"

#include <stdexcept>

namespace point_cloud {

namespace {

constexpr char kInputTopic[] = "/utlidar/cloud";
constexpr char kOutputTopic[] = "/cloud_builder/cloud";
constexpr int kOutputQueueSize = 10;
constexpr int kLogPeriodMs = 2000;

}  // namespace

CloudBuilder::CloudBuilder() : Node("cloud_builder")
{
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(kOutputTopic, kOutputQueueSize);

    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        kInputTopic,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { on_cloud(message); });
}

void CloudBuilder::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
    PointCloud cloud;

    try {
        cloud = from_message(*message);
    } catch (const std::runtime_error &error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogPeriodMs, "cloud skipped: %s", error.what());
        return;
    }

    // TODO: work on the cloud here (filter, accumulate, ...).

    cloud_publisher_->publish(to_message(cloud));

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kLogPeriodMs, "published cloud with %zu points", cloud.points.size());
}

}  // namespace point_cloud
