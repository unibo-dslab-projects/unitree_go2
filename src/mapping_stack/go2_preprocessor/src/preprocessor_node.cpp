#include "go2_preprocessor/preprocessor_node.hpp"
#include "go2_preprocessor/convertor.hpp"
#include "go2_preprocessor/robot_utils.hpp"

#include <cstddef>
#include <stdexcept>

namespace go2_preprocessor {
    namespace {

    constexpr char kInputTopic[] = "/utlidar/cloud";
    constexpr char kOutputTopic[] = "/utlidar/transformed_cloud";
    // Publisher keeps the default reliable QoS: reliable subscribers (Point-LIO)
    // and best-effort ones (RViz) can both connect to it.
    constexpr int kOutputQueueSize = 10;
    constexpr int kLogPeriodMs = 2000;

    }  // namespace

    PreprocessorNode::PreprocessorNode() : Node("go2_preprocessor")
    {
        cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(kOutputTopic, kOutputQueueSize);

        cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            kInputTopic,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { on_cloud(message); });
    }

    void PreprocessorNode::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
    {
        PointCloud cloud;

        // A broken message is skipped; an uncaught exception would stop the node.
        try {
            cloud = from_message(*message);
        } catch (const std::runtime_error &error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogPeriodMs, "cloud skipped: %s", error.what());
            return;
        }

        const std::size_t points_in = cloud.points.size();

        preprocess_cloud(cloud);

        cloud_publisher_->publish(to_message(cloud));

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), kLogPeriodMs, "points in %zu, out %zu, robot points removed %zu",
            points_in, cloud.points.size(), points_in - cloud.points.size());
    }
}
