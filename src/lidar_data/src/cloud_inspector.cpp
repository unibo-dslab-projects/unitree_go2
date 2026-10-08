#include "lidar_data/cloud_inspector.hpp"
#include "lidar_data/cloud_layout.hpp"

namespace lidar_data {

namespace {

constexpr char kCloudTopic[] = "/utlidar/cloud";
constexpr int kLogPeriodMs = 2000;

}  // namespace

CloudInspector::CloudInspector() : Node("cloud_inspector")
{
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        kCloudTopic,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { on_cloud(cloud); });
}

void CloudInspector::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
    // logging macro that limits how often a message is printed
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kLogPeriodMs, "%s", describe_layout(*cloud).c_str());
}

}  // namespace lidar_data
