#include "lidar_data/cloud_inspector.hpp"

namespace lidar_data {

namespace {

constexpr char kCloudTopic[] = "/utlidar/cloud";

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
    // TODO: log the cloud layout (frame_id, width, height, point_step, row_step, is_dense, fields).
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "cloud received: %zu bytes", cloud->data.size());
}

}  // namespace lidar_data
