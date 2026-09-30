#include "lidar_data/cloud_inspector.hpp"

#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_data::CloudInspector>());
    rclcpp::shutdown();
    return 0;
}
