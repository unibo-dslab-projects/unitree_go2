#include "point_cloud/cloud_builder.hpp"

#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<point_cloud::CloudBuilder>());
    rclcpp::shutdown();
    return 0;
}
