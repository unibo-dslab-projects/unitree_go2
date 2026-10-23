#include "map_builder/node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<map_builder::MapBuilderNode>());
    rclcpp::shutdown();
    return 0;
}
