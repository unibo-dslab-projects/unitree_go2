#include "map_builder/node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<map_builder::MapBuilderNode>();
    rclcpp::spin(node);
    // spin returns on Ctrl-C; the map is still in memory here
    node->save_at_exit();
    rclcpp::shutdown();
    return 0;
}
