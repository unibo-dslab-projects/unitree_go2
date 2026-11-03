#include "vcortex_bridge/node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vcortex_bridge::BridgeNode>());
    rclcpp::shutdown();
    return 0;
}
