#include <memory>

#include <rclcpp/rclcpp.hpp>

// Placeholder: spins an empty node until the preprocessor node class exists.
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rclcpp::Node>("go2_preprocessor"));
    rclcpp::shutdown();
    return 0;
}
