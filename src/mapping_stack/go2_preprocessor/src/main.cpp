#include "go2_preprocessor/preprocessor_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<go2_preprocessor::PreprocessorNode>());
    rclcpp::shutdown();
    return 0;
}
