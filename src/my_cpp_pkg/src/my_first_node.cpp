#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv) {
    
    // Initialize ROS2 communication.
    rclcpp::init(argc, argv);

    // Create Node
    auto node = std::make_shared<rclcpp::Node>("cpp_test");

    // Write a message from this node
    RCLCPP_INFO(node->get_logger(), "Hello cpp Node");

    // Spin the node so program stays running
    rclcpp::spin(node);

    // Shutdown program
    rclcpp::shutdown();
    
    return 0;
}