#include "rclcpp/rclcpp.hpp"


class MyNode: public rclcpp::Node{

public:
    MyNode(): Node("cpp_test_oop"), counter_{0}  {
        
        RCLCPP_INFO(this->get_logger(), "Hello cpp Node");

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this));
    }

private:

    void timerCallback() {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

};


int main(int argc, char **argv) {
    
    // Initialize ROS2 communication.
    rclcpp::init(argc, argv);

    // Create Node
    //auto node = std::make_shared<rclcpp::Node>("cpp_test");
    auto node = std::make_shared<MyNode>();

    // Write a message from this node
    //RCLCPP_INFO(node->get_logger(), "Hello cpp Node");

    // Spin the node so program stays running
    rclcpp::spin(node);

    // Shutdown program
    rclcpp::shutdown();
    
    return 0;
}