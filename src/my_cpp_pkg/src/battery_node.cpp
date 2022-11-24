#include "rclcpp/rclcpp.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery"), state_(100)
    {
        RCLCPP_INFO(this->get_logger(), "Battery Node has been started.");
    }

private:
    int state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
