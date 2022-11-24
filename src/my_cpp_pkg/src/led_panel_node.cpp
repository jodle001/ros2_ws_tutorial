#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel"), leds_({0, 0, 0})
    {
        led_pub_ = this->create_publisher<my_robot_interfaces::msg::LedStates>(
            "led_panel_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LedPanelNode::publishLedPanelState, this));
                    
        RCLCPP_INFO(this->get_logger(), "Led Panel has been started");
    }

private:

    void publishLedPanelState()
    {
        auto msg = my_robot_interfaces::msg::LedStates();
        msg.led1 = leds_[0];
        msg.led2 = leds_[1];
        msg.led3 = leds_[2];
        led_pub_->publish(msg);
    }

    std::array<int, 3> leds_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr led_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
