#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {
        this->declare_parameter("led_states", std::vector<int64_t>({0, 0, 0}));
        leds_ = this->get_parameter("led_states").as_integer_array();

        led_pub_ = this->create_publisher<my_robot_interfaces::msg::LedStates>(
            "led_panel_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LedPanelNode::publishLedPanelState, this));
        
        set_led_service_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led", std::bind(&LedPanelNode::setLedPanel, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Led Panel has been started");
    }

private:

    void publishLedPanelState()
    {
        auto msg = my_robot_interfaces::msg::LedStates();
        msg.led_states = leds_;
        led_pub_->publish(msg);
    };

    void setLedPanel(
        const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
        {
            if (request->number > (long int)leds_.size() || request->number <= 0) {
                response->success = false;
            } else {
                if (request->state)
                    leds_[request->number-1] = 1;
                else
                    leds_[request->number-1] = 0;
                    
                response->success = true;
            }
        }

    std::vector<int64_t> leds_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr led_pub_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr set_led_service_;
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
