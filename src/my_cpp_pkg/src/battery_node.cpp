#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery"), state_(100)
    {
        batteryTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&BatteryNode::checkBatteryState, this));
        lastTimeBatteryChanged_ = this->getCurrentTimeSeconds();

        RCLCPP_INFO(this->get_logger(), "Battery Node has been started.");
    }

    void checkBatteryState()
    {
        auto timeNow = this->getCurrentTimeSeconds();
        if (state_ == 100)
        {
            if (timeNow - this->lastTimeBatteryChanged_ > 4.0)
            {
                state_ = 0;
                RCLCPP_INFO(this->get_logger(), "Battery is empty! Charging battery...");
                lastTimeBatteryChanged_ = timeNow;

                thread_ = new std::thread(std::bind(&BatteryNode::callSetLedService, this, 100));
            }
        }
        else
        {
            if (timeNow - lastTimeBatteryChanged_ > 6.0)
            {
                state_ = 100;
                RCLCPP_INFO(this->get_logger(), "Battery is now full again.");
                lastTimeBatteryChanged_ = timeNow;
                thread_ = new std::thread(std::bind(&BatteryNode::callSetLedService, this, 0));
            }
        }
    }

    double getCurrentTimeSeconds()
    {
        return this->get_clock()->now().seconds();
    }

    void callSetLedService(int battery_level)
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting to connect to Set Led Service");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->number = 3;
        if (battery_level)
            request->state = true;
        else
            request->state = false;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            
            std::string success;
            if (response)
                success = "Success";
            else
                success = "Failed";

            RCLCPP_INFO(this->get_logger(), "The led response is %s", success.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
        // Maybe try this?
        // std::terminate();
    }

private:
    int state_;
    double lastTimeBatteryChanged_;
    rclcpp::TimerBase::SharedPtr batteryTimer_;
    std::thread *thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
