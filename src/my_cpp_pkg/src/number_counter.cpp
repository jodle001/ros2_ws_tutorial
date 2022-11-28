#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), num_(0)
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
        

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);


        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberCounterNode::callbackResetNumber, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
    }

private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        num_ += msg->data;
        // RCLCPP_INFO(this->get_logger(), "%d", num_);
        auto msgOut = example_interfaces::msg::Int64();
        msgOut.data = num_;
        publisher_->publish(msgOut);
    }

    void callbackResetNumber(
        const example_interfaces::srv::SetBool::Request::SharedPtr request,
        const example_interfaces::srv::SetBool::Response::SharedPtr response
    )
    {   

        try
        {
            if (request->data)
            {
                num_ = 0;
                RCLCPP_INFO(this->get_logger(), "Counter has been set to zero");
            }        
            else
            {
                RCLCPP_INFO(this->get_logger(), "Counter was not reset");
            }
            response->success = true;
            response->message = "reset counter ran successfully";
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
            response->success = false;
            response->message = "Service call failed";
        }
    }

    int64_t num_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
