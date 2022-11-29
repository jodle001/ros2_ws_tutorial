#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <math.h>

using std::placeholders::_1;

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller"), turtlesimUp_(false)
    {
        poseSub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleControllerNode::subscribeTurtlePose, this, _1));

        turtleArraySub_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
            "alive_turtles",
            10,
            std::bind(&TurtleControllerNode::subscribeTurtleArray, this, _1));

        cmdVelPub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TurtleControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Turtle Controller has been started.");
    }

private:
    void subscribeTurtlePose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        pose_ = *msg.get();
        turtlesimUp_ = true;
    }

    void subscribeTurtleArray(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        if (!msg->turtles.empty())
            turtleToCatch_ = msg->turtles.at(0);
    }

    void publishCmdVel(double x, double theta)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = x;
        msg.angular.z = theta;
        cmdVelPub_->publish(msg);
    }

    void controlLoop()
    {
        if (!turtlesimUp_ || turtleToCatch_.name == "")
        {
            return;
        }

        double dist_x = turtleToCatch_.x - pose_.x;
        double dist_y = turtleToCatch_.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.5)
        {
            // position
            msg.linear.x = 2 * distance;

            // orientation
            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            // target reached!
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

        cmdVelPub_->publish(msg);
    }


    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr poseSub_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr turtleArraySub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_;
    turtlesim::msg::Pose pose_;
    my_robot_interfaces::msg::TurtleArray::SharedPtr aliveTurtles_;
    my_robot_interfaces::msg::Turtle turtleToCatch_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool turtlesimUp_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
