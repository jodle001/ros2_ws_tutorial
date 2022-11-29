#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <random>
#include <math.h>

using std::placeholders::_1;
using std::placeholders::_2;

float randomNumber(float lo, float hi)
{
    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::uniform_real_distribution<float> dist(lo, hi);
    return dist(engine);
}


class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner")
    {
        alive_turtles_pub_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>(
            "alive_turtles", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TurtleSpawnerNode::publishAliveTurtles, this));

        catchTurtleService_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>("catch_turtle", 
            std::bind(&TurtleSpawnerNode::callbackCatchTurtleService, this, _1, _2));
        
        threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::callSpawnService, this)));
        RCLCPP_INFO(this->get_logger(), "Turtle Spawner has been started.");
    }

private:

    void callbackCatchTurtleService(
        const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
        const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response
    )
    {
        // Search list and remove matching turtle
    }

    void publishAliveTurtles()
    {
        auto msg = my_robot_interfaces::msg::TurtleArray();
        msg.turtles = turtles_;
        alive_turtles_pub_->publish(msg);
    }    

    void callSpawnService()
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!client->wait_for_service(std::chrono::seconds(2)))
            RCLCPP_WARN(this->get_logger(), "Waiting for turtlesim /spawn service...");

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = randomNumber(0.0, 11.0);
        request->y = randomNumber(0.0, 11.0);
        request->theta = randomNumber(-1.0, 1.0) * M_PI;

        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", request->x, request->y, request->theta);

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            auto turtle  = my_robot_interfaces::msg::Turtle();
            turtle.name = response->name;
            turtle.x = request->x;
            turtle.y = request->y;
            turtle.theta = request->theta;
            turtles_.push_back(turtle);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    std::vector<std::thread> threads_;
    std::vector<my_robot_interfaces::msg::Turtle> turtles_;
    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_pub_;
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catchTurtleService_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
