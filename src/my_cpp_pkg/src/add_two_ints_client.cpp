#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        // This will not work, because the future.get() function
        // will block the thread from spinning the node, which is
        // required in order to get the response.
        // callAddTwoIntsService(1, 4);

        // The answer is to use another thread
        threads_.push_back(std::thread(std::bind(
                &AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4)));

        // Using this method you can run the client multiple times
        threads_.push_back(std::thread(std::bind(
                &AddTwoIntsClientNode::callAddTwoIntsService, this, 6, 42)));
    }

    void callAddTwoIntsService(int a, int b)
    {
        // Create a client with the correct datatype and service name
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        // Wait to connect to service
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // Once service connection is esablished, create a request object.
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        // Fill request with desired parameter data
        request->a = a;
        request->b = b;

        // Send the request asyncronously using the client
        auto future = client->async_send_request(request);

        // the get() function will wait and block until we recieve a response
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, (int)response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
