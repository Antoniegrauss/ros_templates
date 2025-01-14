#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
    using AddInts = example_interfaces::srv::AddTwoInts;

public:
    AddTwoIntsClientNode() : Node("add_two_ints_cclient")
    {
        RCLCPP_INFO(get_logger(), "Service client has started");
        thread_ = std::thread([this](){callAddTwoIntsService(5, 4);});
    }

    void callAddTwoIntsService(int a, int b) 
    {
        auto client = create_client<AddInts>("add_two_ints");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(get_logger(), "Waiting for communication with server");
        }

        auto request = std::make_shared<AddInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);

        try 
        {
            auto response = future.get();
            RCLCPP_INFO(get_logger(), "%d + %d = %d", a, b, response->sum);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Service call failed");
        }
    }
private:
    std::thread thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}