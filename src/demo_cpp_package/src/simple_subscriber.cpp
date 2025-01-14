#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "example_interfaces/msg/string.hpp"
#include "demo_interfaces/msg/hardware_status.hpp"

class SimpleSubscriber : public rclcpp::Node
{
public:
    using MessageString = example_interfaces::msg::String;
    SimpleSubscriber() : Node("simple_subscriber")
    {
        subscriber_ = create_subscription<MessageString>("simple_topic", 10, [this](const MessageString::SharedPtr message)
                                                         {callback(message); });
        RCLCPP_INFO(get_logger(), "cpp subscriber started");
    }

private:
    void callback(const MessageString::SharedPtr message_ptr)
    {
        RCLCPP_INFO(get_logger(), message_ptr->data.c_str());
        auto message = demo_interfaces::msg::HardwareStatus();
        message.temperature = 30;
        message.are_motors_ready = true;
        message.debug_message = "Motors are fine";
    }

private:
    rclcpp::Subscription<MessageString>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
