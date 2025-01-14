#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "example_interfaces/msg/int64.hpp"

class SimplePublisher : public rclcpp::Node
{
public:
    using MessageNumber = example_interfaces::msg::Int64;
    SimplePublisher() : Node("number_publisher")
    {
        publisher_ = create_publisher<MessageNumber>("number", 10);
        RCLCPP_INFO(get_logger(), "cpp publisher started");
        timer_ = create_wall_timer(std::chrono::seconds(1),
                                   [this]()
                                   { publishMessage(); 
                                   counter_++; });
    }

private:
    void publishMessage()
    {
        auto message = MessageNumber();
        message.data = counter_ * 2;
        publisher_->publish(message);
    }

private:
    rclcpp::Publisher<MessageNumber>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
