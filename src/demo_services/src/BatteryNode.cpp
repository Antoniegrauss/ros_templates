#include "rclcpp/rclcpp.hpp"
#include "demo_interfaces/srv/set_led.hpp"

enum class BatteryState {
    Full,
    Empty
};

const std::chrono::seconds time_to_full {6};
const std::chrono::seconds time_to_empty {4};

class BatteryNode : public rclcpp::Node{
public:
using LedService = demo_interfaces::srv::SetLed;
    BatteryNode() : Node("battery_node")
    {
        timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            second_counter_++;
            std::string debug = "Battery Tick seconds: " + 
                std::to_string(second_counter_);
            RCLCPP_INFO(get_logger(), debug.c_str());

            auto request = std::make_shared<LedService::Request>();
            if (state_ == BatteryState::Full) {
                if (second_counter_ >= time_to_empty.count()) {
                    // Battery empty again
                    RCLCPP_INFO(get_logger(), "Battery empty again");
                    state_ = BatteryState::Empty;
                    second_counter_ = 0;

                    request->led_number = 3;
                    request->state = false;
                }
            } else {
                if (second_counter_ >= time_to_full.count()) {
                    // Battery full again
                    RCLCPP_INFO(get_logger(), "Battery full again");
                    state_ = BatteryState::Full;
                    second_counter_ = 0;

                    request->led_number = 3;
                    request->state = true;
                }
            }

            if (second_counter_ != 0) 
            {
                return;
            }

            auto client = create_client<LedService>("set_led");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_INFO(get_logger(), "Waiting for communication with server");
            }

            auto response = client->async_send_request(request);
        });
    }
private:
    BatteryState state_{BatteryState::Full};
    rclcpp::TimerBase::SharedPtr timer_;
    int second_counter_ {0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}