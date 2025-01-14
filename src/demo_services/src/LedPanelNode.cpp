#include "rclcpp/rclcpp.hpp"
#include "demo_interfaces/srv/set_led.hpp"
#include "demo_interfaces/msg/led_state.hpp"

const int amount_of_leds {3};
struct LedStates{
    std::vector<bool> led_states {false, false, false};

    bool set_led(int choose_led, bool state) {
        if (choose_led >= amount_of_leds) {
            return false;
        }
        if (led_states[choose_led] == state) {
            return false;
        }
        led_states[choose_led] = state;
        return true;
    }
};

class LedPanelNode : public rclcpp::Node
{
public:
using LedService = demo_interfaces::srv::SetLed;
using LedStateMessage = demo_interfaces::msg::LedState;
    
    LedPanelNode() : Node("led_panel_node")
    {
        const std::vector<bool> led_states_default_value({true, false, true});
        declare_parameter("led_states", led_states_default_value);
        led_states_.led_states = get_parameter("led_states").as_bool_array();

        server_ = create_service<LedService>("set_led", [this](
            const LedService::Request::SharedPtr request,
            const LedService::Response::SharedPtr response
        ) {
            response->success = led_states_.set_led(request->led_number - 1, request->state);
            std::string debug = "Switcing led: " + std::to_string(request->led_number - 1) + ", " + std::to_string(request->state);
            RCLCPP_INFO(get_logger(), debug.c_str());
        });

        publisher_ = create_publisher<LedStateMessage>("led_panel_state", 10);
        timer_ = create_wall_timer(std::chrono::seconds(1), [this] () {
            LedStateMessage message;
            message.led_1_state = led_states_.led_states[0];
            message.led_2_state = led_states_.led_states[1];
            message.led_3_state = led_states_.led_states[2];
            publisher_->publish(message);
        });
    }

private:
    rclcpp::Publisher<LedStateMessage>::SharedPtr publisher_;
    rclcpp::Service<LedService>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
    LedStates led_states_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}