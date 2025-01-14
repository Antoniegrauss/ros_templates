#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "demo_interfaces/action/count_until.hpp"

using CountUntil = demo_interfaces::action::CountUntil;
using CountUntilFeedback = demo_interfaces::action::CountUntil_Feedback;
using CountUntilResult = demo_interfaces::action::CountUntil_Result;
const std::string count_until_action_name {"count_until"};