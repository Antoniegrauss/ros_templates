#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "demo_interfaces/action/move_robot.hpp"

using MoveRobot = demo_interfaces::action::MoveRobot;
using MoveRobotFeedback = demo_interfaces::action::MoveRobot_Feedback;
using MoveRobotResult = demo_interfaces::action::MoveRobot_Result;
const std::string move_robot_action_name {"move_robot"};
const std::string move_robot_cancel_topic {"cancel_move_robot"};