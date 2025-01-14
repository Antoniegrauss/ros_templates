#include "std_msgs/msg/empty.hpp"
#include "actions_cpp/move_robot_action_includes.hpp"

using MoveRobotGoalHandle = rclcpp_action::ClientGoalHandle<MoveRobot>;
using ActionSendGoalOptions = rclcpp_action::Client<MoveRobot>::SendGoalOptions;

class MoveRobotClientNode : public rclcpp::Node
{
public:
    MoveRobotClientNode() : Node("move_robot_client_node")
    {
        client_ = rclcpp_action::create_client<MoveRobot>(
            this, move_robot_action_name
        );
        subscription_ = create_subscription<Empty>(
            move_robot_cancel_topic,
            10,
            [this] (const std_msgs::msg::Empty::SharedPtr) {
                if (!goal_handle_) {
                    return;
                }
                RCLCPP_INFO(get_logger(), "Canceling goal");
                client_->async_cancel_goal(goal_handle_);
                goal_handle_ = nullptr;
            }
        );
    }

    void send_goal(int position, int velocity)
    {
        client_->wait_for_action_server();

        auto goal = MoveRobot::Goal();
        goal.position = position;
        goal.velocity = velocity;

        auto options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        options.feedback_callback = [this](
            std::shared_ptr<MoveRobotGoalHandle> /*goal_handle*/, 
            std::shared_ptr<const MoveRobotFeedback> feedback)
        {
            RCLCPP_INFO(get_logger(), "Feedback: position = %d", 
                static_cast<int>(feedback->current_position));
        };
        options.result_callback = [this](
            const MoveRobotGoalHandle::WrappedResult& result)
        {
            int final_position = result.result->position;
            RCLCPP_INFO(get_logger(), "Final position: %d", final_position);
        };
        options.goal_response_callback = [this](
            std::shared_ptr<MoveRobotGoalHandle> goal_handle)
        {
            if (! goal_handle) {
                RCLCPP_INFO(get_logger(), "Goal got rejected");
            }
            else {
                RCLCPP_INFO(get_logger(), "Goal got accepted");
                goal_handle_ = goal_handle;
            }
        };

        RCLCPP_INFO(get_logger(), "Sending a goal");
        client_->async_send_goal(goal, options);
    }

private:
    rclcpp_action::Client<MoveRobot>::SharedPtr client_;
    
    using Empty = std_msgs::msg::Empty;
    rclcpp::Subscription<Empty>::SharedPtr subscription_;
    
    std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotClientNode>();
    node->send_goal(900, 11);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}