#include "actions_cpp/count_until_action_includes.hpp"

using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using ActionSendGoalOptions = rclcpp_action::Client<CountUntil>::SendGoalOptions;

std::string to_string(rclcpp_action::ResultCode code) {
    switch(code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return "SUCCEEDED";
        case rclcpp_action::ResultCode::ABORTED:
            return "ABORTED";
        case rclcpp_action::ResultCode::CANCELED:
            return "CANCELED";
        case rclcpp_action::ResultCode::UNKNOWN:
            return "UNKNOWN";
        default:
            return "";
    }
}

class CountUntilClient : public rclcpp::Node
{
public:
    CountUntilClient() : Node("count_until_client")
    {
        client_ = rclcpp_action::create_client<CountUntil>(
            this, count_until_action_name
        );
    }

    void send_goal(int target_number, float period) {
        client_->wait_for_action_server();

        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.period = period;      

        RCLCPP_INFO(get_logger(), "Sending a goal");
        client_->async_send_goal(goal, create_callbacks());

        // timer_ = create_wall_timer(std::chrono::seconds(2),
        //     [this]() {
        //         timer_->cancel();
        //         client_->async_cancel_goal(goal_handle_);
        //     }
        // );
    }

private:
    rclcpp_action::Client<CountUntil>::SharedPtr client_;
    std::shared_ptr<CountUntilGoalHandle> goal_handle_;
    rclcpp::TimerBase::SharedPtr timer_;

    using GoalCallbackType = std::function<void 
        (const CountUntilGoalHandle::WrappedResult&)>;
    using ResponseCallbackType = std::function<void 
        (std::shared_ptr<CountUntilGoalHandle>)>;
    using FeedbackCallbackType = std::function<void 
        (std::shared_ptr<CountUntilGoalHandle>, 
        std::shared_ptr<const CountUntilFeedback>)>;

    ActionSendGoalOptions create_callbacks() {
        ActionSendGoalOptions options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        
        options.result_callback = create_goal_result_callback();
        options.goal_response_callback = create_response_callback();
        options.feedback_callback = create_feedback_callback();

        return options;
    }

    GoalCallbackType create_goal_result_callback() {
        return [this](
            const CountUntilGoalHandle::WrappedResult& result) 
        {
            std::string message = "Goal state: " + to_string(result.code);
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), message.c_str());
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(get_logger(), message.c_str());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), message.c_str());
                    break;
                case rclcpp_action::ResultCode::UNKNOWN:
                    RCLCPP_ERROR(get_logger(), message.c_str());
                    break;
            }
            
            int reached_number = result.result->reached_number;
            RCLCPP_INFO(get_logger(), "Result: %d", reached_number);
        };
    }

    ResponseCallbackType create_response_callback() {
        return [this](
            std::shared_ptr<CountUntilGoalHandle> goal_handle) 
        {
            if (! goal_handle) {
                RCLCPP_INFO(get_logger(), "Goal got rejected");
            }
            else {
                RCLCPP_INFO(get_logger(), "Goal got accepted");
                goal_handle_ = goal_handle;
            }
        };
    }

    FeedbackCallbackType create_feedback_callback() {
        return [this](
            std::shared_ptr<CountUntilGoalHandle> /*goal_handle*/, 
            std::shared_ptr<const CountUntilFeedback> feedback) 
        {
            int number = feedback->current_number;
            RCLCPP_INFO(get_logger(), "Feedback: %d", number);
            
            if (number == 4) {
                // Could cancel the goal here for example

                // RCLCPP_INFO(get_logger(), "Cancel the goal");
                // client_->async_cancel_goal(goal_handle);
            }
        };
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClient>();
    node->send_goal(6, 0.8);
    node->send_goal(7, 0.8);
    node->send_goal(8, 0.8);
    // node->send_goal(-6, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
