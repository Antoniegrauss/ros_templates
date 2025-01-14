#include "actions_cpp/move_robot_action_includes.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;


class MoveRobotActionImpl {
public:
    MoveRobotActionImpl(int target_position, int velocity) :
        target_position_(target_position),
        velocity_(velocity) {}

    // Moves the robot and returns the new position
    void tick() {
        current_position_ += velocity_;
        if (current_position_ >= target_position_) {
            current_position_ = target_position_;
        }
    }

    bool is_done() {
        return current_position_ == target_position_;
    }

    int current_position() const {
        return current_position_;
    }

private:
    int current_position_ {0};
    int target_position_ {0};
    int velocity_ {0};
};

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::
    LifecycleNodeInterface::CallbackReturn;
class MoveRobotServerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    MoveRobotServerNode() : LifecycleNode("move_robot_server_node"), 
        action_implementer_(0, 0)
    {
        callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override
    {
        declare_parameter("action_name", move_robot_action_name);
        action_name_ = get_parameter("action_name").as_string();

        auto goal_callback =
        [this](const rclcpp_action::GoalUUID& /*uuid*/, 
            std::shared_ptr<
            const demo_interfaces::action::MoveRobot_Goal> /*goal*/) 
        {
            {
                std::lock_guard<std::mutex> guard(goal_mutex_);

                if (! server_activated_)
                {
                    RCLCPP_INFO(get_logger(), "Server not activated");
                    return rclcpp_action::GoalResponse::REJECT;
                }

                if (goal_ && goal_->is_active()) {
                    RCLCPP_INFO(get_logger(), "Overriding current goal");
                    uuid_to_abort_ = goal_->get_goal_id();
                }
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        auto cancel_callback = 
        [this](
            const std::shared_ptr<MoveRobotGoalHandle> /*goal_handle*/)
        {
            RCLCPP_INFO(get_logger(), "Received a cancel request");
            RCLCPP_INFO(get_logger(), "Canceling the goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto handle_accepted_callback = 
        [this](
            const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
        {
            goal_ = goal_handle;
            execute_goal(goal_handle);
        };

        server_ = rclcpp_action::create_server<MoveRobot>(
            this,
            action_name_,
            goal_callback,
            cancel_callback,
            handle_accepted_callback,
            rcl_action_server_get_default_options(),
            callback_group_
        );
        rclcpp_lifecycle::LifecycleNode::on_configure(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override
    {
        server_.reset();
        undeclare_parameter("action_name");
        action_name_ = move_robot_action_name;
        rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override
    {
        {
            std::lock_guard<std::mutex> guard(goal_mutex_);
            server_activated_ = true;
        }
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override
    {
        {
            std::lock_guard<std::mutex> guard(goal_mutex_);
            server_activated_ = false;
        }
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

private:
    rclcpp_action::Server<MoveRobot>::SharedPtr server_;
    bool server_activated_ {false};
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    int start_position {0};
    MoveRobotActionImpl action_implementer_;
    std::string action_name_ {move_robot_action_name};

    std::shared_ptr<MoveRobotGoalHandle> goal_;
    rclcpp_action::GoalUUID uuid_to_abort_;
    std::mutex goal_mutex_;
    

    void execute_goal(const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Action started execution");
        action_implementer_ = MoveRobotActionImpl(
            goal_handle->get_goal()->position,
            goal_handle->get_goal()->velocity);

        auto feedback = std::make_shared<MoveRobot::Feedback>();
        rclcpp::Rate loop_rate(1.0);
        while (! action_implementer_.is_done())
        {
            rclcpp_action::GoalUUID uuid_to_abort;
            {
                std::lock_guard<std::mutex> guard(goal_mutex_);
                uuid_to_abort = uuid_to_abort_;
            }
            if (! goal_handle->is_active() ||
                goal_handle->get_goal_id() == uuid_to_abort ||
                !server_activated_) 
            {
                RCLCPP_INFO(get_logger(), "Aborting goal");
                goal_handle->abort(
                    generate_result(action_implementer_.current_position()));
                return;
            }
            else if (goal_handle->is_canceling())
            {
                RCLCPP_INFO(get_logger(), "Canceling goal");
                goal_handle->canceled(
                    generate_result(action_implementer_.current_position()));
                return;
            }

            action_implementer_.tick();
            feedback->current_position = action_implementer_.current_position();
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }

        goal_handle->succeed(generate_result(
            action_implementer_.current_position()));
        RCLCPP_INFO(get_logger(), "Finished action");
    }

    std::shared_ptr<MoveRobotResult> generate_result(int position)
    {
        auto result = std::make_shared<MoveRobot::Result>();
        result->position = position;
        result->message = "Gave final position";
        return result;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
