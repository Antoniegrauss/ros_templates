#include <queue>
#include <memory>
#include <thread>

#include "actions_cpp/count_until_action_includes.hpp"
#include "actions_cpp/goal_policy.hpp"
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;

class NumberLoop {
public:
    NumberLoop(int target) : 
        target_(target) {
            std::cout<< target_;
        }

    void tick() {
        if (counter_ < target_) {
            counter_++;
            std::cout << counter_;
        }
    }

    bool is_finished() const {
        return counter_ == target_;
    }

    int get_counter() const {
        return counter_;
    }

private:
    int counter_ {0};
    int target_ {0};
};

bool goal_valid(int target_number) {
    return target_number >= 0;
}

class CountUntilServerNode : public rclcpp::Node
{
public:
    CountUntilServerNode(GoalPolicy policy) : Node("count_until_server_node")
    {
        policy_ = policy;
        if (policy_ == GoalPolicy::Queue) {
            goals_worker_ = std::thread(create_execute_callback_queue());
        }

        callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        server_ = rclcpp_action::create_server<CountUntil>(
            this,
            count_until_action_name,
            create_goal_callback(),
            create_cancel_callback(),
            create_handle_accepted_callback(),
            rcl_action_server_get_default_options(),
            callback_group_
        );
        RCLCPP_INFO(get_logger(), "Action server has started");
    }

    ~CountUntilServerNode() {
        goals_worker_.join();
    }

private:
    rclcpp_action::Server<CountUntil>::SharedPtr server_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    GoalPolicy policy_;
    
    std::queue<std::shared_ptr<CountUntilGoalHandle>> goals_;
    std::mutex goals_lock_;
    std::thread goals_worker_;

    std::function<void()> create_execute_callback_queue() 
    {
        return [this]() {
            rclcpp::Rate loop_rate(10);
            while(rclcpp::ok()) {
                std::shared_ptr<CountUntilGoalHandle> next_goal;
                {
                    std::lock_guard<std::mutex> guard(goals_lock_);
                    if (!goals_.empty())
                    {
                        next_goal = goals_.front();
                        goals_.pop();
                    }                    
                }

                if (next_goal) {
                    RCLCPP_INFO(get_logger(), "Executing next goal in queue");
                    execute_goal(next_goal);
                }

                loop_rate.sleep();
            }
        };
    } 

    std::function<rclcpp_action::GoalResponse(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const CountUntil::Goal>)> create_goal_callback() 
    {
        return [this](const rclcpp_action::GoalUUID &/*uuid*/,
                            std::shared_ptr<const CountUntil::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Received a goal");

            switch (policy_) 
            {
                case GoalPolicy::Parallel:
                    break;
                case GoalPolicy::RejectSecond:
                {
                    if (goal_running()) {
                        RCLCPP_INFO(get_logger(), "Rejected a goal, current goal still running");
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                    break;
                }
                case GoalPolicy::OverrideFirst:
                {
                    if(goal_running() && 
                        goal_valid(goal->target_number)) 
                    {
                        std::lock_guard<std::mutex> guard(goals_lock_);
                        // Note: could also abort in the execute loop by remembering the UUID
                        goals_.front()->abort(generate_result(-1));
                        goals_.pop();
                        RCLCPP_INFO(get_logger(), "Overriding current goal");
                    }
                    break;
                }
                case GoalPolicy::Queue:
                    RCLCPP_INFO(get_logger(), "Adding goal to queue");
                    break;
            }

            if (!goal_valid(goal->target_number)) 
            {
                RCLCPP_INFO(get_logger(), "Rejected a goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            RCLCPP_INFO(get_logger(), "Accepted a goal");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };
    }

    std::function<rclcpp_action::CancelResponse(
        const std::shared_ptr<CountUntilGoalHandle>)> create_cancel_callback()
    {
        return [this](
            const std::shared_ptr<CountUntilGoalHandle> /*goal_handle*/)
        {
            RCLCPP_INFO(get_logger(), "Received a cancel request");
            RCLCPP_INFO(get_logger(), "Canceling the goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        };
    }

    std::function<void(
        const std::shared_ptr<CountUntilGoalHandle>)> 
        create_handle_accepted_callback()
    {
        return [this](
            const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {
            add_goal_to_queue(goal_handle);
            if (policy_ != GoalPolicy::Queue) {
                execute_goal(goal_handle);
            }
        };
    }

    void execute_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        NumberLoop number_loop(goal_handle->get_goal()->target_number);

        float sleep_time_seconds = goal_handle->get_goal()->period;
        rclcpp::Rate loop_rate(1.0 / sleep_time_seconds);

        auto feedback = std::make_shared<CountUntil::Feedback>();
        while(! number_loop.is_finished()) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(
                    generate_result(number_loop.get_counter()));
                    RCLCPP_INFO(get_logger(), "Goal was canceled");
                return;
            }

            if (!goal_handle->is_active()) {
                generate_result(number_loop.get_counter());
                    RCLCPP_INFO(get_logger(), "Goal not active");
                return;
            }

            number_loop.tick();
            feedback->current_number = number_loop.get_counter();
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }

        goal_handle->succeed(
            generate_result(number_loop.get_counter()));
    }

    std::shared_ptr<CountUntilResult> generate_result(int counter)
    {
        auto result = std::make_shared<CountUntil::Result>();
        result->reached_number = counter;
        return result;
    }

    bool goal_running() {
        std::lock_guard<std::mutex> guard(goals_lock_);
        return ! goals_.empty() &&
        goals_.front()->is_active();
    }

    void add_goal_to_queue(const std::shared_ptr<CountUntilGoalHandle> goal) {
        std::lock_guard<std::mutex> guard(goals_lock_);
        goals_.push(goal);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>(GoalPolicy::Queue);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
