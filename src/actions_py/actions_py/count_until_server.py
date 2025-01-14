#!/usr/bin/env python3
import time
import enum
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import (
    ActionServer,
    GoalResponse,
    CancelResponse
)
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from demo_interfaces.action import CountUntil

# Policy on how to handle multiple action requests
class GoalPolicy(enum.Enum):
    Parralel = 1
    SingleGoal = 2
    PreEmptCurrent = 3
    Queue = 4

class CountUntilServerNode(Node):
    def __init__(self, goal_policy: GoalPolicy):
        super().__init__("count_until_server_node")

        self.goal_policy_ = goal_policy
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()

        if self.goal_policy_ == GoalPolicy.Queue:
            self.goal_queue_ = []

        self.count_until_server_ = ActionServer(
            self, 
            CountUntil, 
            "count_until",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received a goal")

        # Policy: reject new goal if current goal is active
        if self.goal_policy_ == GoalPolicy.SingleGoal:
            with self.goal_lock_:   
                if self.goal_handle_ is not None and \
                    self.goal_handle_.is_active:

                    self.get_logger().info("A goal is already acrive, rejecting new goal")
                    return GoalResponse.REJECT

        # Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Reject the goal: number is: " + \
                str(goal_request.target_number))
            return GoalResponse.REJECT
        
        # Policy pre-empt current goal if new goal requested
        if self.goal_policy_ == GoalPolicy.PreEmptCurrent:
            with self.goal_lock_:
                if self.goal_handle_ is not None and \
                    self.goal_handle_.is_active:

                    self.get_logger().info("Abort current goal, accept new goal")
                    self.goal_handle_.abort()


        self.get_logger().info("Accepting the goal: number is: " + \
            str(goal_request.target_number))
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        if self.goal_policy_ != GoalPolicy.Queue:
            goal_handle.execute()
            return

        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cencel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        # Get request from the goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        
        counter = 0
        for _ in range (target_number):
            if not goal_handle.is_active:
                return self.trigger_action_end(counter=counter)
            
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel: stopping action execution")
                goal_handle.canceled()
                return self.trigger_action_end(counter=counter)
            
            if should_abort(counter=counter):
                goal_handle.abort()
                return self.trigger_action_end(counter=counter)
            
            counter += 1
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback=feedback)
            time.sleep(period)

        # Set goal final state
        if counter == target_number:
            goal_handle.succeed()

        # Send the result
        return self.trigger_action_end(counter=counter)
    
    def trigger_action_end(self, counter) -> CountUntil.Result:
        result = CountUntil.Result()
        result.reached_number = counter
        self.process_next_goal_in_queue()
        return result

    def process_next_goal_in_queue(self):
        if not self.goal_policy_ == GoalPolicy.Queue:
            return

        with self.goal_lock_:
            if self.goal_queue_:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None

def should_abort(counter):
    # return counter > 0 and counter %3 == 0
    return False

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode(GoalPolicy.Queue)
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
