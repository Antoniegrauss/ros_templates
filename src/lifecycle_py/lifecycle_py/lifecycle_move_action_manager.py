#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.declare_parameter("managed_node_names", rclpy.Parameter.Type.STRING_ARRAY)
        node_names = self.get_parameter("managed_node_names").value
        self.get_logger().info("Nodes: " + str(node_names))

        self.clients_list = []
        for node_name in node_names:
            self.clients_list.append(self.create_client(ChangeState, "/" + node_name + "/change_state"))
        
    def change_state(self, transition: Transition):
        for client in self.clients_list:
            client.wait_for_service()
            request = ChangeState.Request()
            request.transition = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
    
    def initialization_sequence(self):
        # Unconfigured to Inactive
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)
        self.get_logger().info("Configuring OK, now inactive")

        # sleep just for the example
        time.sleep(3)

        # Inactive to Active
        self.get_logger().info("Trying to switch to activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition)
        self.get_logger().info("Activating OK, now active")


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleNodeManager()
    node.initialization_sequence()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
