#!/usr/bin/env python3
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.srv import AddTwoInts

class RecursiveServerNode(Node):
    def __init__(self, executor):
        super().__init__("outer_server")
        self.executor = executor
        self.server_ = self.create_service(AddTwoInts, 
            "outer_server", 
            self.serviceCallback, 
            callback_group=ReentrantCallbackGroup()
        )
        
        self.client = self.create_client(AddTwoInts, "inner_server")
        while not self.client.wait_for_service(1.0):
            self.get_logger().info("Waiting for inner service to connect")
        
    def serviceCallback(self, request, response):
        self.get_logger().info("Calling outer service (server side)")
        return self.callAddTwoIntsService(request, response)

    def callAddTwoIntsService(self, request, response):       
        future = self.client.call_async(request=request)
        self.executor.spin_until_future_complete(future)
        
        response = future.result()
        return response
            
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = RecursiveServerNode(executor)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()