#!/usr/bin/env python3
from functools import partial
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.srv import AddTwoInts

class RecursiveServerNode(Node):
    def __init__(self):
        super().__init__("outer_server")
        self.server_ = self.create_service(AddTwoInts, 
            "outer_server", 
            self.serviceCallback, 
            callback_group=ReentrantCallbackGroup()
        )
        
        self.client = self.create_client(AddTwoInts, "inner_server")
        while not self.client.wait_for_service(1.0):
            self.get_logger().info("Waiting for inner service to connect")
        
        self.response_ready = threading.Event()  # Event to signal when response is ready

    def serviceCallback(self, request, response):
        self.get_logger().info("Calling outer service (server side)")
        return self.callAddTwoIntsService(request, response)

    def callAddTwoIntsService(self, request, response):       
        future = self.client.call_async(request=request)
        future.add_done_callback(partial(self.addTwoIntsCallback, response=response))
           
        # Block until the response is ready
        self.response_ready.wait()           
                
        self.get_logger().info("Returning result from outer service")
        self.get_logger().info("Sum = " + str(response.sum))
        
        return response

    def addTwoIntsCallback(self, future, response):
        try:
            response = future.result()
            self.get_logger().info("Inner service result: " + str(response.sum))
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")
            
        self.response_ready.set()

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = RecursiveServerNode()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()