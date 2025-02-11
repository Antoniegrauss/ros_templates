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
        super().__init__("recursive_server")
        callback_group = ReentrantCallbackGroup()
        self.server_ = self.create_service(AddTwoInts, 
            "recursive_service", 
            self.serviceCallback, 
            callback_group=callback_group
        )
        
        self.response_ready = threading.Event()  # Event to signal when response is ready
        self.response = None

    def serviceCallback(self, request, response):
        self.get_logger().info("Calling outer service")
        return self.callAddTwoIntsService(request, response)

    def callAddTwoIntsService(self, request, response):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for AddTwoIntsServer to connect")
        
        future = client.call_async(request=request)
        future.add_done_callback(partial(self.addTwoIntsCallback)
        )
           
        # Block until the response is ready
        self.response_ready.wait()           
                
        self.get_logger().info("Returning result from outer service")
        self.get_logger().info("Sum = " + str(self.response.sum))
        
        response.sum = self.response.sum
        return response

    def addTwoIntsCallback(self, future):
        try:
            self.response = future.result()
            self.get_logger().info("Inner service result: " + str(self.response.sum))
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