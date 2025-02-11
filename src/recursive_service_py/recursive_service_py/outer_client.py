#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts

class RecursiveClientNode(Node):
    def __init__(self):
        super().__init__("outer_client")
        self.callAddTwoIntsService(5, 3)

    def callAddTwoIntsService(self, a, b):
        client = self.create_client(AddTwoInts, "outer_server")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for outer server to connect")
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        self.get_logger().info("Sending outer service request (client side)")

        future = client.call_async(request=request)
        future.add_done_callback(partial(self.addTwoIntsCallback, a=a, b=b))

    def addTwoIntsCallback(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info("Response outer service (client side)")
            self.get_logger().info(str(a) + " + " + str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RecursiveClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()