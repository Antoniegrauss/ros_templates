#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from std_srvs.srv import Empty

class NumberCounter(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0

        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.subscriber_ = self.create_subscription(Int64, "number", self.publishCountCallback, 1)
        self.server_ = self.create_service(Empty, "reset_counter", self.resetCounter)
        self.get_logger().info("Number counter has been started")

    def resetCounter(self, request, response):
        self.counter_ = 0
        return response

    def publish_message(self):
        msg = Int64()
        msg.data = self.counter_
        self.publisher_.publish(msg)

    def publishCountCallback(self, number):
        self.counter_ += 1
        self.publish_message()

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()