#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")

        self.subscriber_ = self.create_subscription(String, "simple_topic", self.simple_callback, 10)
        self.get_logger().info("Simple subscriber has been started")

    def simple_callback(self, message):
        self.get_logger().info(message.data)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()