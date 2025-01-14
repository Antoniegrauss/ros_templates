#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from demo_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("simple")
        self.declare_parameter("temperature", 50)
        self.temperature = self.get_parameter("temperature").get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_info", 10)
        self.timer_ = self.create_timer(1.0, self.publishHardwareStatus)
        self.get_logger().info("Hardware publisher has started")

    def publishHardwareStatus(self):
        message = HardwareStatus()
        message.are_motors_ready = True
        message.debug_message = "message to debug"
        message.temperature = self.temperature
        self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()