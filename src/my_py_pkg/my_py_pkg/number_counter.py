#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.num_ = 0
        self.subscriber_ = self.create_subscription(
            Int64, 'number', self.callback_number_counter, 10)
        self.get_logger().info('Number counter has been started')

    # Good naming standard to follow for subscribers
    #   callback_<topic name>()
    def callback_number_counter(self, msg):
        self.num_ += msg.data
        self.get_logger().info(str(self.num_))


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
