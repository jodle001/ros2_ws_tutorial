#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main(args=None):
    # First line of python node program
    rclpy.init(args=args)

    # Create a node object, with the node name as an argument
    node = Node('py_test')

    # Do something here!
    node.get_logger().info('Hello ROS2')

    # Pause program here so that it doesn't shutdown
    rclpy.spin(node)

    # Last line of python node program
    rclpy.shutdown()

if __name__ == '__main__':
    main()