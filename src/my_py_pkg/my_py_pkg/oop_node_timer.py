#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


# The class should be named based on its funciton
class MyNode(Node):

    def __init__(self):
        super().__init__('oop_py_test') # This is the name of the node

        # Do something is now here
        self.get_logger().info('Hello from OOP ROS2')

        # This will keep track of how many times the callback is called
        self.counter_ = 0
        
        # This will create a function callback that runs with the first parameter 
        # determining the number of seconds the provided callback function will be called
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info(f'Hello {self.counter_}')


def main(args=None):
    # First line of python node program
    rclpy.init(args=args)

    # Create a node object, with the node name as an argument
    # node = Node('py_test')

    # For OOP we can now create an object of the class we just defined
    node = MyNode()

    # Do something here!
    # node.get_logger().info('Hello ROS2')

    # Pause program here so that it doesn't shutdown
    rclpy.spin(node)

    # Last line of python node program
    rclpy.shutdown()

if __name__ == '__main__':
    main()