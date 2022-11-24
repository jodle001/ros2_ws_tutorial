#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from functools import partial


class ResetCounterNode(Node):
    def __init__(self):
        super().__init__("reset_counter_client")
        self.call_reset_counter(True)

    def call_reset_counter(self, reset_flag):
        client = self.create_client(SetBool, 'reset_counter')

        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for server number counter')

        request = SetBool.Request()
        request.data = bool(reset_flag)

        future = client.call_async(request)
        future.add_done_callback(partial(self.call_reset_counter))

    def callback_call_reset_counter(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'{response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterNode()
    # rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
