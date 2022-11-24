#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node('add_two_ints_no_oop')

    client = node.create_client(AddTwoInts, 'add_two_ints')

    # Perform a wait for service, with a 1 second timeout
    # This will loop every second until service is found, each loop 
    # without service will print the warning message.
    while not client.wait_for_service(1.0):
        node.get_logger().warn('Waiting for Server Add Two Ints')

    request = AddTwoInts.Request()
    request.a = 3
    request.b = 8

    # Future objects are objects where their value will be set later
    future = client.call_async(request)

    # This will take care of spinning until the Future is complete
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
    except Exception as e:
        node.get_logger().error(f'Service call failed {e}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
