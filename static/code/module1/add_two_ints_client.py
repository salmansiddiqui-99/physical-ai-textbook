#!/usr/bin/env python3
"""
ROS 2 Service Client: Add Two Integers

Purpose: Call the /add_two_ints service with user-provided values
Inputs: Two integer command-line arguments
Outputs: Service call result

Usage:
    # Terminal 1: Start service server
    python3 add_two_ints_service.py

    # Terminal 2: Call service from client
    source /opt/ros/humble/setup.bash
    chmod +x add_two_ints_client.py
    python3 add_two_ints_client.py 5 8

    # Alternative: Call service from command line
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 15}"
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AdditionClient(Node):
    """Service client that calls /add_two_ints service."""

    def __init__(self):
        super().__init__('addition_client')

        # Create client: service name, service type
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """Send addition request and wait for response."""
        self.req.a = a
        self.req.b = b

        # Call service asynchronously
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: add_two_ints_client.py <int> <int>')
        return

    node = AdditionClient()

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    response = node.send_request(a, b)
    node.get_logger().info(f'Result: {a} + {b} = {response.sum}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
