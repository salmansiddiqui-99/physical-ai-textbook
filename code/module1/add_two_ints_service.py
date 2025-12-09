#!/usr/bin/env python3
"""
ROS 2 Service Server: Add Two Integers

Purpose: Provide an add_two_ints service that sums two integers
Inputs: AddTwoInts.srv requests (two int64 values)
Outputs: AddTwoInts.srv responses (sum as int64)

Usage:
    source /opt/ros/humble/setup.bash
    chmod +x add_two_ints_service.py
    python3 add_two_ints_service.py
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AdditionService(Node):
    """
    Service server that adds two integers.

    Service: /add_two_ints (example_interfaces/AddTwoInts)
    """

    def __init__(self):
        super().__init__('addition_service')

        # Create service: service name, service type, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Addition service ready at /add_two_ints')

    def add_two_ints_callback(self, request, response):
        """
        Service callback that processes addition requests.

        Args:
            request (AddTwoInts.Request): Contains a and b integers
            response (AddTwoInts.Response): Object to populate with sum

        Returns:
            AddTwoInts.Response: Populated response object
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AdditionService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
