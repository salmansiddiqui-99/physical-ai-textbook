#!/usr/bin/env python3
"""
Simple ROS 2 Subscriber Node

Purpose: Subscribe to /chatter topic and print received messages
Inputs: std_msgs/String messages on /chatter
Outputs: Console logging of received data

Usage:
    # Terminal 1: Start publisher
    python3 minimal_publisher.py

    # Terminal 2: Start subscriber
    source /opt/ros/humble/setup.bash
    chmod +x minimal_subscriber.py
    python3 minimal_subscriber.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal subscriber that receives string messages.

    Demonstrates:
    - Topic subscription
    - Callback-based message processing
    """

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscription: topic, message type, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('MinimalSubscriber node started')

    def listener_callback(self, msg):
        """
        Called automatically when a message arrives on /chatter.

        Args:
            msg (std_msgs.msg.String): The received message
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
