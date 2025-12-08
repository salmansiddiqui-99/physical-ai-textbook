#!/usr/bin/env python3
"""
Simple ROS 2 Publisher Node

Purpose: Publish string messages to /chatter topic at 2 Hz
Inputs: None
Outputs: std_msgs/String messages on /chatter

Usage:
    source /opt/ros/humble/setup.bash
    chmod +x minimal_publisher.py
    python3 minimal_publisher.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal publisher that sends periodic string messages.

    This node demonstrates:
    - Node initialization and inheritance
    - Timer-based callbacks
    - Topic publishing with std_msgs
    """

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publisher: topic name, message type, queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer: period in seconds, callback function
        timer_period = 0.5  # 2 Hz (0.5 seconds between messages)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0  # Message counter
        self.get_logger().info('MinimalPublisher node started')

    def timer_callback(self):
        """
        Called every 0.5 seconds by the timer.
        Publishes a string message with an incrementing counter.
        """
        msg = String()
        msg.data = f'Hello, ROS 2! Message count: {self.i}'
        self.publisher_.publish(msg)

        # Log to console (viewable with ros2 topic echo)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()

    try:
        rclpy.spin(node)  # Keep node alive until Ctrl+C
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
