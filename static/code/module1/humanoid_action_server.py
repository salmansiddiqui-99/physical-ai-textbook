#!/usr/bin/env python3
"""
Humanoid Walk Forward Action Server

Purpose: Execute "walk forward" goals with real-time feedback
Action: Simulates humanoid walking behavior with distance tracking
Environment: ROS 2 Humble

Usage:
    # Terminal 1: Start action server
    source /opt/ros/humble/setup.bash
    chmod +x humanoid_action_server.py
    python3 humanoid_action_server.py

    # Terminal 2: Send goal (using ROS 2 CLI)
    ros2 action send_goal /walk_forward example_interfaces/action/Fibonacci "{order: 20}"

Note: This uses Fibonacci action as a placeholder. In production, create a custom
WalkForward.action with distance (float32) goal, current_distance (float32) feedback,
and success (bool) + distance_traveled (float32) result.
"""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci  # Using as a placeholder
from geometry_msgs.msg import Twist


class WalkForwardActionServer(Node):
    """
    Action server that simulates a humanoid walking forward.

    In a real robot, this would:
    1. Send joint trajectories to leg controllers
    2. Monitor foot contact sensors
    3. Adjust gait based on IMU feedback
    4. Publish odometry updates
    """

    def __init__(self):
        super().__init__('walk_forward_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Placeholder; use WalkForward in production
            'walk_forward',
            self.execute_callback
        )

        # Publisher for velocity commands (would go to locomotion controller)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Walk Forward Action Server started')

    def execute_callback(self, goal_handle):
        """
        Execute the walk forward action.

        Args:
            goal_handle: Handle for this action goal

        Returns:
            Result message when action completes
        """
        self.get_logger().info('Executing walk forward action...')

        # In real robot: goal_handle.request.distance
        target_distance = 2.0  # meters
        current_distance = 0.0
        step_distance = 0.1  # Simulate 0.1m progress per iteration
        feedback_msg = Fibonacci.Feedback()

        # Simulate walking with periodic feedback
        while current_distance < target_distance:
            # Check if action was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Action canceled')

                # Stop robot
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)

                return Fibonacci.Result()

            # Simulate walking progress
            current_distance += step_distance
            percentage = (current_distance / target_distance) * 100

            # Publish feedback
            feedback_msg.sequence = [int(current_distance * 10)]  # Placeholder
            goal_handle.publish_feedback(feedback_msg)

            # Log progress
            self.get_logger().info(
                f'Progress: {current_distance:.2f}m / {target_distance:.2f}m ({percentage:.1f}%)'
            )

            # Send velocity command to locomotion controller
            cmd = Twist()
            cmd.linear.x = 0.2  # 0.2 m/s forward
            self.cmd_vel_pub.publish(cmd)

            time.sleep(0.5)  # 2 Hz update rate

        # Action completed successfully
        goal_handle.succeed()

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        result = Fibonacci.Result()
        result.sequence = [int(target_distance * 10)]  # Placeholder
        self.get_logger().info('Walk forward action completed!')

        return result


def main(args=None):
    rclpy.init(args=args)
    node = WalkForwardActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
