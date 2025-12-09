#!/usr/bin/env python3
"""
Bipedal Footstep Planner for Humanoid Navigation

Purpose: Generate footstep sequences for stable bipedal locomotion
Environment: ROS 2 with Nav2 integration
Dependencies: nav2_msgs, geometry_msgs

Usage:
    ros2 run humanoid_navigation bipedal_planner
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math


class BipedalFootstepPlanner(Node):
    """
    Footstep planner for humanoid robots

    Converts Nav2 waypoint paths into discrete footstep sequences
    considering bipedal constraints (stride length, step height, balance)
    """

    def __init__(self):
        super().__init__('bipedal_footstep_planner')

        # Robot parameters
        self.declare_parameter('stride_length', 0.5)  # Maximum stride (meters)
        self.declare_parameter('step_width', 0.12)  # Lateral foot separation (meters)
        self.declare_parameter('max_step_height', 0.2)  # Maximum step up/down (meters)
        self.declare_parameter('min_turn_radius', 0.5)  # Minimum turning radius (meters)
        self.declare_parameter('step_duration', 0.6)  # Time per step (seconds)

        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/plan',  # Global path from Nav2 planner
            self.path_callback,
            10
        )

        # Publishers
        self.footstep_pub = self.create_publisher(
            MarkerArray,
            '/footsteps',  # For RViz visualization
            10
        )

        self.footstep_path_pub = self.create_publisher(
            Path,
            '/footstep_path',  # Discrete footstep poses
            10
        )

        self.get_logger().info('Bipedal footstep planner initialized')

    def path_callback(self, path_msg: Path):
        """
        Convert continuous path to discrete footstep sequence

        Args:
            path_msg: Nav2 global path (waypoints)
        """
        if len(path_msg.poses) < 2:
            self.get_logger().warn('Path too short for footstep planning')
            return

        # Generate footsteps
        footsteps = self.plan_footsteps(path_msg.poses)

        # Publish for visualization
        self.publish_footstep_markers(footsteps, path_msg.header)
        self.publish_footstep_path(footsteps, path_msg.header)

        self.get_logger().info(f'Generated {len(footsteps)} footsteps')

    def plan_footsteps(self, waypoints):
        """
        Generate footstep sequence from waypoints

        Returns:
            List of dicts: [{'foot': 'left'|'right', 'pose': (x, y, theta)}, ...]
        """
        footsteps = []
        stride_length = self.get_parameter('stride_length').value
        step_width = self.get_parameter('step_width').value

        # Start with feet together at first waypoint
        start_pose = waypoints[0].pose
        current_x = start_pose.position.x
        current_y = start_pose.position.y
        current_theta = self.quaternion_to_yaw(start_pose.orientation)

        # Alternate feet (start with left)
        current_foot = 'left'

        # Walk along waypoints
        for i in range(1, len(waypoints)):
            goal_pose = waypoints[i].pose
            goal_x = goal_pose.position.x
            goal_y = goal_pose.position.y
            goal_theta = self.quaternion_to_yaw(goal_pose.orientation)

            # Compute distance and bearing to goal
            dx = goal_x - current_x
            dy = goal_y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            bearing = math.atan2(dy, dx)

            # Generate footsteps until reaching this waypoint
            while distance > stride_length / 2:
                # Compute next foot position
                step_x = current_x + stride_length * math.cos(bearing)
                step_y = current_y + stride_length * math.sin(bearing)

                # Offset laterally based on foot (left=positive, right=negative)
                lateral_offset = step_width / 2 if current_foot == 'left' else -step_width / 2
                step_x += lateral_offset * math.cos(bearing + math.pi / 2)
                step_y += lateral_offset * math.sin(bearing + math.pi / 2)

                # Interpolate orientation toward goal
                step_theta = self.interpolate_angle(current_theta, goal_theta, 0.1)

                # Add footstep
                footsteps.append({
                    'foot': current_foot,
                    'pose': (step_x, step_y, step_theta)
                })

                # Update state
                current_x, current_y = step_x, step_y
                current_theta = step_theta
                current_foot = 'right' if current_foot == 'left' else 'left'

                # Recompute distance
                dx = goal_x - current_x
                dy = goal_y - current_y
                distance = math.sqrt(dx**2 + dy**2)

            # Final step to exact goal position
            footsteps.append({
                'foot': current_foot,
                'pose': (goal_x, goal_y, goal_theta)
            })
            current_x, current_y, current_theta = goal_x, goal_y, goal_theta
            current_foot = 'right' if current_foot == 'left' else 'left'

        return footsteps

    def publish_footstep_markers(self, footsteps, header):
        """Visualize footsteps as colored footprints in RViz"""
        marker_array = MarkerArray()

        for i, step in enumerate(footsteps):
            marker = Marker()
            marker.header = header
            marker.ns = "footsteps"
            marker.id = i
            marker.type = Marker.CUBE  # Footprint as box
            marker.action = Marker.ADD

            # Position
            x, y, theta = step['pose']
            marker.pose.position = Point(x=x, y=y, z=0.0)
            marker.pose.orientation = self.yaw_to_quaternion(theta)

            # Size (footprint dimensions)
            marker.scale.x = 0.25  # Foot length
            marker.scale.y = 0.1  # Foot width
            marker.scale.z = 0.02  # Foot height (thin)

            # Color (left=blue, right=red)
            if step['foot'] == 'left':
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            marker.color.a = 0.7  # Semi-transparent

            marker.lifetime = rclpy.duration.Duration(seconds=60.0).to_msg()
            marker_array.markers.append(marker)

        self.footstep_pub.publish(marker_array)

    def publish_footstep_path(self, footsteps, header):
        """Publish footsteps as Path message for controllers"""
        path = Path()
        path.header = header

        for step in footsteps:
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            x, y, theta = step['pose']
            pose_stamped.pose.position = Point(x=x, y=y, z=0.0)
            pose_stamped.pose.orientation = self.yaw_to_quaternion(theta)
            path.poses.append(pose_stamped)

        self.footstep_path_pub.publish(path)

    @staticmethod
    def quaternion_to_yaw(q: Quaternion) -> float:
        """Convert quaternion to yaw angle (radians)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert yaw angle (radians) to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def interpolate_angle(a1, a2, t):
        """Interpolate between two angles (radians) with wrapping"""
        diff = a2 - a1
        # Wrap to [-pi, pi]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return a1 + t * diff


def main(args=None):
    rclpy.init(args=args)
    node = BipedalFootstepPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
