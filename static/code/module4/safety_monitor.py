#!/usr/bin/env python3
"""
Safety Monitor for Humanoid Robot

Purpose: Emergency stop, collision detection, battery monitoring
Environment: ROS 2 Humble, sensor_msgs
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, PointCloud2
from std_msgs.msg import Bool, String
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from enum import Enum


class SafetyLevel(Enum):
    """Safety alert levels"""
    NORMAL = "normal"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class SafetyMonitor(Node):
    """Monitor robot safety and trigger emergency stops"""

    def __init__(self):
        super().__init__('safety_monitor')

        # Parameters
        self.declare_parameter('min_obstacle_distance', 0.3)  # meters
        self.declare_parameter('min_battery_percent', 15.0)
        self.declare_parameter('max_tilt_angle', 15.0)  # degrees
        self.declare_parameter('emergency_stop_enabled', True)

        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.min_battery_percent = self.get_parameter('min_battery_percent').value
        self.max_tilt_angle = self.get_parameter('max_tilt_angle').value
        self.estop_enabled = self.get_parameter('emergency_stop_enabled').value

        # State
        self.safety_level = SafetyLevel.NORMAL
        self.is_stopped = False
        self.closest_obstacle_distance = float('inf')
        self.battery_percent = 100.0
        self.tilt_angle = 0.0

        # Publishers
        self.estop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_status_pub = self.create_publisher(String, '/safety/status', 10)
        self.estop_signal_pub = self.create_publisher(Bool, '/safety/emergency_stop', 10)

        # Subscriptions
        self.create_subscription(
            PointCloud2,
            '/stereo/points2',
            self.obstacle_callback,
            10
        )
        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        # Timer for periodic safety checks
        self.create_timer(0.1, self.safety_check_timer)  # 10 Hz

        self.get_logger().info('Safety monitor active')
        self.get_logger().info(f'  Min obstacle distance: {self.min_obstacle_distance}m')
        self.get_logger().info(f'  Min battery: {self.min_battery_percent}%')
        self.get_logger().info(f'  Max tilt: {self.max_tilt_angle}°')

    def obstacle_callback(self, msg: PointCloud2):
        """
        Check for obstacles too close to robot

        Args:
            msg: Point cloud from stereo depth camera
        """
        try:
            # Convert point cloud to numpy array
            points = []
            for point in pc2.read_points(msg, skip_nans=True):
                x, y, z = point[:3]
                # Only consider points in front of robot (x > 0) and within height range
                if x > 0 and 0.2 < z < 2.0:  # Ground to head height
                    distance = np.sqrt(x**2 + y**2)
                    points.append(distance)

            if points:
                self.closest_obstacle_distance = min(points)

                # Check if too close
                if self.closest_obstacle_distance < self.min_obstacle_distance:
                    self.trigger_emergency_stop(
                        f"Obstacle detected at {self.closest_obstacle_distance:.2f}m "
                        f"(min: {self.min_obstacle_distance}m)"
                    )
            else:
                self.closest_obstacle_distance = float('inf')

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def battery_callback(self, msg: BatteryState):
        """
        Monitor battery level

        Args:
            msg: Battery state message
        """
        self.battery_percent = msg.percentage * 100

        if self.battery_percent < self.min_battery_percent:
            if self.battery_percent < 10.0:
                self.trigger_emergency_stop(
                    f"Critical battery: {self.battery_percent:.1f}% - "
                    "Initiating emergency shutdown"
                )
            else:
                self.get_logger().warn(
                    f'Low battery: {self.battery_percent:.1f}% - '
                    'Consider returning to charger'
                )

    def safety_check_timer(self):
        """Periodic safety checks and status publishing"""
        # Determine current safety level
        new_level = self.evaluate_safety_level()

        # Publish status if changed
        if new_level != self.safety_level:
            self.safety_level = new_level
            self.publish_safety_status()

    def evaluate_safety_level(self) -> SafetyLevel:
        """
        Evaluate current safety level based on all sensors

        Returns:
            Current safety level
        """
        # Check for emergency conditions
        if self.battery_percent < 10.0:
            return SafetyLevel.EMERGENCY
        if self.closest_obstacle_distance < 0.15:  # 15cm
            return SafetyLevel.EMERGENCY
        if self.tilt_angle > self.max_tilt_angle:
            return SafetyLevel.EMERGENCY

        # Check for critical conditions
        if self.battery_percent < self.min_battery_percent:
            return SafetyLevel.CRITICAL
        if self.closest_obstacle_distance < self.min_obstacle_distance:
            return SafetyLevel.CRITICAL

        # Check for warnings
        if self.battery_percent < 20.0:
            return SafetyLevel.WARNING
        if self.closest_obstacle_distance < 0.5:
            return SafetyLevel.WARNING

        return SafetyLevel.NORMAL

    def trigger_emergency_stop(self, reason: str):
        """
        Trigger emergency stop

        Args:
            reason: Human-readable reason for emergency stop
        """
        if not self.estop_enabled:
            self.get_logger().warn(f'Emergency stop disabled: {reason}')
            return

        if self.is_stopped:
            return  # Already stopped

        self.get_logger().error(f'EMERGENCY STOP: {reason}')

        # Publish zero velocity
        stop_cmd = Twist()  # All zeros
        self.estop_pub.publish(stop_cmd)

        # Publish emergency stop signal
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_signal_pub.publish(estop_msg)

        self.is_stopped = True
        self.safety_level = SafetyLevel.EMERGENCY
        self.publish_safety_status()

    def publish_safety_status(self):
        """Publish current safety status"""
        status_msg = String()
        status_msg.data = (
            f"Safety Level: {self.safety_level.value} | "
            f"Battery: {self.battery_percent:.1f}% | "
            f"Nearest Obstacle: {self.closest_obstacle_distance:.2f}m | "
            f"E-Stop: {'ACTIVE' if self.is_stopped else 'INACTIVE'}"
        )
        self.safety_status_pub.publish(status_msg)

        # Log based on severity
        if self.safety_level == SafetyLevel.EMERGENCY:
            self.get_logger().error(status_msg.data)
        elif self.safety_level == SafetyLevel.CRITICAL:
            self.get_logger().warn(status_msg.data)
        elif self.safety_level == SafetyLevel.WARNING:
            self.get_logger().warn(status_msg.data)

    def reset_emergency_stop(self):
        """Reset emergency stop (requires manual intervention)"""
        if not self.is_stopped:
            return

        self.get_logger().info('Resetting emergency stop')

        estop_msg = Bool()
        estop_msg.data = False
        self.estop_signal_pub.publish(estop_msg)

        self.is_stopped = False
        self.safety_level = SafetyLevel.NORMAL
        self.publish_safety_status()


def main(args=None):
    rclpy.init(args=args)
    monitor = SafetyMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
