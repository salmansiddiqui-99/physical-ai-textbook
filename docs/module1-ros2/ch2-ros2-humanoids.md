---
id: ch2-ros2-humanoids
title: Chapter 1.2 - ROS 2 for Humanoid Robots
sidebar_label: ROS 2 for Humanoids
sidebar_position: 2
description: Apply ROS 2 to humanoid robotics with action servers, joint state control, sensor fusion, and multi-controller coordination.
keywords:
  - humanoid robotics
  - ROS 2 actions
  - joint state
  - sensor fusion
  - ros2_control
  - IMU
prerequisites:
  - Chapter 1.1 completed (ROS 2 nodes, topics, services)
  - Understanding of robotic kinematics (joints, links)
  - Familiarity with PID control basics
learning_objectives:
  - Implement ROS 2 action servers for long-running humanoid behaviors
  - Publish and subscribe to joint state and command messages
  - Integrate IMU and force sensor data into ROS 2 control loops
  - Coordinate multiple controllers for locomotion and manipulation
estimated_time: 120 minutes
---

# Chapter 1.2 - ROS 2 for Humanoid Robots

## Learning Objectives

By the end of this chapter, you will be able to:

- Implement ROS 2 action servers to execute goal-based behaviors like "walk forward 2 meters"
- Publish and subscribe to `sensor_msgs/JointState` for controlling humanoid joints
- Integrate IMU data (`sensor_msgs/Imu`) for balance and orientation feedback
- Design multi-controller architectures using `ros2_control` for coordinating locomotion and manipulation

## Prerequisites

Before starting this chapter, you should:

- Have completed [Chapter 1.1: ROS 2 Fundamentals](./ch1-ros2-basics.md) (nodes, topics, services)
- Understand robotic kinematics basics (revolute/prismatic joints, coordinate frames)
- Have ROS 2 Humble installed with `ros-humble-ros2-control` and `ros-humble-controller-manager`

## Introduction

Humanoid robots present unique control challenges compared to wheeled or industrial robots. With 20-40+ degrees of freedom, multiple contact points (feet, hands), and the constant need to maintain balance, humanoid control requires sophisticated coordination across many subsystems.

ROS 2 provides the infrastructure for this complexity through **actions** (goal-based behaviors), **joint state management**, and **controller interfaces**. In this chapter, you'll learn how to apply ROS 2 to humanoid-specific problems: walking controllers, balance feedback from IMUs, force sensing for contact detection, and coordinating arm and leg movements simultaneously.

By the end of this chapter, you'll build a simulated humanoid controller that can receive high-level commands ("walk forward"), execute them using action servers, and provide real-time feedback on joint positions and sensor states—the foundation for full humanoid autonomy.

---

## Section 1: Actions for Goal-Based Behaviors

While **services** provide request-response communication, many humanoid tasks are **long-running** and require **feedback** during execution. For example, "walk to waypoint (2, 0)" might take 10 seconds and should report progress every 100ms. ROS 2 **actions** are designed for exactly this pattern.

### Subsection 1.1: Action Anatomy

A ROS 2 action consists of three message types:

1. **Goal**: The desired behavior (e.g., `target_pose`, `distance_to_walk`)
2. **Feedback**: Periodic status updates (e.g., `current_pose`, `percentage_complete`)
3. **Result**: Final outcome when the action completes (e.g., `success`, `final_error`)

Actions support:
- **Cancellation**: Client can abort long-running actions
- **Feedback**: Server sends progress updates during execution
- **Asynchronous execution**: Multiple goals can be queued

### Subsection 1.2: When to Use Actions vs. Services

| Use Case | Mechanism | Example |
|----------|-----------|---------|
| Instant response, no progress | **Service** | "Get current joint angles" |
| Long-running task with feedback | **Action** | "Walk 5 meters forward" (10+ seconds) |
| Continuous data stream | **Topic** | IMU readings at 100 Hz |

**Key Terminology**:
- **Action Server**: Node that executes the long-running task and publishes feedback
- **Action Client**: Node that sends goals and receives feedback/results
- **Goal Handle**: Reference to a specific action goal for tracking and cancellation
- **Feedback Callback**: Function called periodically as action progresses

---

## Section 2: Implementing a Humanoid Action Server

Let's build an action server for a humanoid "walk forward" behavior. This demonstrates how to structure goal-based locomotion commands.

### Example: Walk Forward Action Server

**Purpose**: Accept distance goals and simulate walking behavior with feedback.

**Environment**: ROS 2 Humble

**Dependencies**:

```bash
source /opt/ros/humble/setup.bash

# Install action interfaces (included with ROS 2)
sudo apt install ros-humble-example-interfaces

# For custom actions (covered later):
# sudo apt install ros-humble-action-msgs
```

**Action Definition**:

We'll use a custom action definition. Create `WalkForward.action`:

```
# Goal: Distance to walk in meters
float32 distance
---
# Result: Whether walk completed successfully
bool success
float32 distance_traveled
---
# Feedback: Current progress
float32 current_distance
float32 percentage_complete
```

For this example, we'll simulate using the built-in `fibonacci` action pattern. In production, you'd build and install custom actions in a ROS 2 package.

**Action Server Code**:

```python
#!/usr/bin/env python3
"""
Humanoid Walk Forward Action Server

Purpose: Execute "walk forward" goals with real-time feedback
Action: custom_interfaces/action/WalkForward (simplified as example here)
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
                return Fibonacci.Result()

            # Simulate walking progress
            current_distance += step_distance
            percentage = (current_distance / target_distance) * 100

            # Publish feedback
            feedback_msg.sequence = [int(current_distance * 10)]  # Placeholder
            goal_handle.publish_feedback(feedback_msg)

            # Log progress
            self.get_logger().info(f'Progress: {current_distance:.2f}m / {target_distance:.2f}m ({percentage:.1f}%)')

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
```

**Code Explanation**:

1. **Lines 30-35**: `ActionServer()` creates an action server with a callback that executes when goals are received. The callback runs in a separate thread, allowing the node to handle multiple concurrent actions.

2. **Lines 53-58**: We check `goal_handle.is_cancel_requested` periodically to support cancellation. If the client cancels, we stop execution and return early.

3. **Lines 61-67**: Feedback is published using `goal_handle.publish_feedback()`. In a real humanoid, this would include current odometry, joint states, and contact sensor status.

4. **Lines 69-72**: We publish `Twist` messages to `/cmd_vel`, which a locomotion controller would use to generate joint trajectories. In simulation (Gazebo/Isaac), this topic directly controls the robot's motion.

5. **Lines 77-82**: When the goal is reached, we call `goal_handle.succeed()` and return a result message. The action client receives this and knows the behavior completed.

**Action Client Code**:

```python
#!/usr/bin/env python3
"""
Humanoid Walk Forward Action Client

Purpose: Send walk forward goals to the action server
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class WalkForwardActionClient(Node):
    """Action client for sending walk forward commands."""

    def __init__(self):
        super().__init__('walk_forward_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'walk_forward')

    def send_goal(self, distance):
        """Send a walk forward goal."""
        goal_msg = Fibonacci.Goal()
        goal_msg.order = int(distance * 10)  # Placeholder encoding

        self.get_logger().info(f'Sending goal: Walk {distance}m forward')

        # Wait for action server
        self._action_client.wait_for_server()

        # Send goal and register callbacks
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle server's acceptance/rejection of goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for action to complete
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Called periodically as action progresses."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.sequence}')

    def result_callback(self, future):
        """Called when action completes."""
        result = future.result().result
        self.get_logger().info(f'Action completed! Result: {result.sequence}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WalkForwardActionClient()

    # Send goal: walk 2 meters forward
    node.send_goal(2.0)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

**Running the Action**:

```bash
# Terminal 1: Start action server
source /opt/ros/humble/setup.bash
python3 walk_action_server.py

# Terminal 2: Send goal from client
source /opt/ros/humble/setup.bash
python3 walk_action_client.py

# Terminal 3: Monitor cmd_vel output
source /opt/ros/humble/setup.bash
ros2 topic echo /cmd_vel
```

**Expected Output**:

Server terminal:
```
[INFO] [walk_forward_action_server]: Walk Forward Action Server started
[INFO] [walk_forward_action_server]: Executing walk forward action...
[INFO] [walk_forward_action_server]: Progress: 0.10m / 2.00m (5.0%)
[INFO] [walk_forward_action_server]: Progress: 0.20m / 2.00m (10.0%)
...
[INFO] [walk_forward_action_server]: Walk forward action completed!
```

Client terminal:
```
[INFO] [walk_forward_action_client]: Sending goal: Walk 2.0m forward
[INFO] [walk_forward_action_client]: Goal accepted, waiting for result...
[INFO] [walk_forward_action_client]: Feedback: [1]
[INFO] [walk_forward_action_client]: Feedback: [2]
...
[INFO] [walk_forward_action_client]: Action completed! Result: [20]
```

**Troubleshooting**:

| Issue | Solution |
|-------|----------|
| "Action server not available" | Ensure server is running before starting client; check action name matches (`walk_forward`) |
| No feedback received | Verify `feedback_callback` is registered in `send_goal_async()` |
| Action doesn't cancel | Implement proper cancellation handling in `execute_callback` |
| Multiple goals interfere | Use `ServerGoalHandle` to track individual goals independently |

---

## Section 3: Joint State Management

Humanoid robots have dozens of joints (ankles, knees, hips, shoulders, elbows, etc.). ROS 2 provides standard interfaces for publishing and commanding these joints.

### Subsection 3.1: JointState Messages

The `sensor_msgs/JointState` message represents the current state of all robot joints:

```
# Joint names (e.g., ["left_hip_pitch", "left_knee", "right_hip_pitch", ...])
string[] name

# Joint positions in radians (revolute) or meters (prismatic)
float64[] position

# Joint velocities (rad/s or m/s)
float64[] velocity

# Joint efforts (torque in N⋅m or force in N)
float64[] effort

# Header with timestamp
std_msgs/Header header
```

### Subsection 3.2: Publishing Joint States

Here's a simple joint state publisher for a 2-DOF leg:

```python
#!/usr/bin/env python3
"""
Humanoid Joint State Publisher

Purpose: Publish joint states for a simple 2-DOF leg (hip and knee)
Outputs: sensor_msgs/JointState messages on /joint_states
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class JointStatePublisher(Node):
    """Publishes simulated joint states for a humanoid leg."""

    def __init__(self):
        super().__init__('joint_state_publisher')

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 100 Hz

        self.time_sec = 0.0
        self.get_logger().info('Joint State Publisher started (100 Hz)')

    def publish_joint_states(self):
        """Publish current joint positions, velocities, and efforts."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Joint names (must match URDF)
        msg.name = ['left_hip_pitch', 'left_knee']

        # Simulate sinusoidal joint motion (walking gait)
        hip_angle = 0.5 * math.sin(2 * math.pi * 0.5 * self.time_sec)  # ±0.5 rad
        knee_angle = 0.8 * math.sin(2 * math.pi * 0.5 * self.time_sec + math.pi/4)

        msg.position = [hip_angle, knee_angle]

        # Calculate velocities (derivative of position)
        msg.velocity = [
            0.5 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * self.time_sec),
            0.8 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * self.time_sec + math.pi/4)
        ]

        # Simulate joint torques (efforts)
        msg.effort = [10.0, 5.0]  # N⋅m

        self.publisher_.publish(msg)
        self.time_sec += 0.01  # Increment time


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Running and Visualizing**:

```bash
# Terminal 1: Publish joint states
python3 joint_state_publisher.py

# Terminal 2: Echo joint states
ros2 topic echo /joint_states --once

# Terminal 3: Plot joint positions in real-time (requires plotjuggler)
sudo apt install ros-humble-plotjuggler-ros
ros2 run plotjuggler plotjuggler
```

---

## Section 4: IMU and Force Sensor Integration

Humanoid robots rely on **Inertial Measurement Units (IMUs)** for balance and **force sensors** in feet for contact detection.

### Subsection 4.1: IMU Data in ROS 2

The `sensor_msgs/Imu` message contains:

- **Orientation** (quaternion): Roll, pitch, yaw angles
- **Angular velocity** (rad/s): Rotation rates around x, y, z axes
- **Linear acceleration** (m/s²): Acceleration in x, y, z directions

### Example: IMU-Based Balance Monitor

```python
#!/usr/bin/env python3
"""
Humanoid IMU Balance Monitor

Purpose: Subscribe to IMU data and detect balance loss
Inputs: sensor_msgs/Imu messages on /imu/data
Outputs: Warnings when tilt exceeds safe thresholds
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class BalanceMonitor(Node):
    """Monitors IMU data to detect dangerous tilt angles."""

    def __init__(self):
        super().__init__('balance_monitor')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Balance thresholds (radians)
        self.max_pitch = math.radians(15)  # 15 degrees forward/backward
        self.max_roll = math.radians(10)   # 10 degrees left/right

        self.get_logger().info('Balance Monitor started')

    def imu_callback(self, msg):
        """Process IMU data and check balance."""
        # Extract quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
        pitch = math.asin(2*(qw*qy - qz*qx))
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

        # Check balance thresholds
        if abs(pitch) > self.max_pitch:
            self.get_logger().warn(
                f'BALANCE WARNING: Pitch angle {math.degrees(pitch):.1f}° exceeds safe limit!'
            )

        if abs(roll) > self.max_roll:
            self.get_logger().warn(
                f'BALANCE WARNING: Roll angle {math.degrees(roll):.1f}° exceeds safe limit!'
            )

        # Log angular velocity (for fall detection)
        angular_vel = msg.angular_velocity
        total_rotation_rate = math.sqrt(
            angular_vel.x**2 + angular_vel.y**2 + angular_vel.z**2
        )

        if total_rotation_rate > 3.0:  # rad/s
            self.get_logger().warn(
                f'RAPID ROTATION DETECTED: {total_rotation_rate:.2f} rad/s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BalanceMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Testing with Simulated IMU Data**:

```bash
# Terminal 1: Start balance monitor
python3 balance_monitor.py

# Terminal 2: Publish test IMU data
ros2 topic pub /imu/data sensor_msgs/msg/Imu "{
  orientation: {x: 0.1, y: 0.2, z: 0.0, w: 0.975},
  angular_velocity: {x: 0.0, y: 0.0, z: 0.5},
  linear_acceleration: {x: 0.0, y: 0.0, z: 9.81}
}"
```

---

## Section 5: ros2_control Framework

The `ros2_control` framework provides a standardized interface for managing robot controllers. It's used by humanoid platforms like PAL Robotics' TALOS and Agility Robotics' Digit.

### Subsection 5.1: Controller Manager Architecture

The **Controller Manager** loads and manages multiple controllers:

- **Joint Trajectory Controller**: Executes smooth joint motions
- **Joint State Broadcaster**: Publishes current joint states
- **Diff Drive Controller**: For wheeled bases (if applicable)
- **Admittance Controller**: For force-controlled manipulation

### Subsection 5.2: Loading Controllers

```bash
# List available controllers
ros2 control list_controllers

# Load joint trajectory controller
ros2 control load_controller joint_trajectory_controller

# Start controller
ros2 control switch_controllers --start joint_trajectory_controller

# Send trajectory commands
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['left_hip_pitch', 'left_knee'], ...}"
```

---

## Hands-On Project: Humanoid Balance Controller

**Goal**: Build a system that monitors IMU data and adjusts ankle joint positions to maintain balance.

**Duration**: 45 minutes

**What You'll Learn**:
- Integrating IMU feedback into control loops
- Commanding joint positions based on sensor data
- Using actions for high-level behaviors

### Step 1: Set Up IMU Publisher (Simulated)

Create `imu_simulator.py`:

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random


class IMUSimulator(Node):
    def __init__(self):
        super().__init__('imu_simulator')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)
        self.time = 0.0

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate slight tilt with noise
        pitch = 0.1 * math.sin(self.time) + random.uniform(-0.05, 0.05)

        # Simplified quaternion (pitch rotation only)
        msg.orientation.x = 0.0
        msg.orientation.y = math.sin(pitch / 2)
        msg.orientation.z = 0.0
        msg.orientation.w = math.cos(pitch / 2)

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.1 * math.cos(self.time)
        msg.angular_velocity.z = 0.0

        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        self.publisher_.publish(msg)
        self.time += 0.01


def main(args=None):
    rclpy.init(args=args)
    node = IMUSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2: Create Balance Controller

Create `balance_controller.py`:

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publish joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # PID gains for ankle control
        self.kp = 2.0  # Proportional gain

        self.get_logger().info('Balance Controller started')

    def imu_callback(self, msg):
        # Extract pitch angle
        qy = msg.orientation.y
        qw = msg.orientation.w
        pitch = math.asin(2 * (qw * qy))

        # Calculate corrective ankle angle (proportional control)
        ankle_correction = -self.kp * pitch

        # Publish ankle command
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ['left_ankle_pitch', 'right_ankle_pitch']
        cmd.position = [ankle_correction, ankle_correction]

        self.joint_pub.publish(cmd)

        self.get_logger().info(
            f'Pitch: {math.degrees(pitch):.2f}°, Ankle correction: {math.degrees(ankle_correction):.2f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Run and Test

```bash
# Terminal 1: Start IMU simulator
python3 imu_simulator.py

# Terminal 2: Start balance controller
python3 balance_controller.py

# Terminal 3: Monitor joint commands
ros2 topic echo /joint_commands
```

**Checkpoint**: You should see the balance controller adjusting ankle angles in response to simulated IMU pitch angles. The corrections should be opposite to the tilt direction (negative feedback).

---

## Challenge: Test Your Understanding

Try these exercises to reinforce your learning:

1. **Basic**: Modify the action server to accept a `speed` parameter (m/s) in addition to distance.
   - *Hint*: Add a field to the action goal and use it to adjust the `cmd_vel.linear.x` value.

2. **Intermediate**: Extend the balance controller to handle roll (left/right tilt) using ankle roll joints.
   - *Hint*: Extract roll from quaternion and apply corrections to `left_ankle_roll` and `right_ankle_roll`.

3. **Advanced**: Implement a force sensor subscriber that detects when the robot's feet lose contact with the ground and triggers an emergency stop.
   - *Hint*: Subscribe to `geometry_msgs/WrenchStamped` on `/left_foot/force` and check if `force.z < threshold`.

**Solutions**: Test your implementations by observing joint commands and system behavior under different IMU conditions.

---

## Summary

In this chapter, you learned:

- **ROS 2 actions**: Goal-based behaviors with feedback for long-running tasks like walking and manipulation
- **Joint state management**: Publishing and subscribing to `sensor_msgs/JointState` for controlling humanoid joints
- **Sensor integration**: Using IMU data (`sensor_msgs/Imu`) for balance monitoring and control
- **ros2_control framework**: Industry-standard controller management for complex robotic systems

**Key Commands**:

```bash
ros2 action list                           # List all active action servers
ros2 action send_goal /action_name <type> "goal_yaml"  # Send action goal from CLI
ros2 topic echo /joint_states              # Monitor joint positions
ros2 control list_controllers              # List loaded controllers
ros2 control switch_controllers --start <name>  # Start a controller
```

**Core Concepts**:
- **Action**: Three-part interface (goal, feedback, result) for long-running tasks
- **JointState**: Standard message for robot joint positions, velocities, and efforts
- **IMU**: Inertial Measurement Unit providing orientation and acceleration data for balance

---

## Further Reading

Official Documentation:
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [ros2_control Documentation](https://control.ros.org/)
- [sensor_msgs Package Reference](https://docs.ros2.org/latest/api/sensor_msgs/)

Tutorials and Examples:
- [ROS 2 Control Demos](https://github.com/ros-controls/ros2_control_demos)
- [Humanoid Controller Examples (PAL Robotics)](https://github.com/pal-robotics)

Research Papers:
- "Humanoid Robot Control with ROS 2" - IEEE Robotics and Automation Magazine
- "Real-time Balance Control for Bipedal Humanoids" - IROS 2023

---

## Next Steps

You're now ready to move on to the next chapter!

**Next Chapter**: [Chapter 1.3: URDF for Humanoid Robots](./ch3-urdf-humanoids.md)

In the next chapter, you'll learn how to describe humanoid robot morphology using URDF (Unified Robot Description Format)—defining links, joints, collision models, and visual meshes for simulation and motion planning.

**Optional Practice**:
- Implement a "reach to target" action server for a 7-DOF humanoid arm
- Create a gait pattern generator that publishes full-body joint trajectories
- Integrate force/torque sensors in wrists for contact-aware manipulation
