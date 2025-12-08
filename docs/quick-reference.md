---
id: quick-reference
title: Quick Reference Guide
sidebar_label: Quick Reference
sidebar_position: 2
description: Essential commands and concepts for ROS 2 humanoid robotics
---

# Quick Reference Guide

Essential commands, concepts, and troubleshooting for Module 1.

---

## ROS 2 Essential Commands

### Node Management

```bash
# List all running nodes
ros2 node list

# Get node information
ros2 node info /node_name

# Kill a node
# Use Ctrl+C in the terminal running the node
```

### Topic Operations

```bash
# List all topics
ros2 topic list

# Show topic info (message type, publishers, subscribers)
ros2 topic info /topic_name

# Echo messages from a topic
ros2 topic echo /topic_name

# Publish to a topic (one-time)
ros2 topic pub /topic_name std_msgs/msg/String "{data: 'Hello'}"

# Publish continuously at 10 Hz
ros2 topic pub -r 10 /topic_name std_msgs/msg/String "{data: 'Hello'}"

# Measure topic frequency
ros2 topic hz /topic_name

# Measure topic bandwidth
ros2 topic bw /topic_name
```

### Service Operations

```bash
# List all services
ros2 service list

# Call a service
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Get service type
ros2 service type /service_name

# Find services of a specific type
ros2 service find example_interfaces/srv/AddTwoInts
```

### Action Operations

```bash
# List all actions
ros2 action list

# Send action goal
ros2 action send_goal /action_name action_type "{goal_fields}"

# Get action info
ros2 action info /action_name
```

### Parameter Operations

```bash
# List parameters of a node
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name parameter_name

# Set parameter value
ros2 param set /node_name parameter_name value
```

---

## Common Message Types

### std_msgs

```python
from std_msgs.msg import String, Int32, Float32, Bool

# String message
msg = String()
msg.data = "Hello, ROS 2!"

# Numeric messages
int_msg = Int32()
int_msg.data = 42

float_msg = Float32()
float_msg.data = 3.14
```

### geometry_msgs

```python
from geometry_msgs.msg import Twist, Pose, Point

# Twist (velocity command)
cmd = Twist()
cmd.linear.x = 0.5   # m/s forward
cmd.angular.z = 0.2  # rad/s rotation

# Point
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 0.0
```

### sensor_msgs

```python
from sensor_msgs.msg import JointState, Imu

# Joint states
joint_msg = JointState()
joint_msg.name = ['joint1', 'joint2']
joint_msg.position = [0.5, 1.0]
joint_msg.velocity = [0.1, 0.2]
joint_msg.effort = [5.0, 10.0]

# IMU data
imu_msg = Imu()
imu_msg.orientation.x = 0.0
imu_msg.orientation.y = 0.0
imu_msg.orientation.z = 0.0
imu_msg.orientation.w = 1.0
```

---

## URDF Quick Reference

### Basic Link

```xml
<link name="link_name">
  <visual>
    <geometry>
      <box size="0.1 0.1 0.1"/>
      <!-- Or: <cylinder radius="0.05" length="0.2"/> -->
      <!-- Or: <sphere radius="0.05"/> -->
    </geometry>
    <material name="color_name">
      <color rgba="1 0 0 1"/>  <!-- Red -->
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.001" ixy="0" ixz="0"
             iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### Basic Joint

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation around Y-axis -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Joint Types

- **revolute**: Rotation with limits (e.g., knee 0° to 150°)
- **continuous**: Unlimited rotation (e.g., wheels)
- **prismatic**: Linear motion (e.g., telescoping)
- **fixed**: No motion (rigidly attached)
- **floating**: 6-DOF free motion (base link)

### URDF Validation

```bash
# Check URDF syntax
check_urdf robot.urdf

# Visualize in RViz
ros2 launch urdf_tutorial display.launch.py model:=robot.urdf

# Generate kinematic tree
urdf_to_graphviz robot.urdf
```

---

## Xacro Macros

### Property Definition

```xml
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="wheel_width" value="0.02"/>
```

### Macro Definition

```xml
<xacro:macro name="wheel" params="prefix reflect">
  <link name="${prefix}_wheel">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </visual>
  </link>

  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel"/>
    <origin xyz="0 ${reflect * 0.2} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</xacro:macro>
```

### Macro Usage

```xml
<!-- Call macro for left and right wheels -->
<xacro:wheel prefix="left" reflect="1"/>
<xacro:wheel prefix="right" reflect="-1"/>
```

### Convert Xacro to URDF

```bash
ros2 run xacro xacro robot.urdf.xacro > robot.urdf
```

---

## Common Issues and Solutions

### ROS 2 Not Found

```bash
# Error: ros2: command not found
# Solution: Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### No Module Named 'rclpy'

```bash
# Error: ModuleNotFoundError: No module named 'rclpy'
# Solution: Install ROS 2 Python packages
sudo apt install python3-rclpy

# Or source ROS 2 environment
source /opt/ros/humble/setup.bash
```

### Topic Not Publishing

```bash
# Check if node is running
ros2 node list

# Check topic existence
ros2 topic list

# Check topic info
ros2 topic info /topic_name

# Echo topic to see messages
ros2 topic echo /topic_name
```

### URDF Not Loading

```bash
# Validate URDF syntax
check_urdf robot.urdf

# Common issues:
# - Missing parent/child links
# - Invalid joint types
# - Malformed XML
# - Circular dependencies
```

---

## Python Node Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')

        # Publisher
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            10
        )

        # Timer
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Node initialized')

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Coordinate Frames

### ROS 2 Convention

- **X**: Forward
- **Y**: Left
- **Z**: Up

### Rotation (rpy - Roll Pitch Yaw)

- **Roll**: Rotation around X-axis
- **Pitch**: Rotation around Y-axis
- **Yaw**: Rotation around Z-axis

### Quaternion to Euler

```python
import math

def quaternion_to_euler(x, y, z, w):
    # Roll (x-axis rotation)
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))

    # Pitch (y-axis rotation)
    pitch = math.asin(2*(w*y - z*x))

    # Yaw (z-axis rotation)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))

    return roll, pitch, yaw
```

---

## Debugging Tips

### Enable Debug Logging

```bash
ros2 run --prefix 'ros2 topic echo' package_name node_name
```

### Use RQT Tools

```bash
# Node graph visualization
rqt_graph

# Plot topic data
rqt_plot /topic_name/field

# Message publisher GUI
rqt_publisher

# Service caller GUI
rqt_service_caller

# All-in-one dashboard
rqt
```

### Check TF Tree

```bash
# View transform tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo parent_frame child_frame
```

---

## Performance Tips

### Reduce CPU Usage

```python
# Use appropriate QoS settings
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(depth=10)
qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

self.publisher_ = self.create_publisher(String, 'topic', qos)
```

### Optimize Callback Frequency

```python
# Use timers instead of high-rate subscriptions
# Bad: subscribing to 1000 Hz topic and processing each message
# Good: subscribing and processing at needed rate (e.g., 10 Hz)

self.timer = self.create_timer(0.1, self.process_data)  # 10 Hz
```

---

## Next Steps

- **Practice**: Try modifying the code examples in `static/code/module1/`
- **Experiment**: Create your own custom messages and services
- **Build**: Design a simple humanoid leg with URDF
- **Advanced**: Integrate multiple sensors in a single controller

---

**Quick Links**:
- [Chapter 1.1: ROS 2 Fundamentals](./module1-ros2/ch1-ros2-basics.md)
- [Chapter 1.2: ROS 2 for Humanoids](./module1-ros2/ch2-ros2-humanoids.md)
- [Chapter 1.3: URDF for Humanoids](./module1-ros2/ch3-urdf-humanoids.md)
- [Course Introduction](./intro.md)
