---
id: ch2-sensor-simulation
title: "Chapter 2.2: Sensor Simulation in Gazebo"
sidebar_label: "2.2 Sensor Simulation"
sidebar_position: 5
description: Simulating LiDAR, depth cameras, and IMUs for humanoid perception including realistic noise models and ROS 2 integration.
learning_objectives:
  - Configure and simulate LiDAR sensors for environment scanning
  - Integrate depth cameras for 3D perception and point cloud generation
  - Implement IMU sensors with realistic noise models for balance control
  - Analyze sensor data visualization in RViz2 and Gazebo
---

# Chapter 2.2: Sensor Simulation in Gazebo

**Learning Objectives:**
- Configure and simulate LiDAR sensors for environment scanning
- Integrate depth cameras for 3D perception
- Add IMU sensors for orientation and motion sensing
- Understand sensor noise models and realistic simulation
- Visualize sensor data in RViz2 and Gazebo
- Process sensor data in ROS 2 for humanoid control

**Estimated Time**: 120 minutes

---

## Introduction: The Humanoid's Senses

A humanoid robot without sensors is like a person blindfolded in an unfamiliar room. Sensors are the **digital senses** that allow robots to perceive their environment, maintain balance, and navigate obstacles.

In this chapter, you'll learn to simulate three critical sensor types:
- **LiDAR**: Laser-based distance measurement for mapping
- **Depth Cameras**: 3D vision for object detection and manipulation
- **IMUs**: Inertial measurement for balance and orientation

**Why Sensor Simulation Matters:**
- Test perception algorithms without expensive hardware
- Generate training data for machine learning models
- Debug sensor fusion before deploying to real robots
- Evaluate sensor placement and configuration

---

## 1. LiDAR Sensor Simulation

### What is LiDAR?

**LiDAR (Light Detection and Ranging)** uses laser pulses to measure distances. It's essential for:
- Environment mapping (SLAM - Simultaneous Localization and Mapping)
- Obstacle detection
- Navigation planning
- Terrain analysis

### LiDAR Configuration in Gazebo

Gazebo uses the **libgazebo_ros_ray_sensor** plugin for LiDAR simulation.

**Key Parameters:**
- **Update Rate**: How often the sensor publishes (Hz)
- **Range**: Minimum and maximum detection distance (meters)
- **Samples**: Number of laser rays per scan
- **Resolution**: Angular spacing between rays
- **Noise**: Simulated measurement uncertainty

### Example: Adding LiDAR to a Humanoid

Let's add a 2D scanning LiDAR to a humanoid's chest.

**File: `chest_lidar.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- LiDAR sensor mounted on chest -->
  <xacro:macro name="chest_lidar" params="parent">

    <!-- Sensor link -->
    <link name="chest_lidar_link">
      <visual>
        <geometry>
          <cylinder radius="0.03" length="0.05"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="0.03" length="0.05"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.0001" ixy="0" ixz="0"
                 iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>

    <!-- Attach to parent (chest) -->
    <joint name="chest_lidar_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="chest_lidar_link"/>
      <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
    </joint>

    <!-- Gazebo LiDAR plugin -->
    <gazebo reference="chest_lidar_link">
      <sensor type="ray" name="chest_lidar_sensor">
        <pose>0 0 0 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>10</update_rate>

        <ray>
          <scan>
            <horizontal>
              <samples>720</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
              <max_angle>3.14159</max_angle>   <!-- +180 degrees -->
            </horizontal>
          </scan>

          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>

          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>

        <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <namespace>/humanoid</namespace>
            <remapping>~/out:=scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>chest_lidar_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>

  </xacro:macro>

</robot>
```

### Understanding LiDAR Parameters

**Scan Configuration:**
```xml
<samples>720</samples>           <!-- 720 rays in one scan -->
<min_angle>-3.14159</min_angle>  <!-- Start at -180° -->
<max_angle>3.14159</max_angle>   <!-- End at +180° -->
```
- **Angular Resolution**: (max - min) / samples = 360° / 720 = 0.5° per ray
- **Higher samples** = better resolution but more computational cost

**Range Configuration:**
```xml
<min>0.1</min>   <!-- Objects closer than 10 cm are not detected -->
<max>30.0</max>  <!-- Maximum detection range is 30 meters -->
```

**Noise Model:**
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- ±1 cm standard deviation -->
</noise>
```
- **Gaussian noise** simulates real-world sensor uncertainty
- Increase `stddev` for cheaper/noisier sensors

### Visualizing LiDAR in RViz2

**Launch file snippet:**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('your_package'),
                'config', 'lidar_view.rviz'
            )]
        ),
    ])
```

**In RViz2:**
1. Add "LaserScan" display
2. Set topic to `/humanoid/scan`
3. Set size to 0.05 for better visibility
4. Choose color scheme (intensity or flat)

---

## 2. Depth Camera Simulation

### What is a Depth Camera?

A **depth camera** (like Intel RealSense or Microsoft Kinect) provides:
- **RGB image**: Color information
- **Depth image**: Distance to each pixel
- **Point cloud**: 3D coordinates of all points in view

**Use cases for humanoids:**
- Object recognition and manipulation
- Facial recognition for HRI (Human-Robot Interaction)
- Grasping pose estimation
- Obstacle avoidance

### Depth Camera Configuration

Gazebo uses the **libgazebo_ros_camera** plugin with depth capabilities.

**File: `head_depth_camera.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Depth camera in the head -->
  <xacro:macro name="head_depth_camera" params="parent">

    <!-- Camera link -->
    <link name="head_camera_link">
      <visual>
        <geometry>
          <box size="0.05 0.1 0.03"/>
        </geometry>
        <material name="dark_grey">
          <color rgba="0.3 0.3 0.3 1"/>
        </material>
      </visual>

      <collision>
        <geometry>
          <box size="0.05 0.1 0.03"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0" ixz="0"
                 iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>

    <!-- Optical frame (standard for cameras in ROS) -->
    <link name="head_camera_optical_frame"/>

    <!-- Attach camera to parent (head) -->
    <joint name="head_camera_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="head_camera_link"/>
      <origin xyz="0.08 0 0.05" rpy="0 0 0"/>
    </joint>

    <!-- Optical frame transformation (ROS camera convention) -->
    <joint name="head_camera_optical_joint" type="fixed">
      <parent link="head_camera_link"/>
      <child link="head_camera_optical_frame"/>
      <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
    </joint>

    <!-- Gazebo depth camera plugin -->
    <gazebo reference="head_camera_link">
      <sensor type="depth" name="head_depth_camera">
        <update_rate>30.0</update_rate>
        <visualize>true</visualize>

        <camera name="head_camera">
          <horizontal_fov>1.047198</horizontal_fov>  <!-- 60 degrees -->
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>10.0</far>
          </clip>

          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>

        <plugin name="head_depth_camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/humanoid</namespace>
            <remapping>~/image_raw:=camera/image_raw</remapping>
            <remapping>~/depth/image_raw:=camera/depth/image_raw</remapping>
            <remapping>~/camera_info:=camera/camera_info</remapping>
            <remapping>~/depth/camera_info:=camera/depth/camera_info</remapping>
            <remapping>~/points:=camera/points</remapping>
          </ros>

          <camera_name>head_camera</camera_name>
          <frame_name>head_camera_optical_frame</frame_name>
          <hack_baseline>0.07</hack_baseline>
          <min_depth>0.05</min_depth>
          <max_depth>10.0</max_depth>
        </plugin>
      </sensor>
    </gazebo>

  </xacro:macro>

</robot>
```

### Understanding Camera Parameters

**Field of View (FOV):**
```xml
<horizontal_fov>1.047198</horizontal_fov>  <!-- 60° in radians -->
```
- Typical values: 60°-90° (1.047 - 1.571 radians)
- Wider FOV = more visible area but lower resolution per degree

**Image Resolution:**
```xml
<width>640</width>
<height>480</height>  <!-- VGA resolution -->
```
- Higher resolution = better detail but more processing
- Common: 640×480 (VGA), 1280×720 (HD), 1920×1080 (Full HD)

**Depth Range:**
```xml
<near>0.05</near>  <!-- 5 cm minimum -->
<far>10.0</far>    <!-- 10 m maximum -->
```

**Optical Frame Convention:**
```xml
<!-- ROS camera convention: X=right, Y=down, Z=forward -->
<origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
```
- This rotation aligns camera frame with ROS standards
- Critical for correct point cloud generation

### Published Topics

The depth camera publishes five topics:

1. **`/humanoid/camera/image_raw`**: RGB image (sensor_msgs/Image)
2. **`/humanoid/camera/depth/image_raw`**: Depth image (sensor_msgs/Image)
3. **`/humanoid/camera/camera_info`**: RGB calibration (sensor_msgs/CameraInfo)
4. **`/humanoid/camera/depth/camera_info`**: Depth calibration (sensor_msgs/CameraInfo)
5. **`/humanoid/camera/points`**: Point cloud (sensor_msgs/PointCloud2)

### Visualizing Depth Data in RViz2

**Add displays in RViz2:**
1. **RGB Image**: Add "Image" display → topic `/humanoid/camera/image_raw`
2. **Depth Image**: Add "Image" display → topic `/humanoid/camera/depth/image_raw`
3. **Point Cloud**: Add "PointCloud2" display → topic `/humanoid/camera/points`

**Point Cloud styling:**
- Color Transformer: "RGB8" (shows real colors)
- Size (m): 0.01
- Style: Points or Squares

---

## 3. IMU Sensor Simulation

### What is an IMU?

An **Inertial Measurement Unit (IMU)** measures:
- **Linear acceleration**: Motion along X, Y, Z axes (m/s²)
- **Angular velocity**: Rotation around X, Y, Z axes (rad/s)
- **Orientation**: Current pose as quaternion (optional, requires sensor fusion)

**Critical for humanoids:**
- Balance control (detect tipping)
- Fall detection
- Gait stabilization
- Odometry correction

### IMU Configuration

Gazebo uses the **libgazebo_ros_imu_sensor** plugin.

**File: `torso_imu.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- IMU sensor in the torso (center of mass) -->
  <xacro:macro name="torso_imu" params="parent">

    <!-- IMU link (usually small/invisible) -->
    <link name="torso_imu_link">
      <inertial>
        <mass value="0.01"/>
        <inertia ixx="0.00001" ixy="0" ixz="0"
                 iyy="0.00001" iyz="0" izz="0.00001"/>
      </inertial>
    </link>

    <!-- Attach to parent (torso) at center of mass -->
    <joint name="torso_imu_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="torso_imu_link"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </joint>

    <!-- Gazebo IMU plugin -->
    <gazebo reference="torso_imu_link">
      <sensor name="torso_imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <visualize>true</visualize>

        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.009</stddev>
                <bias_mean>0.00075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.009</stddev>
                <bias_mean>0.00075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.009</stddev>
                <bias_mean>0.00075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </z>
          </angular_velocity>

          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.017</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.017</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.017</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>

        <plugin name="torso_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <ros>
            <namespace>/humanoid</namespace>
            <remapping>~/out:=imu</remapping>
          </ros>
          <frame_name>torso_imu_link</frame_name>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
        </plugin>
      </sensor>
    </gazebo>

  </xacro:macro>

</robot>
```

### Understanding IMU Noise Parameters

IMUs have two types of errors:

**1. White Noise (stddev):**
```xml
<stddev>0.009</stddev>  <!-- Random noise per measurement -->
```
- High-frequency random fluctuations
- Reduced by low-pass filtering

**2. Bias (drift):**
```xml
<bias_mean>0.00075</bias_mean>      <!-- Constant offset -->
<bias_stddev>0.0000008</bias_stddev>  <!-- Bias variation over time -->
```
- Slowly changing offset (sensor drift)
- Critical for long-term integration (odometry)
- Requires periodic recalibration

**Realistic values** (based on consumer-grade IMUs like MPU-6050):
- Angular velocity noise: ~0.01 rad/s
- Linear acceleration noise: ~0.02 m/s²
- Bias: 0.001-0.1 depending on sensor quality

### IMU Data Structure

The IMU publishes to `/humanoid/imu` with message type `sensor_msgs/Imu`:

```python
# sensor_msgs/Imu structure
orientation:         # Quaternion (x, y, z, w)
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0

angular_velocity:    # rad/s
  x: 0.01
  y: -0.02
  z: 0.005

linear_acceleration: # m/s² (includes gravity!)
  x: 0.1
  y: 0.05
  z: 9.81  # Gravity on Z-axis when stationary
```

**Important:** Linear acceleration includes gravity! When the robot is stationary and upright, you'll see ~9.81 m/s² on the Z-axis.

---

## 4. Sensor Noise Models and Realism

### Why Add Noise to Simulations?

**Real sensors are imperfect.** If you train algorithms on perfect simulated data, they will fail on real robots. Noise simulation provides:
- Robustness testing
- Realistic training data for machine learning
- Algorithm parameter tuning
- Performance benchmarking

### Types of Noise

**1. Gaussian (Normal) Noise:**
```xml
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```
- Most common type
- Models random measurement errors
- Characterized by mean (center) and standard deviation (spread)

**2. Quantization Noise:**
Simulates limited sensor resolution (e.g., 12-bit ADC).

**3. Bias/Drift:**
Slow changes in sensor offset over time.

### Calibrating Noise to Real Sensors

**Step 1:** Collect real sensor data while stationary
```bash
ros2 topic echo /real_robot/imu > imu_data.txt
```

**Step 2:** Calculate standard deviation
```python
import numpy as np

# Load data and calculate stddev
data = np.loadtxt('imu_data.txt')
stddev_gyro = np.std(data[:, 0:3], axis=0)
stddev_accel = np.std(data[:, 3:6], axis=0)

print(f"Gyro noise: {stddev_gyro}")
print(f"Accel noise: {stddev_accel}")
```

**Step 3:** Update Gazebo sensor configuration with measured values

### Sensor Update Rates

**Typical rates:**
- **IMU**: 100-200 Hz (high-rate for control)
- **LiDAR**: 10-40 Hz
- **Depth camera**: 30 Hz (matches video frame rate)

**Trade-off:** Higher rate = more data but more CPU usage.

---

## 5. Sensor Placement for Humanoids

### Best Practices

**IMU Placement:**
- **Torso (center of mass)**: Best for balance and orientation
- Avoid limbs (too much local motion)
- Mount rigidly (vibration adds noise)

**LiDAR Placement:**
- **Chest or waist**: Horizontal scanning for navigation
- **Head**: Multi-plane scanning (but head moves)
- Height: 0.3-1.0 m for obstacle detection

**Camera Placement:**
- **Head**: Natural viewpoint for manipulation
- Tilt down 15-30° to see floor and nearby objects
- Stereo cameras: 6-10 cm baseline for depth

### Example: Full Sensor Suite

```xml
<!-- Torso IMU for balance -->
<xacro:torso_imu parent="torso_link"/>

<!-- Chest LiDAR for navigation -->
<xacro:chest_lidar parent="torso_link"/>

<!-- Head depth camera for manipulation -->
<xacro:head_depth_camera parent="head_link"/>
```

---

## 6. Visualizing Sensor Data

### RViz2 Configuration

**Create a saved configuration** (`sensor_view.rviz`):

1. **Global Options**
   - Fixed Frame: `base_link` or `odom`

2. **TF Display** (show coordinate frames)
   - All Enabled: true

3. **LaserScan**
   - Topic: `/humanoid/scan`
   - Size: 0.05
   - Color: By intensity

4. **Image**
   - Topic: `/humanoid/camera/image_raw`

5. **PointCloud2**
   - Topic: `/humanoid/camera/points`
   - Color Transformer: RGB8
   - Size: 0.01

6. **Imu** (via plugin or custom visualization)
   - Topic: `/humanoid/imu`

### Launch File with RViz2

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('your_humanoid_package')

    return LaunchDescription([
        # Spawn robot in Gazebo (from previous chapter)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, 'launch', 'spawn_humanoid.launch.py')
            ])
        ),

        # RViz2 with sensor visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'sensor_view.rviz')],
            output='screen'
        ),
    ])
```

---

## 7. Processing Sensor Data in ROS 2

### Subscribing to Sensor Topics

**Example: IMU subscriber for tilt detection**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class TiltDetector(Node):
    def __init__(self):
        super().__init__('tilt_detector')

        self.subscription = self.create_subscription(
            Imu,
            '/humanoid/imu',
            self.imu_callback,
            10
        )

        self.tilt_threshold = 0.3  # radians (~17 degrees)

        self.get_logger().info('Tilt detector started')

    def imu_callback(self, msg):
        # Extract orientation (quaternion)
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Convert to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)

        # Check if tilted beyond threshold
        if abs(roll) > self.tilt_threshold or abs(pitch) > self.tilt_threshold:
            self.get_logger().warn(
                f'TILT DETECTED! Roll: {math.degrees(roll):.1f}°, '
                f'Pitch: {math.degrees(pitch):.1f}°'
            )

    def quaternion_to_euler(self, x, y, z, w):
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = TiltDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running the detector:**
```bash
# Terminal 1: Launch Gazebo with sensors
ros2 launch your_package spawn_humanoid_with_sensors.launch.py

# Terminal 2: Run tilt detector
python3 tilt_detector.py

# Terminal 3: Push the robot in Gazebo to see warnings
```

---

## 8. Sensor Fusion with robot_localization

### Why Sensor Fusion?

Individual sensors have weaknesses:
- IMU: Drifts over time
- Odometry: Wheel slip errors
- Vision: Fails in poor lighting

**Sensor fusion** combines multiple sources for better accuracy.

### Using robot_localization Package

The `robot_localization` package implements Extended Kalman Filters (EKF) for fusing:
- IMU data
- Odometry
- GPS (outdoor robots)

**Installation:**
```bash
sudo apt install ros-humble-robot-localization
```

**Configuration file** (`ekf.yaml`):

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: false

    # Frame configuration
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # IMU input
    imu0: /humanoid/imu
    imu0_config: [false, false, false,  # x, y, z position (not from IMU)
                  true,  true,  true,   # roll, pitch, yaw orientation
                  false, false, false,  # x, y, z velocity
                  true,  true,  true,   # roll, pitch, yaw velocity
                  true,  true,  true]   # x, y, z acceleration
    imu0_differential: false
    imu0_relative: false

    # Odometry input (wheel encoders)
    odom0: /humanoid/odom
    odom0_config: [true,  true,  false,  # x, y, z position
                   false, false, true,   # roll, pitch, yaw
                   true,  true,  false,  # x, y, z velocity
                   false, false, true,   # roll, pitch, yaw velocity
                   false, false, false]  # x, y, z acceleration
    odom0_differential: false
    odom0_relative: false

    # Process noise covariance (tuning parameters)
    process_noise_covariance: [0.05, 0.0, 0.0, ...]  # 15x15 matrix
```

**Launch with EKF:**
```python
Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
    output='screen'
)
```

---

## 9. Hands-On Project: Sensor-Based Balance Controller

### Project Goal

Create a controller that uses IMU data to detect when the humanoid is tilting and applies corrective torques to joints.

### Step 1: Create the Controller Node

**File: `balance_controller.py`**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import math

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid/imu', self.imu_callback, 10
        )

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )

        # Control parameters
        self.kp_roll = 50.0   # Proportional gain for roll
        self.kp_pitch = 50.0  # Proportional gain for pitch
        self.kd_roll = 10.0   # Derivative gain for roll
        self.kd_pitch = 10.0  # Derivative gain for pitch

        # State variables
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.previous_roll = 0.0
        self.previous_pitch = 0.0

        # Control loop timer (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('Balance controller started')

    def imu_callback(self, msg):
        # Convert quaternion to Euler angles
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)

        self.previous_roll = self.current_roll
        self.previous_pitch = self.current_pitch
        self.current_roll = roll
        self.current_pitch = pitch

    def control_loop(self):
        # PD control for balance
        # Roll correction (left-right tilt)
        roll_error = 0.0 - self.current_roll  # Target is upright (0)
        roll_rate = (self.current_roll - self.previous_roll) / 0.02
        roll_torque = self.kp_roll * roll_error - self.kd_roll * roll_rate

        # Pitch correction (forward-backward tilt)
        pitch_error = 0.0 - self.current_pitch
        pitch_rate = (self.current_pitch - self.previous_pitch) / 0.02
        pitch_torque = self.kp_pitch * pitch_error - self.kd_pitch * pitch_rate

        # Map torques to ankle joints
        # Simplified: apply roll torque to ankle roll, pitch to ankle pitch
        cmd = Float64MultiArray()
        cmd.data = [
            roll_torque,   # left_ankle_roll
            pitch_torque,  # left_ankle_pitch
            -roll_torque,  # right_ankle_roll (opposite)
            pitch_torque   # right_ankle_pitch
        ]

        self.joint_pub.publish(cmd)

        # Log significant corrections
        if abs(roll_error) > 0.1 or abs(pitch_error) > 0.1:
            self.get_logger().info(
                f'Correcting - Roll error: {math.degrees(roll_error):.1f}°, '
                f'Pitch error: {math.degrees(pitch_error):.1f}°'
            )

    def quaternion_to_euler(self, x, y, z, w):
        # (same as tilt_detector implementation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Test the Controller

**Terminal 1:** Launch Gazebo with humanoid
```bash
ros2 launch your_package spawn_humanoid_with_imu.launch.py
```

**Terminal 2:** Run balance controller
```bash
python3 balance_controller.py
```

**Terminal 3:** Apply external force in Gazebo
- Click on the robot in Gazebo
- Use "Apply Force" tool to push it sideways
- Watch the controller respond to maintain balance

### Expected Behavior

- Robot should resist tilting
- Ankle joints apply corrective torques
- Small oscillations are normal (like human balance)
- Large pushes may cause falling (limited joint torque)

---

## 10. Advanced Topics

### Multi-Sensor Calibration

**Extrinsic calibration** determines the spatial relationship between sensors (e.g., camera relative to LiDAR).

**Tools:**
- `camera_calibration` package (ROS 2)
- `lidar_camera_calibration` for LiDAR-camera alignment

### Sensor Simulation Performance

**Optimizing Gazebo:**
- Reduce update rates for non-critical sensors
- Lower camera resolution during development
- Disable visualization (`<visualize>false</visualize>`)
- Use headless Gazebo (`gzserver` without GUI)

**Benchmarking:**
```bash
# Check real-time factor
gz stats -p

# Target: real_time_factor ≈ 1.0
# < 1.0 means simulation is slower than real-time
```

### Sensor Plugins Beyond Basics

- **GPS**: `libgazebo_ros_gps_sensor`
- **Contact sensors**: `libgazebo_ros_bumper`
- **Force-torque sensors**: `libgazebo_ros_ft_sensor`
- **Sonar/ultrasonic**: `libgazebo_ros_ray_sensor` (configured differently)

---

## Summary

You've learned to simulate three essential sensor types for humanoid robots:

**LiDAR:**
- 2D/3D laser scanning for mapping and navigation
- Configuration: samples, range, update rate, noise
- Visualization in RViz2 as LaserScan

**Depth Cameras:**
- RGB-D sensors for 3D perception
- Outputs: RGB image, depth image, point cloud
- Camera parameters: FOV, resolution, depth range

**IMU:**
- Acceleration and angular velocity measurement
- Noise models: Gaussian noise and bias drift
- Critical for balance and orientation estimation

**Best Practices:**
- Add realistic noise for robustness
- Place sensors strategically (IMU at COM, LiDAR at chest, camera in head)
- Fuse multiple sensors with `robot_localization`
- Visualize all sensor data in RViz2
- Test with external disturbances (pushes, obstacles)

---

## Exercises

### Exercise 1: Tune LiDAR Configuration

Modify the chest LiDAR to:
- 180° field of view (front only)
- 360 samples (0.5° resolution)
- 15 m maximum range
- 0.02 m noise standard deviation

**Test:** Place objects at various distances and verify detection.

### Exercise 2: Dual Camera Setup

Add two cameras to the humanoid head:
- Left camera at `xyz="0.08 0.05 0.05"`
- Right camera at `xyz="0.08 -0.05 0.05"`
- Stereo baseline: 0.1 m

**Test:** Visualize both camera feeds and compare depth perception.

### Exercise 3: IMU Filtering

Implement a complementary filter to combine IMU acceleration (short-term accuracy) with gyroscope integration (drift-free orientation).

**Hint:** `orientation = 0.98 * (prev_orientation + gyro_change) + 0.02 * accel_orientation`

### Exercise 4: Obstacle Avoidance

Create a node that:
- Subscribes to `/humanoid/scan`
- Detects obstacles closer than 1.0 m
- Publishes stop command to `/cmd_vel` when obstacles detected

---

## Troubleshooting

### Problem: No LiDAR Data in RViz2

**Check:**
```bash
# Is the topic publishing?
ros2 topic list | grep scan

# What's the message rate?
ros2 topic hz /humanoid/scan

# Echo to see data
ros2 topic echo /humanoid/scan
```

**Solutions:**
- Ensure `<update_rate>` > 0 in sensor configuration
- Check Fixed Frame in RViz2 matches TF tree
- Verify sensor `<visualize>true</visualize>`

### Problem: Depth Camera Shows Black Image

**Check:**
- Clipping range: `<near>` and `<far>` appropriate
- Object in camera view (not pointing at empty space)
- Lighting in Gazebo world (depth cameras need light)

**Solution:**
```xml
<!-- Add lighting to world -->
<light type="directional" name="sun">
  <pose>0 0 10 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
</light>
```

### Problem: IMU Shows Only Gravity

**This is normal!** When stationary, IMU linear acceleration should be ~9.81 m/s² on Z-axis (gravity).

**To see changes:** Apply forces or move the robot.

### Problem: Gazebo Runs Slowly with Sensors

**Solutions:**
- Reduce camera resolution: `<width>320</width> <height>240</height>`
- Lower LiDAR samples: `<samples>180</samples>`
- Decrease update rates: `<update_rate>5</update_rate>` (for cameras)
- Disable unnecessary visualizations

---

## Additional Resources

**ROS 2 Sensor Documentation:**
- [sensor_msgs Reference](https://docs.ros2.org/latest/api/sensor_msgs/)
- [image_tools Package](https://github.com/ros2/demos/tree/rolling/image_tools)

**Gazebo Sensor Plugins:**
- [Gazebo ROS 2 Sensor Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins)

**Sensor Fusion:**
- [robot_localization Wiki](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [Kalman Filter Tutorial](https://www.kalmanfilter.net/)

**Papers:**
- "Robot Sensor Calibration" (Tutorials on extrinsic calibration)
- "IMU Preintegration for Visual-Inertial Odometry" (Advanced IMU usage)

---

**Next Chapter**: [2.3 Unity Visualization](./ch3-unity-visualization.md) - Create stunning 3D visualizations and human-robot interaction simulations with Unity and ROS 2.

**Previous Chapter**: [2.1 Gazebo Essentials](./ch1-gazebo-essentials.md)

**Module Navigation**: [Course Home](../intro.md)

---

**Estimated Completion Time**: 2 hours
**Prerequisites**: Module 1 (ROS 2 Basics, URDF), Chapter 2.1 (Gazebo Essentials)
**Difficulty**: Intermediate
