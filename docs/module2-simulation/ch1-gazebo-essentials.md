---
id: ch1-gazebo-essentials
title: "Chapter 2.1: Gazebo Essentials for Humanoid Simulation"
sidebar_label: Gazebo Essentials
sidebar_position: 4
description: Master Gazebo physics simulation for humanoid robots including world building, collision detection, and rigid body dynamics.
keywords:
  - Gazebo
  - physics simulation
  - SDF
  - world files
  - collision detection
  - rigid body dynamics
prerequisites:
  - Module 1 completed (ROS 2 and URDF fundamentals)
  - Basic understanding of physics (gravity, friction, forces)
  - Linux terminal proficiency
learning_objectives:
  - Create and configure Gazebo simulation worlds
  - Spawn humanoid URDF models in Gazebo
  - Configure physics parameters for realistic humanoid behavior
  - Debug simulation issues using Gazebo tools
estimated_time: 100 minutes
---

# Chapter 2.1 - Gazebo Essentials

## Learning Objectives

By the end of this chapter, you will be able to:

- Create Gazebo world files with custom environments for humanoid testing
- Spawn and control URDF-based humanoid robots in simulation
- Configure physics engines (gravity, friction, collision properties) for realistic behavior
- Use Gazebo GUI tools and ROS 2 integration for robot debugging

## Prerequisites

Before starting this chapter, you should:

- Have completed [Module 1](../module1-ros2/ch1-ros2-basics.md) (ROS 2 and URDF)
- Understand basic physics concepts (forces, torque, center of mass)
- Have Gazebo Classic or Gazebo Fortress installed with ROS 2 Humble

## Introduction

**Gazebo** is the industry-standard robot simulator used by researchers and companies worldwide—from NASA's Mars rovers to Boston Dynamics' humanoid prototypes. Before deploying expensive hardware, engineers test algorithms in Gazebo's physics-accurate environment.

For humanoid robotics, simulation is critical because:
- **Safety**: Test falling, collision, and recovery behaviors without hardware damage
- **Speed**: Iterate on control algorithms 10-100x faster than real-time
- **Repeatability**: Reproduce exact scenarios for debugging and validation
- **Cost**: Avoid wear-and-tear on mechanical components during development

In this chapter, you'll learn how to create simulation worlds, spawn your URDF humanoids from Module 1, configure physics properties, and integrate with ROS 2 for closed-loop control. By the end, you'll have a virtual humanoid laboratory for testing any control algorithm safely.

---

## Section 1: Gazebo Architecture and Setup

Gazebo consists of three main components that work together to simulate robots.

### Subsection 1.1: Gazebo Components

1. **Gazebo Server (gzserver)**: Physics engine that runs the simulation
   - Computes rigid body dynamics using ODE, Bullet, or DART
   - Handles collision detection and contact forces
   - Updates sensor data (cameras, LiDAR, IMU)
   - Publishes simulation state

2. **Gazebo Client (gzclient)**: 3D visualization GUI
   - Renders robot models and environments
   - Provides interaction tools (move objects, measure distances)
   - Displays sensor visualizations
   - Can run separately from server (headless simulation)

3. **Gazebo Plugins**: Extend functionality
   - Sensor plugins (camera, LiDAR, IMU)
   - Actuator plugins (joint motors, grippers)
   - World plugins (custom physics, logging)
   - ROS 2 integration (ros_gz_bridge)

### Subsection 1.2: Gazebo vs. Gazebo Classic

| Feature | Gazebo Classic | Gazebo (Ignition/Fortress+) |
|---------|----------------|---------------------------|
| **Physics Engines** | ODE, Bullet, Simbody | DART, TPE |
| **File Format** | SDF 1.6-1.9 | SDF 1.8+ |
| **ROS 2 Integration** | ros2_gazebo_ros_pkgs | ros_gz |
| **Performance** | Good | Better (multithreading) |
| **Use Case** | ROS 2 Humble (default) | Newer distributions |

**For this course**: We use **Gazebo Classic** with ROS 2 Humble (most common setup).

### Subsection 1.3: Verifying Installation

```bash
# Check Gazebo version
gazebo --version

# Launch empty Gazebo world
gazebo

# Launch with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

Expected output: Gazebo GUI opens with an empty world (ground plane, sun lighting).

**Key Terminology**:
- **SDF (Simulation Description Format)**: XML format for worlds and models
- **World**: Complete simulation environment (terrain, lighting, physics settings)
- **Model**: Individual entity (robot, obstacle, furniture)
- **Link**: Rigid body within a model (same as URDF)
- **Joint**: Connection between links (same as URDF)
- **Plugin**: Shared library that extends Gazebo functionality

---

## Section 2: Creating Your First World

Gazebo worlds are defined in **.world** files using SDF XML format.

### Example: Simple World with Ground Plane

**Purpose**: Create a minimal world for humanoid testing with proper lighting and physics.

**Code** (`simple_world.world`):

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="humanoid_test_world">

    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom ground with friction -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>   <!-- Friction coefficient -->
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

**Code Explanation**:

1. **Lines 6-11**: Physics configuration
   - `max_step_size`: Integration timestep (1ms for humanoid stability)
   - `real_time_factor`: 1 = real-time, &gt;1 = faster, &lt;1 = slower
   - `real_time_update_rate`: 1000 Hz update rate
   - `gravity`: Standard Earth gravity (9.81 m/s²)

2. **Lines 14-21**: Built-in models
   - Gazebo includes common models (ground, sun, obstacles)
   - `<include>` imports from Gazebo model database

3. **Lines 33-38**: Surface friction
   - `mu` and `mu2`: Friction coefficients (1.0 = concrete-like)
   - Critical for humanoid foot-ground contact

**Launching the World**:

```bash
# Method 1: Direct launch
gazebo simple_world.world

# Method 2: With ROS 2
ros2 launch gazebo_ros gazebo.launch.py world:=simple_world.world
```

**Expected Output**: Gazebo opens with a gray ground plane and sunlight.

---

## Section 3: Spawning Humanoid Robots

Now let's spawn the URDF humanoid we created in Module 1 into our Gazebo world.

### Subsection 3.1: URDF to Gazebo

To use URDF in Gazebo, add **Gazebo-specific** tags for:
- **Collision properties**: Friction, restitution (bounce)
- **Visual materials**: Colors that Gazebo recognizes
- **Sensor plugins**: Camera, IMU, LiDAR
- **Actuator plugins**: Joint controllers

### Example: Gazebo-Enhanced URDF

Add these tags to your `humanoid_full.urdf` from Module 1:

```xml
<!-- Inside <robot> tag, after link/joint definitions -->

<!-- Gazebo material for each link -->
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>1.0</mu1>  <!-- Friction coefficient 1 -->
  <mu2>1.0</mu2>  <!-- Friction coefficient 2 -->
</gazebo>

<gazebo reference="left_foot">
  <material>Gazebo/Black</material>
  <mu1>1.5</mu1>  <!-- High friction for foot -->
  <mu2>1.5</mu2>
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>      <!-- Contact damping -->
</gazebo>

<!-- Repeat for each link -->

<!-- ROS 2 Control plugin -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### Subsection 3.2: Spawning with ROS 2

Create a launch file to spawn the humanoid:

**Code** (`spawn_humanoid.launch.py`):

```python
#!/usr/bin/env python3
"""
Launch file to spawn humanoid robot in Gazebo.

Usage:
    ros2 launch my_robot_bringup spawn_humanoid.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    robot_description_dir = get_package_share_directory('my_robot_description')

    # Path to URDF file
    urdf_file = os.path.join(robot_description_dir, 'urdf', 'humanoid_full.urdf')

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch Gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'simple_world.world'}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',  # Spawn 1m above ground
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
```

**Code Explanation**:

1. **Lines 27-33**: Launch Gazebo with custom world
2. **Lines 36-46**: Spawn robot entity
   - `-entity`: Name in simulation
   - `-topic`: Where to read URDF from
   - `-x`, `-y`, `-z`: Initial position
3. **Lines 49-54**: Publish robot state to TF tree

**Running the Launch File**:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Launch
ros2 launch my_robot_bringup spawn_humanoid.launch.py
```

**Expected Output**:
- Gazebo opens with your humanoid standing at (0, 0, 1)
- Robot appears in 3D viewer
- `/robot_description` topic published
- TF frames available for all links

**Troubleshooting**:

| Issue | Solution |
|-------|----------|
| Robot falls through ground | Increase `<kp>` (contact stiffness) in foot collision |
| Robot explodes/jitters | Reduce timestep to 0.0001, increase link masses |
| No robot appears | Check URDF path, verify `robot_description` topic with `ros2 topic echo` |
| Joints don't move | Add ros2_control plugin and controllers |

---

## Section 4: Physics Configuration

Gazebo's physics engine determines how realistically your humanoid behaves.

### Subsection 4.1: Physics Engine Options

```xml
<physics type="ode">  <!-- Or: bullet, dart, simbody -->
  <!-- Solver settings -->
  <max_step_size>0.001</max_step_size>         <!-- 1ms timestep -->
  <real_time_factor>1.0</real_time_factor>      <!-- 1x real-time -->
  <real_time_update_rate>1000</real_time_rate> <!-- 1000 Hz -->

  <!-- ODE-specific (most common) -->
  <ode>
    <solver>
      <type>quick</type>           <!-- quick | world -->
      <iters>50</iters>            <!-- Solver iterations -->
      <sor>1.3</sor>               <!-- Successive over-relaxation -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>               <!-- Constraint force mixing -->
      <erp>0.2</erp>               <!-- Error reduction parameter -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>

  <!-- Gravity (can be modified at runtime) -->
  <gravity>0 0 -9.81</gravity>
</physics>
```

**Key Parameters for Humanoids**:
- **max_step_size**: 0.001 (1ms) for stability, 0.0001 for high-precision
- **iters**: 50-200 for complex contact scenarios (feet on ground)
- **cfm**: 0 for rigid, >0 for compliant joints
- **contact_surface_layer**: Small values (0.001) for precise contact

### Subsection 4.2: Contact Surface Properties

```xml
<!-- Per-link surface properties -->
<gazebo reference="left_foot">
  <mu1>1.5</mu1>              <!-- Friction coefficient (lateral) -->
  <mu2>1.5</mu2>              <!-- Friction coefficient (longitudinal) -->
  <kp>1000000.0</kp>          <!-- Contact stiffness (N/m) -->
  <kd>100.0</kd>              <!-- Contact damping (N⋅s/m) -->
  <minDepth>0.001</minDepth>  <!-- Minimum penetration before contact -->
  <maxVel>0.1</maxVel>        <!-- Maximum contact correction velocity -->
</gazebo>
```

**Material Friction Values** (Reference):
- Ice: 0.05
- Wood on wood: 0.4
- Rubber on concrete: 0.9-1.5 ✓ (Humanoid foot)
- Steel on steel: 0.8

---

## Section 5: Gazebo GUI Tools

The Gazebo GUI provides essential debugging tools.

### Subsection 5.1: View Menu Tools

1. **Wireframe Mode**: See collision geometries
   - View → Wireframe
   - Helps debug invisible collision boxes

2. **Transparent Mode**: See internal links
   - View → Transparent
   - Useful for multi-link robots

3. **Joints**: Visualize joint axes
   - View → Joints
   - Shows rotation/translation axes

4. **Center of Mass**: Display COM markers
   - View → Center of Mass
   - Critical for balance analysis

5. **Contacts**: Show contact points
   - View → Contacts
   - Verify foot-ground contact

### Subsection 5.2: Useful Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| **Space** | Pause/unpause simulation |
| **Ctrl+R** | Reset world |
| **T** | Toggle translation mode |
| **R** | Toggle rotation mode |
| **S** | Toggle scale mode |
| **Shift+Ctrl+T** | Show model tree |

### Subsection 5.3: Inspection Tools

```bash
# List all models in simulation
gz model --list

# Get model pose
gz model -m humanoid -p

# Set model pose
gz model -m humanoid -x 1.0 -y 2.0 -z 1.0

# Apply force to model
gz model -m humanoid -f "10 0 0"

# Get joint positions
ros2 topic echo /joint_states
```

---

## Hands-On Project: Humanoid Drop Test

**Goal**: Test your humanoid's contact physics by dropping it from height and analyzing impact forces.

**Duration**: 30 minutes

**What You'll Learn**:
- Configure realistic contact physics
- Measure impact forces
- Tune damping for stable landing

### Step 1: Create Drop Test World

```xml
<!-- drop_test.world -->
<sdf version="1.6">
  <world name="drop_test">
    <physics type="ode">
      <max_step_size>0.0001</max_step_size>  <!-- High precision -->
      <real_time_update_rate>10000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <!-- Soft landing pad -->
    <model name="landing_pad">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>2 2 0.1</size></box>
          </geometry>
          <surface>
            <contact>
              <ode>
                <kp>10000</kp>   <!-- Softer than ground -->
                <kd>100</kd>
              </ode>
            </contact>
          </surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Step 2: Launch with Humanoid at 2m Height

```bash
# Terminal 1: Launch Gazebo
ros2 launch gazebo_ros gazebo.launch.py world:=drop_test.world

# Terminal 2: Spawn humanoid at height
ros2 run gazebo_ros spawn_entity.py -entity humanoid -file humanoid_full.urdf \
    -x 0 -y 0 -z 2.0
```

### Step 3: Monitor Impact Forces

```python
#!/usr/bin/env python3
"""Monitor foot contact forces during drop test."""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState

class ImpactMonitor(Node):
    def __init__(self):
        super().__init__('impact_monitor')
        self.subscription = self.create_subscription(
            ContactsState,
            '/left_foot/contacts',
            self.contact_callback,
            10
        )
        self.max_force = 0.0

    def contact_callback(self, msg):
        if len(msg.states) > 0:
            # Get total contact force
            force = msg.states[0].total_wrench.force
            force_magnitude = (force.x**2 + force.y**2 + force.z**2)**0.5

            if force_magnitude > self.max_force:
                self.max_force = force_magnitude
                self.get_logger().info(f'Peak force: {self.max_force:.2f} N')

def main(args=None):
    rclpy.init(args=args)
    node = ImpactMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Checkpoint**: Humanoid drops, lands on feet, and peak force is logged. If robot bounces excessively, increase kd (damping). If it sinks into ground, increase kp (stiffness).

---

## Challenge: Test Your Understanding

Try these exercises to reinforce your learning:

1. **Basic**: Create a world with obstacles (boxes, cylinders) for the humanoid to navigate.
   - *Hint*: Use `<model>` tags with `<static>true</static>` for immovable objects.

2. **Intermediate**: Modify physics parameters to simulate the Moon (gravity = 1.62 m/s²).
   - *Hint*: Change `<gravity>0 0 -1.62</gravity>` and observe walking behavior.

3. **Advanced**: Add a slope (ramp) to the world and test humanoid stability on inclines.
   - *Hint*: Rotate a plane link using `<pose>0 0 0 0 0.1 0</pose>` (roll around X-axis).

**Solutions**: Test by running simulations and verifying robot behavior matches expectations.

---

## Summary

In this chapter, you learned:

- **Gazebo architecture**: Server (physics), client (GUI), plugins (extensibility)
- **World creation**: SDF format, physics settings, lighting, terrain
- **Robot spawning**: Launch files, URDF integration, initial positioning
- **Physics tuning**: Contact properties, friction, solver parameters for humanoids
- **Debugging tools**: GUI overlays, command-line inspection, contact visualization

**Key Commands**:

```bash
gazebo world.world                              # Launch Gazebo with world
ros2 launch gazebo_ros gazebo.launch.py         # Launch with ROS 2
ros2 run gazebo_ros spawn_entity.py -file robot.urdf  # Spawn robot
gz model --list                                 # List models
gz model -m robot_name -p                       # Get model pose
ros2 topic list | grep gazebo                   # List Gazebo topics
```

**Core Concepts**:
- **SDF**: Simulation Description Format for worlds and models
- **Physics Engine**: ODE, Bullet, or DART for rigid body dynamics
- **Contact**: Interaction between colliding surfaces (friction, stiffness, damping)
- **Plugin**: Shared library extending Gazebo (sensors, controllers, logging)

---

## Further Reading

Official Documentation:
- [Gazebo Classic Documentation](http://classic.gazebosim.org/)
- [SDF Format Specification](http://sdformat.org/)
- [ROS 2 Gazebo Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)

Tutorials and Examples:
- [Gazebo Classic Tutorials](http://classic.gazebosim.org/tutorials)
- [ROS 2 Gazebo Demos](https://github.com/ros2/gazebo_ros2_control)

Physics References:
- [ODE (Open Dynamics Engine) Manual](https://www.ode.org/ode-latest-userguide.html)
- [Bullet Physics Documentation](https://pybullet.org/wordpress/)

---

## Next Steps

You're now ready to move on to the next chapter!

**Next Chapter**: [Chapter 2.2: Sensor Simulation](./ch2-sensor-simulation.md)

In the next chapter, you'll learn how to add sensors (LiDAR, depth cameras, IMU) to your humanoid for perception—simulating realistic sensor noise, ray tracing for LiDAR, and camera image generation.

**Optional Practice**:
- Create a world with stairs for the humanoid to climb
- Add dynamic objects (rolling balls) for interaction testing
- Experiment with different physics engines (ode vs. bullet)
- Measure simulation performance (real-time factor) with complex scenes
