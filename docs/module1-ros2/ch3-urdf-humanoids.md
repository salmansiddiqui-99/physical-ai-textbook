---
id: ch3-urdf-humanoids
title: Chapter 1.3 - URDF for Humanoid Robots
sidebar_label: URDF for Humanoids
sidebar_position: 3
description: Master URDF modeling for humanoid robots including kinematic chains, joint definitions, collision geometry, and visual meshes.
keywords:
  - URDF
  - robot description
  - kinematics
  - collision geometry
  - RViz
  - xacro
prerequisites:
  - Chapter 1.1 and 1.2 completed (ROS 2 fundamentals, joint states)
  - Basic understanding of coordinate frames and transformations
  - Familiarity with XML syntax
learning_objectives:
  - Define humanoid robot morphology using URDF XML format
  - Create kinematic chains with revolute and prismatic joints
  - Configure collision and visual geometries for simulation
  - Use Xacro macros to simplify repetitive URDF structures
estimated_time: 110 minutes
---

# Chapter 1.3 - URDF for Humanoid Robots

## Learning Objectives

By the end of this chapter, you will be able to:

- Define humanoid robot morphology using URDF (Unified Robot Description Format)
- Create kinematic chains with revolute joints for limbs and prismatic joints where needed
- Configure collision geometries for physics simulation and visual meshes for rendering
- Use Xacro (XML Macros) to parameterize and simplify complex humanoid descriptions

## Prerequisites

Before starting this chapter, you should:

- Have completed [Chapter 1.1](./ch1-ros2-basics.md) and [Chapter 1.2](./ch2-ros2-humanoids.md)
- Understand coordinate frames and transformations (x, y, z axes, roll-pitch-yaw)
- Have ROS 2 Humble installed with `ros-humble-urdf-tutorial` and `ros-humble-joint-state-publisher-gui`

## Introduction

Every robot in ROS 2—from simple wheeled platforms to 40-DOF humanoids—is described using **URDF (Unified Robot Description Format)**. URDF is an XML-based language that defines a robot's **kinematic structure** (links and joints), **collision geometry** (for physics simulation), **visual appearance** (meshes and colors), and **sensor mounting points**.

For humanoid robots, URDF is critical for:
- **Simulation**: Gazebo and Isaac Sim load URDF to create physics-based models
- **Visualization**: RViz renders the robot model for debugging
- **Motion Planning**: MoveIt uses URDF for inverse kinematics and collision checking
- **Control**: `ros2_control` maps URDF joints to hardware interfaces

In this chapter, you'll build a humanoid robot description from scratch—starting with a simple 2-DOF leg, progressing to a full lower body, and finally adding arms and a torso. You'll learn URDF syntax, best practices for humanoid modeling, and how to use Xacro to avoid repetition.

---

## Section 1: URDF Fundamentals

URDF files define robots as **trees** of connected rigid bodies. The two fundamental building blocks are **links** (rigid bodies) and **joints** (connections between links).

### Subsection 1.1: Links

A **link** represents a rigid body part like a thigh, shin, foot, or torso. Each link can have:

1. **Visual**: How it appears in RViz (meshes, primitive shapes, colors)
2. **Collision**: Simplified geometry for physics collision detection
3. **Inertial**: Mass, center of mass, and inertia tensor for dynamics

**Basic Link Syntax**:

```xml
<link name="left_shin">
  <!-- Visual representation (what you see in RViz) -->
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.4"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>

  <!-- Collision geometry (simplified for physics) -->
  <collision>
    <geometry>
      <cylinder radius="0.045" length="0.4"/>
    </geometry>
  </collision>

  <!-- Inertial properties (for dynamic simulation) -->
  <inertial>
    <mass value="1.5"/>
    <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### Subsection 1.2: Joints

A **joint** connects two links and defines their relative motion. ROS 2 supports six joint types:

| Joint Type | Description | Example in Humanoid |
|------------|-------------|---------------------|
| **revolute** | Rotation with limits (most common) | Knee (0° to 150°), elbow |
| **continuous** | Unlimited rotation | Wheel axles (not common in humanoids) |
| **prismatic** | Linear sliding motion | Telescoping spine (rare) |
| **fixed** | No motion (welded together) | Mounting sensors to links |
| **planar** | 2D motion in a plane | Rarely used |
| **floating** | 6-DOF free motion | Base link for humanoids |

**Basic Joint Syntax**:

```xml
<joint name="left_knee" type="revolute">
  <!-- Parent link (fixed reference) -->
  <parent link="left_thigh"/>

  <!-- Child link (moves relative to parent) -->
  <child link="left_shin"/>

  <!-- Origin: where child attaches to parent (x, y, z, roll, pitch, yaw) -->
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>

  <!-- Axis of rotation (z-axis for pitch rotation) -->
  <axis xyz="0 1 0"/>

  <!-- Joint limits (radians) -->
  <limit lower="0" upper="2.618" effort="100" velocity="2.0"/>

  <!-- Dynamics properties -->
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

**Key Terminology**:
- **Link**: A rigid body component (e.g., shin, thigh, foot)
- **Joint**: Connection defining relative motion between two links
- **Parent-Child**: Parent link is fixed; child link moves relative to parent
- **Origin**: Transform (xyz translation + rpy rotation) from parent to child
- **Axis**: Direction of rotation (for revolute) or translation (for prismatic)

---

## Section 2: Building a Simple 2-DOF Humanoid Leg

Let's build a minimal humanoid leg with two joints: hip and knee.

### Example: Two-Joint Leg URDF

**Purpose**: Define a basic leg structure with hip and knee revolute joints.

**Environment**: ROS 2 Humble

**Dependencies**:

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-urdf-tutorial ros-humble-joint-state-publisher-gui
```

**Code** (`simple_leg.urdf`):

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_leg">

  <!-- Base link (pelvis attachment point) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Thigh link -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="gray">
        <color rgba="0.6 0.6 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.027" ixy="0" ixz="0" iyy="0.027" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Hip joint (revolute) -->
  <joint name="left_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch rotation (forward/backward) -->
    <limit lower="-1.57" upper="1.57" effort="150" velocity="2.0"/>
    <dynamics damping="1.0" friction="0.5"/>
  </joint>

  <!-- Shin link -->
  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Knee joint (revolute) -->
  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch rotation -->
    <limit lower="0" upper="2.618" effort="100" velocity="2.0"/>  <!-- 0° to 150° -->
    <dynamics damping="0.7" friction="0.3"/>
  </joint>

  <!-- Foot link -->
  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Ankle joint (fixed for simplicity) -->
  <joint name="left_ankle" type="fixed">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  </joint>

</robot>
```

**Code Explanation**:

1. **Lines 5-15**: `base_link` is the root of the kinematic tree (pelvis). All other links connect to this directly or indirectly.

2. **Lines 48-54**: `left_hip_pitch` joint connects `base_link` (parent) to `left_thigh` (child). The origin `xyz="0 0.1 0"` places the thigh 0.1m to the left of the base.

3. **Lines 74-80**: `left_knee` joint connects thigh to shin. The origin `xyz="0 0 -0.4"` places the shin 0.4m below the thigh (length of thigh link).

4. **Lines 103-107**: `left_ankle` is a **fixed joint** (no motion), rigidly attaching the foot to the shin.

5. **Inertial Properties** (lines 33-36, 61-64, 94-97): Mass and inertia tensor are required for dynamic simulation in Gazebo. For visualization-only, these can be approximate.

**Visualizing the URDF**:

```bash
# Check URDF syntax
check_urdf simple_leg.urdf

# Visualize in RViz
ros2 launch urdf_tutorial display.launch.py model:=simple_leg.urdf
```

**Expected Output**:

- RViz opens with a 3D view of the leg
- A GUI window appears with sliders for `left_hip_pitch` and `left_knee`
- Moving sliders animates the joint angles in real-time

**Troubleshooting**:

| Issue | Solution |
|-------|----------|
| `check_urdf` reports errors | Verify XML syntax, ensure all `<link>` names are unique, check parent-child relationships form a tree |
| RViz shows nothing | Ensure `Fixed Frame` is set to `base_link` in RViz global options |
| Joints don't move in GUI | Verify joint types are `revolute` (not `fixed`), check joint names match |
| Links appear misaligned | Check `<origin xyz>` values in joints; visualize coordinate frames with `Add -> TF` in RViz |

---

## Section 3: Full Lower Body with Xacro Macros

Humanoid robots have **symmetrical** limbs (left/right legs, left/right arms). Writing URDF manually for both sides leads to duplication and errors. **Xacro** (XML Macros) solves this by parameterizing URDF.

### Subsection 3.1: Xacro Basics

Xacro adds:
- **Properties**: Variables (e.g., `<xacro:property name="thigh_length" value="0.4"/>`)
- **Macros**: Reusable templates (e.g., a leg macro called twice for left/right)
- **Math**: Expressions like `${thigh_length * 2}`

### Example: Parameterized Leg Macro

**Code** (`humanoid_lower_body.urdf.xacro`):

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_lower_body">

  <!-- Properties (constants) -->
  <xacro:property name="thigh_length" value="0.4"/>
  <xacro:property name="thigh_radius" value="0.05"/>
  <xacro:property name="shin_length" value="0.4"/>
  <xacro:property name="shin_radius" value="0.04"/>
  <xacro:property name="foot_length" value="0.2"/>
  <xacro:property name="foot_width" value="0.1"/>
  <xacro:property name="hip_spacing" value="0.2"/>  <!-- Distance between hips -->

  <!-- Base link (pelvis) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 ${hip_spacing} 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Leg macro: defines one complete leg -->
  <xacro:macro name="leg" params="side reflect">
    <!-- side: "left" or "right" (string) -->
    <!-- reflect: 1 for left, -1 for right (flips y-axis) -->

    <!-- Thigh link -->
    <link name="${side}_thigh">
      <visual>
        <geometry>
          <cylinder radius="${thigh_radius}" length="${thigh_length}"/>
        </geometry>
        <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
        <material name="gray">
          <color rgba="0.6 0.6 0.6 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${thigh_radius}" length="${thigh_length}"/>
        </geometry>
        <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="2.0"/>
        <inertia ixx="0.027" ixy="0" ixz="0" iyy="0.027" iyz="0" izz="0.0025"/>
      </inertial>
    </link>

    <!-- Hip joint -->
    <joint name="${side}_hip_pitch" type="revolute">
      <parent link="base_link"/>
      <child link="${side}_thigh"/>
      <origin xyz="0 ${reflect * hip_spacing/2} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="150" velocity="2.0"/>
      <dynamics damping="1.0" friction="0.5"/>
    </joint>

    <!-- Shin link -->
    <link name="${side}_shin">
      <visual>
        <geometry>
          <cylinder radius="${shin_radius}" length="${shin_length}"/>
        </geometry>
        <origin xyz="0 0 ${-shin_length/2}" rpy="0 0 0"/>
        <material name="dark_gray">
          <color rgba="0.3 0.3 0.3 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${shin_radius}" length="${shin_length}"/>
        </geometry>
        <origin xyz="0 0 ${-shin_length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <!-- Knee joint -->
    <joint name="${side}_knee" type="revolute">
      <parent link="${side}_thigh"/>
      <child link="${side}_shin"/>
      <origin xyz="0 0 ${-thigh_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.618" effort="100" velocity="2.0"/>
      <dynamics damping="0.7" friction="0.3"/>
    </joint>

    <!-- Foot link -->
    <link name="${side}_foot">
      <visual>
        <geometry>
          <box size="${foot_length} ${foot_width} 0.05"/>
        </geometry>
        <origin xyz="${foot_length/4} 0 0" rpy="0 0 0"/>
        <material name="black">
          <color rgba="0.1 0.1 0.1 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <box size="${foot_length} ${foot_width} 0.05"/>
        </geometry>
        <origin xyz="${foot_length/4} 0 0" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
      </inertial>
    </link>

    <!-- Ankle joint -->
    <joint name="${side}_ankle_pitch" type="revolute">
      <parent link="${side}_shin"/>
      <child link="${side}_foot"/>
      <origin xyz="0 0 ${-shin_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.785" upper="0.785" effort="50" velocity="2.0"/>
      <dynamics damping="0.5" friction="0.2"/>
    </joint>

  </xacro:macro>

  <!-- Instantiate left and right legs -->
  <xacro:leg side="left" reflect="1"/>
  <xacro:leg side="right" reflect="-1"/>

</robot>
```

**Code Explanation**:

1. **Lines 4-11**: Properties define dimensions once. Changing `thigh_length` updates both legs automatically.

2. **Lines 29-131**: The `leg` macro defines a complete leg (thigh, shin, foot, hip, knee, ankle). Parameters `side` and `reflect` customize each instance.

3. **Line 58**: `${reflect * hip_spacing/2}` places the left hip at +0.1m and right hip at -0.1m (reflected).

4. **Lines 134-135**: Macro is called twice with different parameters, generating left and right legs.

**Converting Xacro to URDF**:

```bash
# Convert Xacro to URDF (for tools that don't support Xacro)
ros2 run xacro xacro humanoid_lower_body.urdf.xacro > humanoid_lower_body.urdf

# Visualize
ros2 launch urdf_tutorial display.launch.py model:=humanoid_lower_body.urdf
```

**Expected Output**:

- RViz displays a humanoid lower body with symmetrical left/right legs
- Joint sliders control 6 joints: left_hip_pitch, left_knee, left_ankle_pitch, right_hip_pitch, right_knee, right_ankle_pitch

---

## Section 4: Adding Visual Meshes and Collision Geometry

For realistic humanoid models, we replace primitive shapes (cylinders, boxes) with **3D meshes** (STL or DAE files).

### Subsection 4.1: Visual vs. Collision Geometry

- **Visual**: High-detail meshes for appearance (can be complex)
- **Collision**: Simplified shapes for physics (must be convex or decomposed)

**Why separate?** Physics engines like Gazebo perform collision detection millions of times per second. Complex meshes slow simulation significantly.

### Example: Using Mesh Files

```xml
<link name="left_thigh">
  <!-- Visual: detailed mesh -->
  <visual>
    <geometry>
      <mesh filename="package://my_robot_description/meshes/thigh.dae" scale="1 1 1"/>
    </geometry>
  </visual>

  <!-- Collision: simplified cylinder (much faster) -->
  <collision>
    <geometry>
      <cylinder radius="0.06" length="0.4"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </collision>

  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.027" ixy="0" ixz="0" iyy="0.027" iyz="0" izz="0.0025"/>
  </inertial>
</link>
```

**Mesh File Formats**:
- **STL**: Binary mesh format (Gazebo Classic, Isaac Sim)
- **DAE (COLLADA)**: XML-based with materials (RViz, Gazebo)
- **OBJ**: Widely supported, no material info

**Best Practices**:
1. Place meshes in `meshes/` directory of your ROS 2 package
2. Reference with `package://` URI (resolved by ROS 2)
3. Keep collision meshes under 1000 triangles for performance
4. Use convex decomposition tools (V-HACD) for concave collision shapes

---

## Section 5: Integrating URDF with ros2_control

To control joints on real hardware, we add **transmission** and **ros2_control** tags to URDF.

### Subsection 5.1: Transmission Elements

A **transmission** maps URDF joints to hardware interfaces (position, velocity, or effort control).

```xml
<transmission name="left_hip_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_hip_pitch">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_hip_motor">
    <mechanicalReduction>100</mechanicalReduction>  <!-- Gear ratio -->
  </actuator>
</transmission>
```

### Subsection 5.2: ros2_control Plugin

Add the ros2_control plugin to load controllers:

```xml
<ros2_control name="humanoid_system" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <joint name="left_hip_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- Repeat for all controlled joints -->
</ros2_control>

<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find my_robot_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

---

## Hands-On Project: Full Humanoid Upper Body

**Goal**: Extend the lower body URDF to include a torso, arms, and head.

**Duration**: 50 minutes

**What You'll Learn**:
- Creating multi-DOF kinematic chains (7-DOF arm)
- Organizing complex URDF with multiple macros
- Testing full-body joint control

### Step 1: Add Torso Link

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.4 0.5"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <inertial>
    <mass value="15.0"/>
    <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.3"/>
  </inertial>
</link>

<joint name="spine" type="fixed">
  <parent link="base_link"/>
  <child link="torso"/>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
</joint>
```

### Step 2: Create Arm Macro (7-DOF)

```xml
<xacro:macro name="arm" params="side reflect">
  <!-- Shoulder link -->
  <link name="${side}_shoulder">
    <visual>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="${side}_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="${side}_shoulder"/>
    <origin xyz="0 ${reflect * 0.25} 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="50" velocity="2.0"/>
  </joint>

  <!-- Upper arm link -->
  <link name="${side}_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="gray"/>
    </visual>
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.009" ixy="0" ixz="0" iyy="0.009" iyz="0" izz="0.0006"/>
    </inertial>
  </link>

  <joint name="${side}_shoulder_roll" type="revolute">
    <parent link="${side}_shoulder"/>
    <child link="${side}_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.5" upper="1.5" effort="50" velocity="2.0"/>
  </joint>

  <!-- Elbow and forearm (add similarly) -->
  <!-- ... -->

</xacro:macro>

<xacro:arm side="left" reflect="1"/>
<xacro:arm side="right" reflect="-1"/>
```

### Step 3: Visualize Full Body

```bash
ros2 run xacro xacro full_humanoid.urdf.xacro > full_humanoid.urdf
ros2 launch urdf_tutorial display.launch.py model:=full_humanoid.urdf
```

**Checkpoint**: You should see a complete humanoid with legs, torso, and arms. All joints should be controllable via the GUI sliders.

---

## Challenge: Test Your Understanding

Try these exercises to reinforce your learning:

1. **Basic**: Add a `head` link connected to the torso with a `neck_pitch` joint (±30°).
   - *Hint*: Create a sphere for the head and attach it above the torso with a revolute joint.

2. **Intermediate**: Implement ankle roll joints (side-to-side tilt) in addition to ankle pitch.
   - *Hint*: Add a new link between shin and foot, with axis `xyz="1 0 0"` for roll rotation.

3. **Advanced**: Create a hand with 3 fingers, each with 2 joints (12 DOF total per hand).
   - *Hint*: Use a nested Xacro macro for fingers, parameterized by finger name and offset.

**Solutions**: Validate your URDF with `check_urdf` and visualize in RViz. Test joint limits by moving sliders to extremes.

---

## Summary

In this chapter, you learned:

- **URDF structure**: Links (rigid bodies) and joints (connections) define robot morphology
- **Xacro macros**: Parameterize and reuse URDF components to avoid duplication
- **Visual vs. collision geometry**: High-detail meshes for appearance, simplified shapes for physics
- **ros2_control integration**: Transmission elements map URDF joints to hardware interfaces

**Key Commands**:

```bash
check_urdf <file>.urdf                  # Validate URDF syntax and structure
ros2 run xacro xacro <file>.xacro       # Convert Xacro to URDF
ros2 launch urdf_tutorial display.launch.py model:=<file>.urdf  # Visualize in RViz
urdf_to_graphviz <file>.urdf            # Generate kinematic tree diagram
```

**Core Concepts**:
- **Link**: Rigid body with visual, collision, and inertial properties
- **Joint**: Connection defining relative motion (revolute, prismatic, fixed)
- **Xacro**: Macro language for parameterizing URDF and reducing duplication

---

## Further Reading

Official Documentation:
- [URDF Tutorials (ROS 2)](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [ros2_control URDF Integration](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html)

Tools and Utilities:
- [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)
- [Blender URDF Tools](https://github.com/dfki-ric/phobos)
- [V-HACD Convex Decomposition](https://github.com/kmammou/v-hacd)

Tutorials and Examples:
- [ROS 2 Humanoid Robot Examples](https://github.com/ros2/examples)
- [PAL Robotics TALOS URDF](https://github.com/pal-robotics/talos_robot)

---

## Next Steps

Congratulations! You've completed **Module 1: ROS 2 Foundations**.

**Next Module**: Module 2: Simulation Environments - Gazebo & Unity (coming soon)

In Module 2, you'll learn how to simulate your humanoid URDF in physics engines—spawning robots in Gazebo, applying forces, simulating sensors (cameras, LiDAR, IMU), and testing control algorithms in safe virtual environments.

**Optional Practice**:
- Export a humanoid CAD model from SolidWorks/Fusion 360 to URDF
- Create a URDF for a quadruped robot (4 legs with 3 DOF each)
- Add RGB cameras and LiDAR sensors to your humanoid's head link
- Build a custom Gazebo world with obstacles and test collision geometry

**Module 1 Complete!** You now have the foundational knowledge to build ROS 2 control systems for humanoid robots.
