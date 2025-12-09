---
id: ch3-navigation-humanoids
title: "Navigation for Humanoids: Bipedal Path Planning with Nav2"
sidebar_label: "Humanoid Navigation"
sidebar_position: 9
description: Learn Nav2 navigation stack for humanoid robots including bipedal motion constraints, dynamic obstacle avoidance, and integration with Isaac ROS perception for autonomous indoor navigation.
keywords:
  - Nav2
  - humanoid navigation
  - bipedal locomotion
  - path planning
  - obstacle avoidance
  - costmap
  - behavior trees
  - footstep planning
prerequisites:
  - Completion of Chapters 3.1 (Isaac Sim) and 3.2 (Isaac ROS Perception)
  - Understanding of ROS 2 actions and services from Module 1
  - Familiarity with costmaps and path planning concepts
learning_objectives:
  - Explain Nav2 architecture and configure navigation stack for bipedal robots
  - Implement bipedal-specific motion constraints and footstep planners
  - Configure costmaps for obstacle avoidance considering humanoid height and stability
  - Integrate Isaac ROS perception (VSLAM, stereo) with Nav2 for autonomous navigation
estimated_time: 90 minutes
---

# Navigation for Humanoids: Bipedal Path Planning with Nav2

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain Nav2 architecture and adapt it for bipedal locomotion constraints
- Implement costmap configuration for humanoid-specific obstacle detection (stairs, low ceilings)
- Configure path planners and controllers for stable bipedal walking
- Integrate Isaac ROS VSLAM and stereo depth with Nav2 for full autonomous navigation

## Prerequisites

Before starting this chapter, you should:

- Have completed [Chapter 3.1: Isaac Sim Basics](./ch1-isaac-sim.md) and [Chapter 3.2: Isaac ROS Perception](./ch2-isaac-ros-perception.md)
- Understand ROS 2 actions (from [Module 1](../module1-ros2/ch2-ros2-humanoids.md))
- Have Nav2 installed (`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`)
- Understand basic path planning concepts (A*, costmaps, dynamic window approach)

## Introduction

Autonomous navigation for humanoid robots is fundamentally different from wheeled robots. A differential-drive robot can rotate in place, move sideways with Mecanum wheels, and stop instantly. A humanoid must maintain balance while walking, cannot stop mid-step without falling, and has kinematic constraints (maximum stride length, step height clearance).

ROS 2 Navigation Stack (**Nav2**) is designed for mobile robots but can be adapted for humanoids with custom planners and controllers. The key is replacing wheeled-robot assumptions (instantaneous velocity changes, planar motion) with bipedal constraints (footstep sequences, center-of-mass stability, swing-leg clearance).

This chapter shows how to configure Nav2 for humanoid robots using Isaac Sim and Isaac ROS perception. You'll implement costmap layers that account for vertical obstacles (humanoid can duck under tables but not drive under), path planners that generate footstep sequences, and behavior trees that coordinate navigation with balancing controllers.

---

## Section 1: Nav2 Architecture for Humanoids

### Subsection 1.1: Nav2 Core Components

Nav2 consists of modular servers communicating via ROS 2 actions:

**Key Servers**:
1. **Map Server**: Provides static map (floor plan) for localization
2. **AMCL / SLAM Toolbox**: Localization (particle filter or SLAM)
3. **Planner Server**: Global path planning (A*, NavFn, Smac Planner)
4. **Controller Server**: Local trajectory execution (DWB, TEB, MPPI)
5. **Recoveries Server**: Behaviors for getting unstuck (rotate, back up)
6. **Behavior Tree (BT) Navigator**: Orchestrates navigation behaviors

**Navigation Workflow**:
1. User sends goal pose to BT Navigator
2. BT Navigator requests global path from Planner Server
3. Controller Server follows path, avoiding dynamic obstacles
4. If blocked, Recoveries Server executes recovery behaviors
5. Repeat until goal reached

### Subsection 1.2: Adapting Nav2 for Bipedal Robots

**Key Modifications for Humanoids**:

| Component | Wheeled Robot | Humanoid Modification |
|-----------|---------------|----------------------|
| **Controller** | DWB (continuous velocity) | Footstep-based MPC or stepped DWB |
| **Costmap** | 2D obstacle layer | 3D voxel layer (head clearance, stairs) |
| **Recovery Behaviors** | Rotate in place | Sidestepping, COM shift (no in-place rotation) |
| **Localization** | Wheel odometry + LIDAR | Visual SLAM (Isaac ROS cuVSLAM) |
| **Path Planning** | Smooth curves | Waypoint-based (discrete foot placements) |

**Humanoid-Specific Constraints**:
- **Minimum step duration**: ~0.4s per step (faster = unstable)
- **Maximum stride length**: Typically 0.5-0.8m for human-sized humanoid
- **Step height clearance**: Must clear obstacles &lt;20cm (e.g., cables, curbs)
- **Turning radius**: Cannot pivot in place (must take multiple steps to rotate)

### Subsection 1.3: Installing Nav2 and Dependencies

```bash
# Install Nav2 (if not already installed)
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install SLAM Toolbox for online mapping
sudo apt install ros-humble-slam-toolbox

# Install Smac Planner (lattice-based planner, good for humanoids)
sudo apt install ros-humble-nav2-smac-planner

# Verify installation
ros2 pkg list | grep nav2
```

---

## Section 2: Costmap Configuration for Humanoids

### Subsection 2.1: What is a Costmap?

A **costmap** is a 2D/3D grid where each cell has a cost (0-254):
- **0**: Free space (robot can traverse)
- **253**: Inscribed obstacle (within robot footprint)
- **254**: Lethal obstacle (collision)
- **1-252**: Proximity to obstacles (higher = closer)

**Costmap Layers** (combined via max operator):
1. **Static Layer**: From pre-built map (walls, furniture)
2. **Obstacle Layer**: From sensors (laser, depth camera)
3. **Inflation Layer**: Expands obstacles by robot radius + safety margin
4. **Voxel Layer**: 3D obstacles for height-aware navigation

### Subsection 2.2: Humanoid Costmap Parameters

**Key Differences for Humanoids**:

```yaml
# File: humanoid_costmap_common.yaml
costmap_common_params:
  robot_radius: 0.35  # Shoulder width / 2 + margin (meters)

  # Observation sources (sensors)
  observation_sources: lidar_scan stereo_point_cloud

  # LiDAR at chest height (detects obstacles at torso level)
  lidar_scan:
    topic: /scan
    sensor_frame: chest_lidar_link
    observation_persistence: 0.0
    expected_update_rate: 10.0
    data_type: LaserScan
    clearing: true
    marking: true
    min_obstacle_height: 0.3  # Ignore ground (robot can step over low obstacles)
    max_obstacle_height: 2.0  # Detect up to humanoid head height

  # Stereo depth for 3D obstacles (stairs, overhead clearance)
  stereo_point_cloud:
    topic: /stereo/points2
    sensor_frame: head_camera_link
    observation_persistence: 0.0
    expected_update_rate: 5.0
    data_type: PointCloud2
    clearing: true
    marking: true
    min_obstacle_height: 0.1  # Detect low obstacles (cables, curbs)
    max_obstacle_height: 2.2  # Ceiling/doorframe clearance

  # Voxel grid for 3D obstacles
  voxel_layer:
    enabled: true
    publish_voxel_map: true
    origin_z: 0.0
    z_resolution: 0.2  # 20cm vertical resolution
    z_voxels: 12  # 2.4m max height (12 * 0.2m)
    mark_threshold: 2  # Mark as obstacle if 2+ voxels occupied
    unknown_threshold: 10
    combination_method: 1  # Max operator

  # Inflation layer (expand obstacles by robot size)
  inflation_layer:
    enabled: true
    cost_scaling_factor: 3.0  # Exponential decay rate
    inflation_radius: 0.6  # Robot radius + safety margin (meters)
```

**Visualization in RViz**:
```bash
# Launch Nav2 with humanoid costmap config
ros2 launch nav2_bringup bringup_launch.py \
  params_file:=/path/to/humanoid_nav2_params.yaml

# In RViz, add:
# - Map display (topic: /global_costmap/costmap)
# - PointCloud2 (topic: /stereo/points2)
# - LaserScan (topic: /scan)
# Costmap should show inflated obstacles with 3D awareness
```

### Subsection 2.3: Handling Stairs and Slopes

Humanoids can climb stairs, but Nav2's 2D costmap treats them as obstacles. Solution: **stair detection and traversability layer**.

**Custom Costmap Plugin Concept**:
```cpp
// Pseudo-code for stair-aware costmap layer
class StairLayer : public nav2_costmap_2d::Layer {
  void updateCosts(costmap_2d::Costmap2D& master_grid, ...) override {
    // 1. Detect stair regions from point cloud (plane fitting)
    // 2. Check if stair geometry matches robot capabilities:
    //    - Step height < max_step_height (e.g., 25cm)
    //    - Step depth > min_foot_length (e.g., 20cm)
    // 3. Mark traversable stairs as FREE (cost 0)
    // 4. Mark non-traversable stairs as LETHAL (cost 254)
  }
};
```

**Simplified Approach for Beginners**:
- Pre-mark stair regions in static map as "traversable zones"
- Use behavior tree to trigger "climb stairs" action when entering zone
- Disable costmap obstacle detection in stair zones

---

## Section 3: Path Planning for Bipedal Locomotion

### Subsection 3.1: Global Planners: Smac vs. NavFn

**NavFn (Traditional A\*)**:
- Assumes circular robot (holonomic or differential drive)
- Plans smooth paths ignoring kinematic constraints
- **Problem for humanoids**: Generates tight turns requiring in-place rotation

**Smac Planner (State Lattice)**:
- Pre-computes feasible motion primitives (e.g., "step forward-left", "step right")
- Plans over discrete states (x, y, theta)
- **Better for humanoids**: Respects turning radius, generates footstep-like paths

**Configuration**:
```yaml
# File: humanoid_nav2_params.yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"  # Hybrid A* (better for humanoids)

      # Motion model parameters
      motion_model_for_search: "REEDS_SHEPP"  # Car-like model (no in-place rotation)
      minimum_turning_radius: 0.5  # Meters (humanoid needs ~2 steps to turn 90°)

      # Cost penalties
      change_penalty: 0.05  # Penalize direction changes (prefer straight paths)
      non_straight_penalty: 1.2  # Further penalize curves
      cost_travel_multiplier: 2.0

      # Search parameters
      max_iterations: 1000000
      max_planning_time: 5.0  # Seconds (allow time for complex plans)

      # Footprint (humanoid shoulder width)
      footprint: "[[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]]"
```

### Subsection 3.2: Local Controllers: DWB with Bipedal Constraints

**DWB (Dynamic Window Approach)** samples velocity trajectories and scores them.

**Modifications for Humanoids**:
- **Discrete velocity samples**: Match walking gait speeds (0.2, 0.4, 0.6 m/s)
- **Penalize lateral motion**: Humanoids walk forward, not sideways
- **Limit angular velocity**: Slow rotation (e.g., 0.5 rad/s max)

**Configuration**:
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0  # Lower than wheeled robots (humanoid controller runs at 100Hz separately)
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      # Velocity limits (match humanoid walking capabilities)
      min_vel_x: 0.0
      max_vel_x: 0.6  # Max forward walking speed (m/s)
      min_vel_y: 0.0  # No lateral motion
      max_vel_y: 0.0
      max_vel_theta: 0.5  # Slow turning (rad/s)
      min_speed_xy: 0.1  # Minimum speed to maintain balance
      max_speed_xy: 0.6
      acc_lim_x: 0.3  # Slow acceleration (avoid destabilizing robot)
      acc_lim_theta: 0.5
      decel_lim_x: -0.5
      decel_lim_theta: -1.0

      # Trajectory simulation
      vx_samples: 5  # Sample fewer velocities (faster planning)
      vy_samples: 1  # No Y motion
      vtheta_samples: 10
      sim_time: 2.0  # Simulate 2s ahead (longer for slower humanoid reactions)

      # Scoring weights
      path_distance_bias: 32.0  # Stay close to global path
      goal_distance_bias: 20.0  # Prefer trajectories toward goal
      occdist_scale: 0.02  # Penalize trajectories near obstacles
      forward_point_distance: 0.3  # Lookahead distance for path tracking

      # Critics (trajectory scoring functions)
      critics: [
        "RotateToGoal",
        "Oscillation",
        "BaseObstacle",
        "GoalAlign",
        "PathAlign",
        "PathDist",
        "GoalDist"
      ]
```

### Subsection 3.3: Footstep Planning (Advanced)

For precise bipedal control, replace DWB with **footstep planner**:

**Concept**:
1. Global planner generates waypoints
2. Footstep planner computes foot placements (left, right, left, ...) to reach each waypoint
3. Whole-body controller executes footstep sequence

**ROS 2 Package**: `humanoid_navigation` (community package)

**Example Footstep Plan**:
```
Goal: Move forward 2m, turn right 90°
Footsteps:
  1. Left foot: (0.4, 0.1, 0°)
  2. Right foot: (0.8, -0.1, 0°)
  3. Left foot: (1.2, 0.1, 0°)
  4. Right foot: (1.6, -0.1, 15°)  # Start turning
  5. Left foot: (2.0, 0.2, 45°)
  6. Right foot: (2.2, 0.0, 90°)    # Final orientation
```

**Integration with Nav2**:
- Use Smac Planner for global waypoints
- Replace DWB with footstep controller that tracks waypoints
- Controller publishes `geometry_msgs/PoseStamped` for each foot to whole-body controller

---

## Section 4: Integrating Isaac ROS Perception with Nav2

### Subsection 4.1: cuVSLAM as Localization Source

**Replace AMCL with cuVSLAM**:

AMCL (Adaptive Monte Carlo Localization) uses particle filters with laser scans. For humanoids with cameras, use cuVSLAM instead.

**Launch File**:
```python
# File: humanoid_nav2_isaac.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Nav2 bringup (without AMCL)
    nav2_params = RewrittenYaml(
        source_file='humanoid_nav2_params.yaml',
        root_key='',
        param_rewrites='',
        convert_types=True
    )

    return LaunchDescription([
        # cuVSLAM for localization (replaces AMCL)
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_imu_fusion': True,
                'enable_localization_n_mapping': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }],
            remappings=[
                ('/camera/image_raw', '/humanoid/camera/left/image_raw'),
                ('/camera/camera_info', '/humanoid/camera/left/camera_info'),
                ('/imu/data', '/humanoid/imu/data')
            ]
        ),

        # Nav2 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{'autostart': True, 'node_names': ['planner_server', 'controller_server', 'bt_navigator']}]
        ),

        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params]
        ),

        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params]
        ),

        # Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params]
        )
    ])
```

**Key Configuration**:
```yaml
# In humanoid_nav2_params.yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /visual_slam/tracking/odometry  # Use cuVSLAM odometry instead of wheel encoders
    transform_tolerance: 0.5  # Allow 500ms TF latency (visual SLAM can lag slightly)
```

### Subsection 4.2: Stereo Depth for Obstacle Avoidance

**Publish Point Cloud to Costmap**:

```yaml
# In humanoid_costmap_common.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # ... (other params)

      observation_sources: stereo_points

      stereo_points:
        topic: /stereo/points2
        sensor_frame: head_camera_optical_frame
        data_type: PointCloud2
        clearing: true
        marking: true
        min_obstacle_height: 0.2  # Ignore ground plane
        max_obstacle_height: 2.0  # Detect obstacles up to head height
        obstacle_range: 5.0  # Only consider obstacles within 5m
        raytrace_range: 6.0  # Clear space up to 6m
```

**Filter Ground Plane** (critical for point clouds):
```bash
# Use PCL ground plane removal
ros2 run pcl_ros_filter_ground_plane_node \
  --ros-args \
  --remap /input:=/stereo/points2 \
  --remap /output:=/stereo/points2_filtered \
  --param distance_threshold:=0.05 \
  --param max_iterations:=100
```

### Subsection 4.3: Complete Navigation Launch File

```python
# File: humanoid_full_navigation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # 1. Isaac Sim (run separately)
        # Assumed to be publishing:
        # - /humanoid/camera/left/image_raw
        # - /humanoid/camera/right/image_raw
        # - /humanoid/imu/data

        # 2. Isaac ROS cuVSLAM
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_imu_fusion': True,
                'map_frame': 'map',
                'odom_frame': 'odom'
            }]
        ),

        # 3. Isaac ROS Stereo Depth
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('isaac_ros_stereo_image_proc'),
                             'launch', 'isaac_ros_stereo_image_pipeline.launch.py')
            )
        ),

        # 4. Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': '/path/to/humanoid_nav2_params.yaml',
                'autostart': 'true'
            }.items()
        ),

        # 5. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')]
        )
    ])
```

---

## Section 5: Behavior Trees for Humanoid Navigation

### Subsection 5.1: What are Behavior Trees?

**Behavior Trees (BT)** are a control architecture for AI and robotics:
- **Nodes**: Conditions, actions, control flow (sequence, fallback, parallel)
- **Execution**: Tick nodes from root, propagate SUCCESS/FAILURE/RUNNING
- **Modularity**: Compose complex behaviors from simple building blocks

**Nav2 Behavior Tree**:
- Root: Navigate to pose
- Children: Compute path → Follow path → Recovery behaviors

### Subsection 5.2: Custom Humanoid Behaviors

**Example: "Check Balance Before Moving"**

```xml
<!-- File: humanoid_navigate_to_pose.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="NavigateWithBalanceCheck">
      <!-- 1. Check if robot is balanced -->
      <Condition ID="IsBalanced" topic="/robot/is_balanced" expected="true"/>

      <!-- 2. Compute global path -->
      <ComputePathToPose goal="{goal}" path="{path}"/>

      <!-- 3. Follow path with balance monitoring -->
      <RecoveryNode number_of_retries="3" name="FollowPathWithRecovery">
        <Sequence>
          <Condition ID="IsBalanced"/>  <!-- Check balance each iteration -->
          <FollowPath path="{path}"/>
        </Sequence>

        <!-- If balance lost, execute recovery -->
        <Fallback>
          <Sequence>
            <Action ID="StopMotion"/>  <!-- Freeze all joints -->
            <Action ID="RegainBalance"/>  <!-- Shift COM, adjust stance -->
            <Wait wait_duration="1"/>
          </Sequence>
          <Action ID="Abort"/>  <!-- If recovery fails, abort navigation -->
        </Fallback>
      </RecoveryNode>

      <!-- 4. Success -->
      <Action ID="SuccessAction"/>
    </Sequence>
  </BehaviorTree>
</root>
```

**Load Custom BT**:
```yaml
# In humanoid_nav2_params.yaml
bt_navigator:
  ros__parameters:
    default_bt_xml_filename: "/path/to/humanoid_navigate_to_pose.xml"
```

---

## Hands-On Project: Autonomous Warehouse Navigation

**Goal**: Navigate a humanoid through a simulated warehouse using Isaac Sim, cuVSLAM, stereo depth, and Nav2.

**Duration**: 60 minutes

**What You'll Learn**:
- Set up complete perception + navigation pipeline
- Tune costmap for dynamic obstacles
- Send navigation goals and monitor execution
- Handle navigation failures with recoveries

### Step 1: Prepare Isaac Sim Scene

(From Chapter 3.1, create warehouse with humanoid + stereo camera)

### Step 2: Install and Configure Nav2

```bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Create workspace
mkdir -p ~/humanoid_nav_ws/src
cd ~/humanoid_nav_ws/src

# Clone Isaac ROS (if not already)
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# Build
cd ~/humanoid_nav_ws
colcon build --symlink-install
source install/setup.bash
```

### Step 3: Create Nav2 Configuration

Save the following as `humanoid_nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /visual_slam/tracking/odometry
    default_bt_xml_filename: "navigate_to_pose_w_replanning_and_recovery.xml"

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"

controller_server:
  ros__parameters:
    controller_frequency: 10.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5
      max_vel_theta: 0.5
      acc_lim_x: 0.3
      acc_lim_theta: 0.5

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.35
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.35
      resolution: 0.05
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.6
```

### Step 4: Launch Navigation Stack

```bash
# Terminal 1: Launch Isaac Sim (from Chapter 3.1)

# Terminal 2: Launch perception + navigation
ros2 launch humanoid_full_navigation.launch.py

# Terminal 3: Send navigation goal
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Step 5: Visualize in RViz

In RViz, you should see:
- **Global costmap**: Static warehouse obstacles
- **Local costmap**: Dynamic obstacles from stereo depth
- **Global path**: Green line from planner
- **Local trajectory**: DWB controller trajectory
- **Robot model**: Humanoid moving along path

**Monitor Progress**:
```bash
# Check navigation status
ros2 topic echo /bt_navigator/status

# Check if goal reached
ros2 topic echo /bt_navigator/result
```

---

## Challenge: Test Your Understanding

Try these exercises to reinforce your learning:

1. **Basic**: Add a static obstacle to the costmap by modifying the map YAML file, then navigate around it
   - *Hint*: Edit `map.yaml` to add occupied cells, reload with map_server

2. **Intermediate**: Implement a custom recovery behavior that backs up the humanoid if stuck
   - *Hint*: Create ROS 2 action server that publishes negative velocity for 2 seconds

3. **Advanced**: Integrate footstep planner from `humanoid_navigation` package to replace DWB controller
   - *Hint*: Create custom controller plugin that calls footstep planner and publishes foot targets

---

## Summary

In this chapter, you learned:

- **Nav2 architecture**: Planner, controller, recoveries, behavior trees for orchestrating navigation
- **Humanoid costmap configuration**: 3D voxel layers, sensor fusion (stereo depth + LiDAR), stair handling
- **Bipedal path planning**: Smac Planner for kinematic constraints, DWB tuning for stable walking
- **Isaac ROS integration**: cuVSLAM for localization, stereo depth for obstacle avoidance, complete pipeline

**Key Commands**:

```bash
# Launch Nav2 with custom config
ros2 launch nav2_bringup bringup_launch.py params_file:=humanoid_nav2_params.yaml

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 2.0}}}}"

# Monitor costmap topics
ros2 topic echo /local_costmap/costmap_updates
ros2 topic echo /global_costmap/costmap

# Check TF tree (ensure map→odom→base_link connected)
ros2 run tf2_tools view_frames
```

**Core Concepts**:
- **Costmap Layers**: Static, obstacle, voxel, inflation (combined to create navigation cost field)
- **Smac Planner**: Lattice-based planning with kinematic constraints (no holonomic motion)
- **DWB Controller**: Dynamic window approach with velocity sampling (adapted for bipedal speeds)
- **Behavior Trees**: Modular navigation logic with recoveries for robust autonomy

---

## Further Reading

Official Documentation:
- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)
- [Smac Planner Details](https://navigation.ros.org/configuration/packages/configuring-smac-planner.html)

Tutorials and Examples:
- [Nav2 Tutorials (First-Time Robot Setup)](https://navigation.ros.org/setup_guides/index.html)
- [Behavior Trees in Nav2](https://navigation.ros.org/behavior_trees/index.html)
- [Costmap Configuration Guide](https://navigation.ros.org/setup_guides/footprint/setup_footprint.html)

Research Papers:
- "Humanoid Navigation System in Cluttered Environments" (Stasse et al., 2018) - Footstep planning with MPC
- "A Survey of Bipedal Locomotion Control" (Kuo et al., 2020) - Stability constraints for walking robots
- "DWA for Mobile Robot Navigation" (Fox et al., 1997) - Original dynamic window approach

---

## Next Steps

Congratulations! You've completed Module 3: NVIDIA Isaac Platform.

**Next Module**: Module 4: Vision-Language-Action Systems (Coming Soon)

In Module 4, you'll learn how to integrate voice commands, natural language task planning with LLMs, and vision-language-action models to create truly intelligent humanoid robots that understand and execute complex commands.

**Optional Practice**:
- Implement a "follow person" behavior using YOLO object detection + Nav2 goal updates
- Tune DWB parameters for different walking speeds (slow/normal/fast modes)
- Create a map of your lab/home using SLAM Toolbox + Isaac Sim simulation
- Join the Nav2 community on ROS Discourse for advanced navigation discussions
