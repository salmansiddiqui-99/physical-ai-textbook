---
id: ch1-isaac-sim
title: "Isaac Sim Basics: Photorealistic Robot Simulation"
sidebar_label: "Isaac Sim Basics"
sidebar_position: 7
description: Learn NVIDIA Isaac Sim fundamentals including photorealistic rendering, USD scene workflows, synthetic data generation for training perception models, and ROS 2 integration for humanoid robotics.
keywords:
  - Isaac Sim
  - NVIDIA Omniverse
  - USD
  - photorealistic simulation
  - synthetic data
  - RTX rendering
  - robot simulation
  - ROS 2 Isaac
prerequisites:
  - Completion of Module 1 (ROS 2 Foundations)
  - Completion of Module 2 (Simulation Environments)
  - NVIDIA GPU with RTX support (recommended)
  - Basic understanding of 3D graphics concepts
learning_objectives:
  - Explain Isaac Sim architecture and its advantages over traditional simulators
  - Create and configure USD scenes with humanoid robots and environments
  - Implement synthetic data generation pipelines for computer vision tasks
  - Integrate Isaac Sim with ROS 2 for realistic humanoid control workflows
estimated_time: 90 minutes
---

# Isaac Sim Basics: Photorealistic Robot Simulation

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain Isaac Sim architecture, USD workflows, and photorealistic rendering capabilities
- Create USD scenes with humanoid robots, sensors, and realistic environments
- Implement synthetic data generation for training perception models
- Integrate Isaac Sim with ROS 2 for realistic sensor simulation and control

## Prerequisites

Before starting this chapter, you should:

- Have completed [Module 1: ROS 2 Foundations](../module1-ros2/ch1-ros2-basics.md) and [Module 2: Simulation](../module2-simulation/ch1-gazebo-essentials.md)
- Understand basic 3D graphics concepts (meshes, materials, lighting)
- Have access to an NVIDIA GPU with RTX support (RTX 2060 or higher recommended)
- Have installed Isaac Sim 2023.1.1 or later (see installation guide below)

## Introduction

NVIDIA Isaac Sim represents a paradigm shift in robot simulation. Unlike traditional simulators that prioritize speed over realism, Isaac Sim leverages RTX ray tracing and physically-based rendering to create photorealistic environments indistinguishable from real-world camera feeds. This realism is critical for humanoid robotics, where perception models trained on synthetic data must transfer seamlessly to physical robots.

Isaac Sim is built on NVIDIA Omniverse, a platform for real-time 3D collaboration and simulation using Universal Scene Description (USD). USD, originally developed by Pixar, provides a powerful framework for composing, editing, and rendering complex 3D scenes. By adopting USD as its scene format, Isaac Sim enables artists, engineers, and roboticists to collaborate using industry-standard tools like Blender, Maya, and Unreal Engine.

For humanoid robots operating in human environments, Isaac Sim offers unparalleled advantages. You can generate millions of labeled images showing humanoids grasping objects, navigating cluttered spaces, or interacting with people—all with perfect ground truth for object poses, depth maps, and semantic segmentation. This chapter will guide you through setting up Isaac Sim, creating your first USD scene, and generating synthetic data for training vision-based humanoid controllers.

---

## Section 1: Isaac Sim Architecture and USD Fundamentals

### Subsection 1.1: What is Isaac Sim?

Isaac Sim is NVIDIA's robotics simulation platform built on Omniverse. It combines:

- **PhysX 5.0**: High-fidelity physics engine with GPU acceleration for contacts, rigid bodies, and articulations
- **RTX Rendering**: Ray-traced lighting, shadows, and reflections for photorealistic visuals
- **USD Scene Format**: Industry-standard 3D scene description enabling interoperability
- **Isaac SDK/ROS Integration**: Native ROS 2 bridge for seamless robot control
- **Synthetic Data Generation**: Automated labeling for computer vision (bounding boxes, segmentation, depth)

**Key Differences from Gazebo**:

| Feature | Gazebo Classic/Fortress | Isaac Sim |
|---------|-------------------------|-----------|
| Rendering | OpenGL (approximate lighting) | RTX ray tracing (photorealistic) |
| Physics | ODE/Bullet/DART | PhysX 5.0 (GPU-accelerated) |
| Scene Format | SDF/URDF (custom) | USD (industry standard) |
| Synthetic Data | Manual scripting required | Built-in replicator API |
| Performance | CPU-bound | GPU-accelerated (10-100x faster) |

### Subsection 1.2: Universal Scene Description (USD)

USD is a file format and runtime API for describing 3D scenes. Think of it as "Photoshop layers" for 3D content:

- **Layers**: Compose scenes from reusable components (robot, environment, sensors)
- **Prims**: Scene graph nodes representing objects (meshes, cameras, lights)
- **Attributes**: Properties like position, color, or physics parameters
- **Variants**: Switchable configurations (e.g., day/night lighting, different robot payloads)

**USD File Types**:
- `.usd`: Binary format (fast loading, production use)
- `.usda`: ASCII format (human-readable, version control friendly)
- `.usdc`: Compact binary format (smallest file size)

**Example USD Hierarchy**:
```
World.usd
├── /Environment
│   ├── /Warehouse (mesh, materials)
│   ├── /Lighting (HDR dome light)
│   └── /Ground (physics collision plane)
├── /Humanoid
│   ├── /robot_base (articulation root)
│   ├── /Sensors
│   │   ├── /head_camera
│   │   └── /chest_lidar
│   └── /Controllers (ROS 2 action graph)
└── /ReplicatorGraph (synthetic data config)
```

### Subsection 1.3: Isaac Sim Workflow Overview

**Typical Development Cycle**:

1. **Scene Assembly**: Import robot URDF, add environments, configure sensors
2. **Physics Tuning**: Set friction, damping, contact parameters
3. **Sensor Configuration**: Attach cameras, LiDAR, IMU with realistic noise
4. **ROS 2 Integration**: Create action graph for pub/sub communication
5. **Synthetic Data Setup**: Define replicator for data generation
6. **Execution**: Run simulation, collect data or test control algorithms

---

## Section 2: Installing and Launching Isaac Sim

### Subsection 2.1: System Requirements

**Minimum Specifications**:
- **OS**: Ubuntu 20.04/22.04 or Windows 10/11
- **GPU**: NVIDIA RTX 2060 or higher (6GB VRAM minimum)
- **CPU**: Intel i7 or AMD Ryzen 7 (8 cores recommended)
- **RAM**: 32GB (64GB recommended for large scenes)
- **Storage**: 50GB free space (SSD recommended)
- **Drivers**: NVIDIA GPU Driver 525.60.11 or later

**Recommended for Humanoid Workflows**:
- **GPU**: RTX 3090 / RTX 4080 / A6000 (24GB VRAM for complex scenes)
- **RAM**: 64GB+ (enables multiple humanoids in scene)
- **Storage**: NVMe SSD (faster scene loading and data writing)

### Subsection 2.2: Installation via Omniverse Launcher

Isaac Sim is distributed through the NVIDIA Omniverse Launcher.

**Step 1: Download Omniverse Launcher**:
```bash
# Visit https://www.nvidia.com/en-us/omniverse/download/
# Download and install Omniverse Launcher for your OS

# On Linux, run the AppImage:
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

**Step 2: Install Isaac Sim**:
1. Open Omniverse Launcher
2. Navigate to "Exchange" tab
3. Search for "Isaac Sim"
4. Click "Install" (downloads ~15GB)
5. Choose installation path (default: `~/.local/share/ov/pkg/isaac_sim-2023.1.1`)

**Step 3: Verify Installation**:
```bash
# Launch Isaac Sim from Launcher or via command line
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./isaac-sim.sh
```

You should see the Isaac Sim GUI with a welcome screen.

### Subsection 2.3: Python Standalone Workflows

For headless data generation or ROS 2 integration, use the Python API:

**Example: Running Isaac Sim Programmatically**:

```python
#!/usr/bin/env python3
"""
Minimal Isaac Sim Python example

Purpose: Launch Isaac Sim, create a simple scene, run simulation loop
Environment: Isaac Sim 2023.1.1+, Python 3.10
"""

from isaacsim import SimulationApp

# Create simulation app instance (headless=True for servers)
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create physics world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add a cube that will fall
cube = DynamicCuboid(
    prim_path="/World/Cube",
    position=[0, 0, 2.0],  # 2 meters above ground
    size=0.5,  # 50cm cube
    color=[0.8, 0.2, 0.1]  # Orange
)
world.scene.add(cube)

# Reset simulation
world.reset()

# Run simulation for 500 steps
for i in range(500):
    world.step(render=True)  # Step physics and render
    if i % 50 == 0:
        position, _ = cube.get_world_pose()
        print(f"Step {i}: Cube at height {position[2]:.2f}m")

# Cleanup
simulation_app.close()
```

**Running the Script**:
```bash
# Must use Isaac Sim's Python interpreter
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh minimal_example.py
```

**Expected Output**:
```
Step 0: Cube at height 2.00m
Step 50: Cube at height 1.52m
Step 100: Cube at height 0.87m
Step 150: Cube at height 0.31m
Step 200: Cube at height 0.25m  # Settled on ground
```

---

## Section 3: Creating Your First USD Scene

### Subsection 3.1: Scene Composition Basics

USD scenes are built through **composition**: layering multiple USD files to create complex environments.

**Composition Arcs** (in order of strength):
1. **Sublayers**: Merge layers (e.g., base scene + lighting overrides)
2. **References**: Reuse assets (e.g., include robot USD from library)
3. **Payloads**: Lazy-load heavy assets (e.g., high-poly environment meshes)
4. **Variants**: Switchable options (e.g., day/night lighting)

**Best Practices**:
- **One robot = one USD file**: Package robot, sensors, controllers together
- **Environment assets as references**: Reuse warehouse, office, outdoor scenes
- **Sensors as prims**: Cameras and LiDAR as child prims of robot links
- **Use variants for configurations**: Switch between simulation/real-world sensor params

### Subsection 3.2: Importing a URDF Humanoid

Isaac Sim can import URDF files and convert them to USD format with physics.

**Example: Importing Humanoid URDF**:

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.urdf import _urdf
import omni.isaac.core.utils.prims as prims_utils

# Import URDF (Isaac Sim converts to USD automatically)
urdf_path = "/path/to/humanoid.urdf"
robot_prim_path = "/World/Humanoid"

# Import with physics enabled
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False  # Keep kinematic structure
import_config.convex_decomp = True  # Use convex hulls for collision
import_config.import_inertia_tensor = True
import_config.default_drive_strength = 100000  # Joint motor strength
import_config.default_position_drive_damping = 10000

success, robot_prim = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    dest_path=robot_prim_path
)

if success:
    print(f"Robot imported at {robot_prim_path}")
else:
    print("URDF import failed!")
```

**What Happens During Import**:
- URDF `<link>` → USD `Xformable` prims with meshes
- URDF `<joint>` → PhysX `ArticulationJoint` (revolute/prismatic)
- URDF `<inertial>` → PhysX mass and inertia properties
- URDF `<collision>` → PhysX collision shapes (convex hulls if enabled)
- URDF `<visual>` → Mesh geometry with materials

### Subsection 3.3: Configuring Physics Parameters

After importing, fine-tune physics for realistic behavior.

**Key Parameters for Humanoids**:

```python
import omni.physx.scripts.utils as physx_utils

# Set global simulation parameters
physx_scene = world.get_physics_context()
physx_scene.set_gravity(-9.81)  # Earth gravity
physx_scene.set_solver_type("TGS")  # Temporal Gauss-Seidel (stable)

# Configure articulation (whole robot)
from pxr import PhysxSchema
articulation_prim = world.stage.GetPrimAtPath("/World/Humanoid")
physx_articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(articulation_prim)
physx_articulation_api.CreateSolverPositionIterationCountAttr(32)  # More iterations = more stable
physx_articulation_api.CreateSolverVelocityIterationCountAttr(16)

# Set ground contact friction
ground_prim = world.stage.GetPrimAtPath("/World/groundPlane")
physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(ground_prim)
physx_material = PhysxSchema.PhysxMaterialAPI.Apply(ground_prim)
physx_material.CreateStaticFrictionAttr(0.8)  # Prevents slipping
physx_material.CreateDynamicFrictionAttr(0.6)
physx_material.CreateRestitutionAttr(0.01)  # Low bounce
```

**Troubleshooting Physics**:

| Issue | Cause | Solution |
|-------|-------|----------|
| Robot explodes on contact | Iteration count too low | Increase `solverPositionIterationCount` to 64+ |
| Feet slip during walking | Friction too low | Increase `staticFriction` to 1.0-1.5 |
| Simulation too slow | Too many collision checks | Use convex decomposition, reduce mesh complexity |
| Joints drift over time | Damping too low | Increase `positionDriveDamping` in joints |

---

## Section 4: Photorealistic Rendering and Cameras

### Subsection 4.1: RTX Ray Tracing Basics

Isaac Sim uses NVIDIA RTX for real-time ray tracing.

**Rendering Modes**:
- **RTX - Real-Time**: Interactive ray tracing (good for development)
- **RTX - Interactive (Path Tracing)**: Higher quality, slower (1-2 FPS for complex scenes)
- **RTX - Accurate (Path Tracing)**: Production quality (offline rendering)

**Enabling RTX in Python**:
```python
import omni.kit.viewport.utility as viewport_utils

# Get active viewport
viewport = viewport_utils.get_active_viewport()

# Set RTX real-time mode
viewport_utils.get_viewport_widget(viewport).set_render_mode("rtx_realtime")
```

### Subsection 4.2: Adding Cameras for Perception

Cameras are the primary perception sensors for humanoid robots.

**Example: Attach RGB Camera to Robot Head**:

```python
from omni.isaac.core.utils import stage
from omni.isaac.sensor import Camera
import numpy as np

# Create camera at robot head position
camera = Camera(
    prim_path="/World/Humanoid/head/camera",
    position=np.array([0.05, 0, 0.15]),  # 5cm forward, 15cm up from head link
    frequency=30,  # 30 Hz (standard video framerate)
    resolution=(1280, 720),  # HD resolution
    orientation=np.array([1, 0, 0, 0])  # Quaternion (identity = looking forward)
)

# Initialize camera
camera.initialize()

# Set camera properties
camera.set_focal_length(24.0)  # Focal length in mm (wide angle)
camera.set_focus_distance(400.0)  # Focus at 4 meters
camera.set_f_stop(8.0)  # Aperture (higher = more in focus)
camera.set_clipping_range(0.01, 10000.0)  # Near/far clip planes
```

**Capturing Images**:
```python
# After world.step(), capture camera data
camera.initialize()
camera_data = camera.get_current_frame()

rgb_image = camera_data["rgba"]  # Shape: (720, 1280, 4), dtype: uint8
depth_map = camera_data["depth"]  # Shape: (720, 1280), dtype: float32

print(f"Captured {rgb_image.shape[0]}x{rgb_image.shape[1]} RGB image")
print(f"Depth range: {depth_map.min():.2f}m to {depth_map.max():.2f}m")
```

### Subsection 4.3: Semantic and Instance Segmentation

Isaac Sim can render perfect ground truth for object detection.

**Enable Segmentation Render Product**:
```python
from omni.isaac.core.utils import render_product

# Create render product for segmentation
rp = render_product.create_render_product(
    camera.prim_path,
    resolution=(1280, 720)
)

# Annotate with semantic segmentation
from omni.syntheticdata import sensors
sem_sensor = sensors.enable_sensors(
    render_product=rp,
    sensor_types=["semantic_segmentation", "instance_segmentation"]
)
```

**Accessing Segmentation Masks**:
```python
# After world.step()
semantic_data = sem_sensor.get_semantic_data()
instance_data = sem_sensor.get_instance_data()

# semantic_data: (720, 1280) with class IDs per pixel
# instance_data: (720, 1280) with unique instance IDs per object
```

**Semantic Class Mapping**:
Isaac Sim uses a class mapping to label objects:
- Class 0: Unlabeled/background
- Class 1-N: Custom labels (e.g., "person", "chair", "door")

---

## Section 5: ROS 2 Integration

### Subsection 5.1: ROS 2 Bridge Architecture

Isaac Sim includes a built-in ROS 2 bridge using **OmniGraph** (visual scripting for robotics).

**Key Components**:
- **ROS 2 Context**: Initializes ROS 2 node within Isaac Sim
- **ROS 2 Publishers**: Send sensor data, joint states, TF transforms
- **ROS 2 Subscribers**: Receive velocity commands, joint commands
- **ROS 2 Clock**: Synchronize ROS 2 time with simulation time

### Subsection 5.2: Creating ROS 2 Action Graph

**Example: Publish Camera Images to ROS 2**:

```python
import omni.graph.core as og

# Create action graph
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ROS2Context.inputs:domain_id", 0),
            ("CameraHelper.inputs:topicName", "/humanoid/head_camera/image_raw"),
            ("CameraHelper.inputs:frameId", "camera_link"),
            ("CameraHelper.inputs:type", "rgb"),
            ("CameraHelper.inputs:renderProductPath", rp),
        ],
    },
)
```

**Verifying ROS 2 Topics**:
```bash
# In separate terminal (source ROS 2 workspace first)
source /opt/ros/humble/setup.bash

# List topics published by Isaac Sim
ros2 topic list

# Expected output:
# /humanoid/head_camera/image_raw
# /humanoid/head_camera/camera_info

# View images in RViz2
rviz2
# Add Image display, set topic to /humanoid/head_camera/image_raw
```

### Subsection 5.3: Subscribing to Joint Commands

**Example: Control Robot Joints via ROS 2**:

```python
# Add articulation controller to action graph
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ActionGraph"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
            ("ROS2SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ROS2SubscribeJointState.inputs:execIn"),
            ("ROS2SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("ROS2SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ROS2SubscribeJointState.inputs:topicName", "/joint_command"),
            ("ArticulationController.inputs:robotPath", "/World/Humanoid"),
        ],
    },
)
```

**Publishing Joint Commands from ROS 2**:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_command', 10)
        self.timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        msg = JointState()
        msg.name = ['left_shoulder_pitch', 'right_shoulder_pitch']
        msg.position = [0.5, -0.5]  # Raise left arm, lower right arm
        self.publisher.publish(msg)

rclpy.init()
node = JointCommandPublisher()
rclpy.spin(node)
```

---

## Section 6: Synthetic Data Generation

### Subsection 6.1: Replicator API Overview

**Omniverse Replicator** is Isaac Sim's tool for generating synthetic training data at scale.

**Key Features**:
- **Randomization**: Procedurally vary lighting, textures, object poses
- **Ground Truth Labels**: Automatic bounding boxes, segmentation, keypoints
- **High Throughput**: Generate 10,000+ labeled images per hour on RTX 3090
- **Domain Randomization**: Reduce sim-to-real gap via visual diversity

### Subsection 6.2: Creating a Replicator Script

**Example: Generate 1000 Images of Humanoid Grasping Objects**:

```python
import omni.replicator.core as rep

# Define randomization for camera position
with rep.new_layer():
    # Camera orbiting around humanoid
    camera = rep.create.camera(position=(2, 0, 1.5), look_at="/World/Humanoid/torso")

    # Randomize camera position in spherical coordinates
    with camera:
        rep.modify.pose(
            position=rep.distribution.sphere(
                center=(0, 0, 1.2),  # Center on humanoid torso
                radius=rep.distribution.uniform(1.5, 3.0)  # 1.5-3m distance
            ),
            look_at="/World/Humanoid/torso"
        )

    # Randomize lighting
    light = rep.create.light(
        light_type="Dome",
        rotation=(rep.distribution.uniform(0, 360), 0, 0)
    )

    # Randomize object to grasp (place in humanoid's hand)
    objects = ["/World/Objects/Mug", "/World/Objects/Bottle", "/World/Objects/Box"]
    with rep.create.group(objects):
        rep.randomizer.scatter_2d(
            surface_prims="/World/Humanoid/right_hand",
            check_for_collisions=True
        )

    # Render and capture
    render_product = rep.create.render_product(camera, resolution=(1280, 720))

    # Attach writers for data output
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="/tmp/humanoid_grasp_dataset",
        rgb=True,
        semantic_segmentation=True,
        bounding_box_2d_tight=True,
        distance_to_camera=True
    )
    writer.attach([render_product])

# Run replicator for 1000 frames
with rep.orchestrator.step(num_steps=1000):
    rep.randomizer.randomize()
```

**Running Data Generation**:
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh generate_dataset.py
```

**Output Structure**:
```
/tmp/humanoid_grasp_dataset/
├── rgb_0000.png
├── rgb_0001.png
├── ...
├── semantic_segmentation_0000.png
├── bounding_box_2d_tight_0000.json
├── distance_to_camera_0000.npy
└── metadata.json
```

---

## Hands-On Project: Humanoid in Warehouse Scene

**Goal**: Create a photorealistic warehouse scene with a humanoid robot, attach sensors, and publish data to ROS 2.

**Duration**: 45 minutes

**What You'll Learn**:
- Import and assemble USD assets into a scene
- Configure humanoid physics and sensors
- Integrate ROS 2 for real-time teleoperation
- Generate synthetic perception data

### Step 1: Create Warehouse Environment

```python
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Initialize world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add warehouse asset (from NVIDIA assets library)
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Environments/Industrial/warehouse.usd",
    prim_path="/World/Warehouse"
)

# Add lighting
import omni.isaac.core.utils.prims as prims_utils
prims_utils.create_prim(
    prim_path="/World/DomeLight",
    prim_type="DomeLight",
    attributes={"intensity": 1000.0, "color": (1.0, 0.95, 0.9)}
)

print("Warehouse scene created")
```

### Step 2: Import Humanoid and Attach Sensors

```python
from omni.isaac.urdf import _urdf

# Import humanoid URDF
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.convex_decomp = True
import_config.default_drive_strength = 100000

omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="/path/to/humanoid.urdf",
    import_config=import_config,
    dest_path="/World/Humanoid"
)

# Attach camera to head
from omni.isaac.sensor import Camera
import numpy as np

head_camera = Camera(
    prim_path="/World/Humanoid/head/camera",
    position=np.array([0.05, 0, 0.15]),
    frequency=30,
    resolution=(1280, 720)
)
head_camera.initialize()

print("Humanoid and sensors configured")
```

### Step 3: Create ROS 2 Bridge

```python
import omni.graph.core as og

# Create ROS 2 action graph
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("CameraPublisher", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("JointStatePublisher", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "CameraPublisher.inputs:execIn"),
            ("OnTick.outputs:tick", "JointStatePublisher.inputs:execIn"),
        ],
        keys.SET_VALUES: [
            ("ROS2Context.inputs:domain_id", 0),
            ("CameraPublisher.inputs:topicName", "/humanoid/camera/image_raw"),
            ("CameraPublisher.inputs:type", "rgb"),
            ("JointStatePublisher.inputs:topicName", "/joint_states"),
            ("JointStatePublisher.inputs:targetPrim", "/World/Humanoid"),
        ],
    },
)

print("ROS 2 bridge created")
```

### Step 4: Run Simulation

```python
# Reset world
world.reset()

# Run simulation loop
for i in range(1000):
    world.step(render=True)

    if i % 100 == 0:
        # Get camera image
        camera_data = head_camera.get_current_frame()
        print(f"Step {i}: Captured image shape {camera_data['rgba'].shape}")

print("Simulation complete")
```

### Step 5: Verify in RViz2

```bash
# In separate terminal
source /opt/ros/humble/setup.bash

# Launch RViz2
rviz2

# Add displays:
# - Image: /humanoid/camera/image_raw
# - RobotModel: /robot_description
# - TF: Shows coordinate frames

# You should see photorealistic camera feed from Isaac Sim!
```

---

## Challenge: Test Your Understanding

Try these exercises to reinforce your learning:

1. **Basic**: Modify the warehouse scene to include 5 random boxes on the ground
   - *Hint*: Use `DynamicCuboid` with random positions via `rep.distribution.uniform`

2. **Intermediate**: Create a replicator script that generates 100 images with random humanoid arm poses
   - *Hint*: Use `rep.modify.semantics` to change joint positions, then `rep.trigger.on_frame()` to capture

3. **Advanced**: Build a ROS 2 node that subscribes to Isaac Sim camera images and runs YOLO object detection
   - *Hint*: Use `cv_bridge` to convert ROS Image messages to OpenCV format, then apply YOLOv8

---

## Summary

In this chapter, you learned:

- **Isaac Sim's architecture**: PhysX, RTX, USD, and how it differs from Gazebo (photorealism, GPU acceleration)
- **USD workflows**: Creating scenes via composition, importing URDFs, configuring physics
- **Photorealistic rendering**: Attaching cameras, capturing RGB/depth/segmentation data
- **ROS 2 integration**: Using OmniGraph to publish sensor data and subscribe to commands
- **Synthetic data generation**: Replicator API for domain randomization and ground truth labels

**Key Commands**:

```bash
# Launch Isaac Sim GUI
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh

# Run headless Python script
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh your_script.py

# Verify ROS 2 topics from Isaac Sim
ros2 topic list
ros2 topic echo /joint_states
```

**Core Concepts**:
- **USD Composition**: Layering assets via references and sublayers for reusable scenes
- **PhysX Articulations**: GPU-accelerated physics for complex multi-body robots
- **RTX Ray Tracing**: Photorealistic rendering enabling sim-to-real perception transfer
- **Replicator**: Procedural data generation with automatic ground truth labeling

---

## Further Reading

Official Documentation:
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [USD Introduction by Pixar](https://graphics.pixar.com/usd/release/intro.html)
- [Omniverse Replicator](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html)

Tutorials and Examples:
- [Isaac Sim Tutorials (NVIDIA)](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials/index.html)
- [ROS 2 Bridge Setup Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_navigation.html)
- [Synthetic Data Generation Pipeline](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html)

Research Papers:
- "Sim-to-Real Transfer in Robotics: A Survey" (Zhao et al., 2020) - Domain randomization techniques
- "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (Tobin et al., 2017)

---

## Next Steps

You're now ready to move on to the next chapter!

**Next Chapter**: [Chapter 3.2: Isaac ROS Perception](./ch2-isaac-ros-perception.md)

In the next chapter, you'll learn about Isaac ROS GEMs (GPU-accelerated perception nodes), implement VSLAM for localization, and build complete perception pipelines for humanoid navigation.

**Optional Practice**:
- Explore the NVIDIA Assets library in Omniverse (hundreds of free 3D models)
- Try importing a custom URDF from your research or favorite open-source robot
- Join the NVIDIA Omniverse Discord for community support and examples
