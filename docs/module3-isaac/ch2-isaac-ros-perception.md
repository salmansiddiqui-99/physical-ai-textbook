---
id: ch2-isaac-ros-perception
title: "Isaac ROS Perception: GPU-Accelerated Vision Pipelines"
sidebar_label: "Isaac ROS Perception"
sidebar_position: 8
description: Learn Isaac ROS GEMs for GPU-accelerated perception including visual SLAM, stereo depth estimation, object detection, and DNN inference for real-time humanoid robot perception.
keywords:
  - Isaac ROS
  - VSLAM
  - cuVSLAM
  - stereo depth
  - DNN inference
  - GPU acceleration
  - perception pipeline
  - NITROS
prerequisites:
  - Completion of Module 1 (ROS 2 Foundations) and Module 2 (Simulation)
  - Chapter 3.1 (Isaac Sim Basics)
  - NVIDIA GPU with CUDA support (RTX 2060 or higher)
  - ROS 2 Humble installed
learning_objectives:
  - Explain Isaac ROS architecture and NITROS zero-copy acceleration framework
  - Implement visual SLAM using cuVSLAM for real-time localization
  - Configure stereo depth estimation pipelines with disparity and point clouds
  - Integrate DNN models for object detection and semantic segmentation on GPU
estimated_time: 90 minutes
---

# Isaac ROS Perception: GPU-Accelerated Vision Pipelines

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain Isaac ROS GEMs architecture and how NITROS enables zero-copy GPU acceleration
- Implement cuVSLAM (visual SLAM) for real-time localization and mapping
- Configure stereo depth pipelines to generate dense point clouds for obstacle avoidance
- Integrate TensorRT-accelerated DNNs for object detection and semantic segmentation

## Prerequisites

Before starting this chapter, you should:

- Have completed [Chapter 3.1: Isaac Sim Basics](./ch1-isaac-sim.md)
- Understand ROS 2 topics, nodes, and launch files from [Module 1](../module1-ros2/ch1-ros2-basics.md)
- Have an NVIDIA GPU with CUDA support (RTX 2060+, Jetson Xavier/Orin for embedded)
- Have ROS 2 Humble and Isaac ROS packages installed (installation guide below)

## Introduction

Traditional ROS 2 perception nodes (like ORB-SLAM3, OpenCV stereo matching) run on CPU, limiting throughput to 5-15 FPS for complex tasks. For humanoid robots navigating dynamic environments, this latency is unacceptable—a robot moving at 1 m/s travels 20cm between 5 Hz perception updates, increasing collision risk.

NVIDIA Isaac ROS solves this with **GEMs** (GPU-Enabled Modules): drop-in ROS 2 nodes that offload perception to the GPU. Using **NITROS** (NVIDIA Isaac Transport for ROS), data flows directly between GPU-accelerated nodes without copying to CPU memory, achieving 10-100x speedups. For example, cuVSLAM runs visual SLAM at 60+ FPS on an RTX 3060, enabling real-time tracking for fast-moving humanoids.

This chapter covers the core Isaac ROS GEMs for humanoid perception: cuVSLAM for localization, stereo depth for 3D sensing, and TensorRT inference for AI-based object recognition. You'll build complete perception pipelines that run in Isaac Sim and transfer directly to physical robots with NVIDIA Jetson or discrete GPUs.

---

## Section 1: Isaac ROS Architecture and NITROS

### Subsection 1.1: What is Isaac ROS?

Isaac ROS is a collection of hardware-accelerated ROS 2 packages for robotics perception, manipulation, and navigation.

**Key Components**:
- **cuVSLAM**: Visual SLAM (Simultaneous Localization and Mapping)
- **Stereo Depth**: Dense depth maps from stereo cameras
- **Image Processing**: Rectification, debayering, resize (all on GPU)
- **DNN Inference**: TensorRT-accelerated object detection, segmentation
- **AprilTag**: Fiducial marker detection for precise localization
- **NITROS**: Zero-copy transport layer for GPU data

**Why GPU Acceleration Matters**:

| Task | CPU (8-core i7) | GPU (RTX 3060) | Speedup |
|------|----------------|----------------|---------|
| Stereo depth matching (1280x720) | 12 FPS | 120 FPS | 10x |
| YOLO object detection | 8 FPS | 60 FPS | 7.5x |
| Visual SLAM (cuVSLAM) | N/A (requires GPU) | 60+ FPS | ∞ |
| Semantic segmentation (DeepLabV3) | 3 FPS | 45 FPS | 15x |

### Subsection 1.2: NITROS: Zero-Copy Transport

**Problem with Standard ROS 2**:
1. Node A (GPU) processes image → copies result to CPU RAM
2. ROS 2 middleware serializes data and sends via DDS
3. Node B receives → deserializes → copies to GPU RAM

**NITROS Solution**:
1. Node A processes image → keeps data in GPU RAM
2. NITROS passes GPU memory pointer (no copy!)
3. Node B receives pointer → processes directly on GPU

**Result**: 50-80% latency reduction, 3-5x throughput increase.

**NITROS Message Types**:
- `nitros_bridge/NitrosImage`: GPU-resident images
- `nitros_bridge/NitrosDisparity`: Stereo disparity maps
- `nitros_bridge/NitrosCameraInfo`: Camera calibration
- `nitros_bridge/NitrosPointCloud`: 3D point clouds

**Key Concept**: Standard ROS 2 nodes can subscribe to NITROS topics (NITROS auto-converts to CPU), but for best performance, chain NITROS-enabled nodes together.

### Subsection 1.3: Installation

**Prerequisites**:
- Ubuntu 22.04 LTS
- NVIDIA GPU with CUDA 11.8+ (or Jetson with JetPack 5.1+)
- ROS 2 Humble

**Install Isaac ROS**:
```bash
# Install NVIDIA Container Toolkit (for Docker-based workflows)
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Test GPU access in Docker
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# Clone Isaac ROS common repository
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
git checkout main

# Run Isaac ROS Docker container (includes all dependencies)
./scripts/run_dev.sh
```

**Install Specific Isaac ROS Packages** (inside container):
```bash
# Inside Isaac ROS Docker container
cd /workspaces/isaac_ros-dev/src

# cuVSLAM
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Stereo depth
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# DNN inference
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

# Build
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

**Verify Installation**:
```bash
ros2 pkg list | grep isaac_ros
# Should show: isaac_ros_visual_slam, isaac_ros_stereo_image_proc, etc.
```

---

## Section 2: Visual SLAM with cuVSLAM

### Subsection 2.1: What is Visual SLAM?

**SLAM (Simultaneous Localization and Mapping)** solves two problems simultaneously:
1. **Localization**: "Where am I?" (estimate robot pose)
2. **Mapping**: "What does the environment look like?" (build 3D map)

**Visual SLAM (VSLAM)** uses cameras instead of LiDAR. Advantages for humanoids:
- Lightweight sensors (cameras are smaller/cheaper than LiDAR)
- Rich texture information (useful for object recognition)
- Passive sensing (no spinning parts, silent operation)

**cuVSLAM Algorithm**:
1. **Feature Extraction**: Detect ORB keypoints in each frame (GPU-accelerated)
2. **Tracking**: Match keypoints between frames to estimate camera motion
3. **Mapping**: Triangulate 3D positions of landmarks (bundle adjustment)
4. **Loop Closure**: Detect revisited locations to correct drift

### Subsection 2.2: Running cuVSLAM with Stereo Camera

**Hardware Setup**:
- Stereo camera (e.g., RealSense D455, ZED 2) or simulated stereo in Isaac Sim
- Baseline (distance between cameras): 10-20cm for humanoid head mounting

**Example: Launch cuVSLAM**:

```bash
# Source workspace
source /workspaces/isaac_ros-dev/install/setup.bash

# Launch cuVSLAM node
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
  camera_info_topic:=/camera/left/camera_info \
  image_topic:=/camera/left/image_raw \
  depth_image_topic:=/camera/depth/image_raw \
  imu_topic:=/imu/data
```

**Configuration Parameters** (`isaac_ros_visual_slam/config/params.yaml`):
```yaml
visual_slam:
  ros__parameters:
    # Enable IMU fusion for better pose estimation
    enable_imu_fusion: true
    gyro_noise_density: 0.000244  # rad/s/√Hz (sensor-dependent)
    gyro_random_walk: 0.000019    # rad/s²/√Hz
    accel_noise_density: 0.001    # m/s²/√Hz
    accel_random_walk: 0.0002     # m/s³/√Hz

    # Map management
    enable_localization_n_mapping: true
    map_frame: 'map'
    odom_frame: 'odom'
    base_frame: 'base_link'

    # Performance tuning
    num_cameras: 1  # Use 2 for stereo
    rectified_images: true  # Already rectified by image pipeline
    enable_slam_visualization: true  # Publish landmarks for RViz
    enable_observations_view: true  # Debug feature matching

    # Loop closure
    enable_loop_closure: true
    min_loop_closure_score: 0.15  # Lower = more aggressive closure
```

### Subsection 2.3: Visualizing SLAM Output

**Published Topics**:
```bash
ros2 topic list
# /visual_slam/tracking/odometry         # Pose estimate (nav_msgs/Odometry)
# /visual_slam/tracking/vo_pose          # Visual odometry pose
# /visual_slam/tracking/slam_path        # Full trajectory path
# /visual_slam/status                    # SLAM status (tracking, lost, etc.)
# /visual_slam/vis/landmarks_cloud       # 3D map points (for RViz)
```

**Visualize in RViz2**:
```bash
ros2 run rviz2 rviz2
```

Add these displays:
1. **Odometry** → Topic: `/visual_slam/tracking/odometry`
2. **Path** → Topic: `/visual_slam/tracking/slam_path`
3. **PointCloud2** → Topic: `/visual_slam/vis/landmarks_cloud`
4. **TF** → Shows `map`, `odom`, `base_link` frames

You should see the robot trajectory as a colored path and map points as a 3D cloud.

### Subsection 2.4: Integrating with Isaac Sim

**Example: Run cuVSLAM with Isaac Sim Camera**:

**Step 1**: Launch Isaac Sim with humanoid and stereo camera (see Chapter 3.1)

**Step 2**: Create launch file for cuVSLAM:

```python
# File: isaac_sim_vslam.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # cuVSLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_imu_fusion': True,
                'num_cameras': 1,
                'rectified_images': True,
                'enable_localization_n_mapping': True
            }],
            remappings=[
                ('/camera/image_raw', '/humanoid/head_camera/left/image_raw'),
                ('/camera/camera_info', '/humanoid/head_camera/left/camera_info'),
                ('/imu/data', '/humanoid/imu/data')
            ]
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'isaac_slam.rviz']
        )
    ])
```

**Step 3**: Launch and verify:
```bash
ros2 launch isaac_sim_vslam.launch.py

# In Isaac Sim, move the robot (teleop or scripted path)
# You should see SLAM tracking in RViz with path visualization
```

---

## Section 3: Stereo Depth Estimation

### Subsection 3.1: Stereo Vision Fundamentals

**Principle**: Two cameras separated by baseline *B* capture the same scene. Objects appear shifted (disparity *d*) between left/right images. Depth *Z* is computed as:

```
Z = (f * B) / d
```

Where:
- *f*: Focal length (pixels)
- *B*: Baseline (meters, distance between camera centers)
- *d*: Disparity (pixels, shift between left/right image points)

**Example**:
- Baseline = 0.12m (12cm)
- Focal length = 400 pixels
- Disparity = 50 pixels

Depth = (400 * 0.12) / 50 = 0.96 meters

### Subsection 3.2: Isaac ROS Stereo Pipeline

**Nodes in Pipeline**:
1. **Rectify**: Align left/right images to same plane (removes lens distortion)
2. **Stereo Matching**: Compute disparity map (GPU-accelerated SGM algorithm)
3. **Point Cloud**: Convert disparity to 3D points

**Launch Stereo Pipeline**:
```bash
ros2 launch isaac_ros_stereo_image_proc isaac_ros_stereo_image_pipeline.launch.py \
  left_image_topic:=/camera/left/image_raw \
  left_camera_info_topic:=/camera/left/camera_info \
  right_image_topic:=/camera/right/image_raw \
  right_camera_info_topic:=/camera/right/camera_info
```

**Configuration** (`stereo_pipeline_params.yaml`):
```yaml
stereo_image_proc:
  ros__parameters:
    # Stereo matching algorithm parameters
    use_color: false  # Use grayscale for matching (faster)

    # Disparity range (inversely related to depth range)
    min_disparity: 0  # Max depth = infinity
    max_disparity: 128  # Min depth = (f*B)/128

    # Smoothness vs. accuracy trade-off
    p1: 8  # Penalty for small disparity changes (lower = smoother)
    p2: 109  # Penalty for large disparity changes

    # Matching window size
    window_size: 5  # Larger = more robust but less detail

    # Post-filtering
    uniqueness_ratio: 15  # Reject ambiguous matches (%)
    disp12_max_diff: 1  # Left-right consistency check
```

### Subsection 3.3: Generating Point Clouds

**Published Topics After Pipeline**:
```bash
# /stereo/disparity         # Disparity image (stereo_msgs/DisparityImage)
# /stereo/points2           # 3D point cloud (sensor_msgs/PointCloud2)
# /stereo/depth             # Depth image (sensor_msgs/Image, float32)
```

**Visualize Point Cloud in RViz**:
```bash
ros2 run rviz2 rviz2

# Add PointCloud2 display
# Topic: /stereo/points2
# Fixed Frame: camera_link
# You should see dense 3D reconstruction of the scene
```

**Accessing Point Cloud Data Programmatically**:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/stereo/points2',
            self.point_cloud_callback,
            10
        )

    def point_cloud_callback(self, msg):
        # Extract XYZ points from point cloud
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        count = 0
        close_obstacles = 0
        for p in points:
            x, y, z = p
            count += 1

            # Count points within 1 meter (obstacle detection)
            if z < 1.0 and abs(x) < 0.5:  # 1m ahead, ±0.5m left/right
                close_obstacles += 1

        if count > 0:
            obstacle_ratio = close_obstacles / count
            self.get_logger().info(
                f"Point cloud: {count} points, {obstacle_ratio*100:.1f}% obstacles within 1m"
            )

rclpy.init()
node = PointCloudProcessor()
rclpy.spin(node)
```

---

## Section 4: DNN Inference with TensorRT

### Subsection 4.1: TensorRT Acceleration

**TensorRT** is NVIDIA's deep learning inference optimizer. It converts trained models (PyTorch, TensorFlow) to highly optimized GPU engines.

**Optimizations**:
- **Layer Fusion**: Combine Conv + BatchNorm + ReLU into single kernel
- **Precision Calibration**: Use FP16 or INT8 instead of FP32 (2-4x speedup)
- **Kernel Autotuning**: Profile and select fastest CUDA kernels per layer

**Typical Speedups**:
- YOLOv5 (640x640): 80 FPS (TensorRT FP16) vs. 20 FPS (PyTorch FP32)
- ResNet50: 500 FPS vs. 150 FPS
- SegFormer: 35 FPS vs. 8 FPS

### Subsection 4.2: Object Detection with YOLO

**Isaac ROS DNN Inference** supports TensorRT-optimized models for object detection.

**Step 1: Convert YOLO Model to TensorRT**:
```bash
# Export PyTorch model to ONNX
python export.py --weights yolov5s.pt --include onnx

# Convert ONNX to TensorRT engine (FP16 precision)
/usr/src/tensorrt/bin/trtexec \
  --onnx=yolov5s.onnx \
  --saveEngine=yolov5s_fp16.engine \
  --fp16 \
  --workspace=4096  # 4GB workspace for layer optimization
```

**Step 2: Launch Isaac ROS DNN Inference Node**:
```python
# File: yolo_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_dnn_inference',
            executable='isaac_ros_dnn_inference',
            name='yolo_inference',
            parameters=[{
                'model_file_path': '/path/to/yolov5s_fp16.engine',
                'engine_file_path': '/path/to/yolov5s_fp16.engine',
                'input_tensor_names': ['images'],
                'input_binding_names': ['images'],
                'output_tensor_names': ['output'],
                'output_binding_names': ['output'],
                'verbose': True,
                'force_engine_update': False
            }],
            remappings=[
                ('/tensor_pub', '/yolo/detections'),
                ('/image', '/humanoid/camera/image_raw')
            ]
        ),

        # Decoder node (converts raw tensors to bounding boxes)
        Node(
            package='isaac_ros_yolov5',
            executable='yolov5_decoder_node',
            name='yolo_decoder',
            parameters=[{
                'confidence_threshold': 0.5,
                'nms_threshold': 0.4
            }],
            remappings=[
                ('/detections', '/yolo/bounding_boxes')
            ]
        )
    ])
```

**Step 3: Visualize Detections**:
```bash
# Subscribe to bounding boxes
ros2 topic echo /yolo/bounding_boxes

# Or use image_view with overlays
ros2 run image_view image_view --ros-args \
  --remap /image:=/yolo/image_with_boxes
```

### Subsection 4.3: Semantic Segmentation

**Semantic Segmentation** labels every pixel with a class (e.g., "floor", "wall", "person").

**Example: Run SegFormer for Scene Understanding**:

**Step 1**: Convert SegFormer to TensorRT (similar to YOLO)

**Step 2**: Launch segmentation node:
```bash
ros2 launch isaac_ros_dnn_inference segformer_inference.launch.py \
  model_path:=/path/to/segformer_b2_fp16.engine \
  image_topic:=/humanoid/camera/image_raw \
  output_topic:=/segmentation/mask
```

**Step 3**: Process segmentation mask:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class SegmentationProcessor(Node):
    def __init__(self):
        super().__init__('segmentation_processor')
        self.subscription = self.create_subscription(
            Image,
            '/segmentation/mask',
            self.mask_callback,
            10
        )

        # Class labels (example for indoor scenes)
        self.classes = {
            0: 'floor', 1: 'wall', 2: 'furniture',
            3: 'door', 4: 'person', 5: 'object'
        }

    def mask_callback(self, msg):
        # Convert ROS Image to numpy array
        mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)

        # Count pixels per class
        unique, counts = np.unique(mask, return_counts=True)
        total_pixels = mask.size

        for class_id, count in zip(unique, counts):
            class_name = self.classes.get(class_id, 'unknown')
            percentage = (count / total_pixels) * 100
            self.get_logger().info(f"{class_name}: {percentage:.1f}%")

rclpy.init()
node = SegmentationProcessor()
rclpy.spin(node)
```

**Use Cases for Humanoids**:
- **Floor detection**: Identify safe walking surfaces
- **Obstacle classification**: Distinguish between static furniture and dynamic people
- **Door/opening detection**: Plan navigation through doorways

---

## Section 5: Complete Perception Pipeline

### Subsection 5.1: Integrating cuVSLAM + Stereo + DNN

**Architecture**:
```
Isaac Sim Cameras → cuVSLAM (localization)
                  ↘
                   Stereo Depth (obstacle avoidance)
                  ↘
                   YOLO Detection (object recognition)
                  ↘
                   Navigation Stack (path planning)
```

**Launch File for Complete Pipeline**:
```python
# File: humanoid_perception_pipeline.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. cuVSLAM for localization
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{'enable_imu_fusion': True}],
            remappings=[
                ('/camera/image_raw', '/humanoid/camera/left/image_raw'),
                ('/camera/camera_info', '/humanoid/camera/left/camera_info')
            ]
        ),

        # 2. Stereo depth for 3D sensing
        Node(
            package='isaac_ros_stereo_image_proc',
            executable='stereo_image_proc',
            name='stereo_proc',
            remappings=[
                ('/left/image_raw', '/humanoid/camera/left/image_raw'),
                ('/right/image_raw', '/humanoid/camera/right/image_raw')
            ]
        ),

        # 3. YOLO object detection
        Node(
            package='isaac_ros_dnn_inference',
            executable='isaac_ros_dnn_inference',
            name='yolo_inference',
            parameters=[{'model_file_path': '/models/yolov5s_fp16.engine'}],
            remappings=[('/image', '/humanoid/camera/left/image_raw')]
        ),

        # 4. RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'perception_pipeline.rviz']
        )
    ])
```

### Subsection 5.2: Performance Profiling

**Measure Pipeline Latency**:
```bash
# Check node computation time
ros2 run ros2_performance_test latency_tester \
  --topic /humanoid/camera/left/image_raw \
  --output-topic /yolo/bounding_boxes

# Typical results on RTX 3060:
# Camera capture → YOLO output: 25-30ms (33 FPS)
# Camera capture → Stereo depth: 15ms (66 FPS)
# Camera capture → SLAM pose: 18ms (55 FPS)
```

**Optimization Tips**:
1. **Use NITROS throughout**: Avoid CPU conversions
2. **Reduce image resolution**: 1280x720 instead of 1920x1080 (4x faster)
3. **Lower DNN precision**: FP16 or INT8 (2-4x faster, minimal accuracy loss)
4. **Disable unused outputs**: Don't publish debug visualizations in production

---

## Hands-On Project: Humanoid Navigation with Perception

**Goal**: Build a perception pipeline where a humanoid localizes with VSLAM, detects obstacles with stereo, and recognizes objects with YOLO.

**Duration**: 45 minutes

**What You'll Learn**:
- Integrate multiple Isaac ROS nodes into a single pipeline
- Tune stereo depth parameters for humanoid height
- Combine SLAM and object detection for semantic mapping

### Step 1: Launch Isaac Sim with Humanoid and Stereo Camera

(Use setup from Chapter 3.1, ensure stereo camera is attached to robot head)

### Step 2: Create Multi-Sensor Launch File

```python
# File: humanoid_multi_sensor.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # cuVSLAM
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{'enable_imu_fusion': True}]
            ),
            # Stereo depth
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::StereoImageProcNode',
                name='stereo_proc'
            ),
        ],
        output='screen'
    )

    return LaunchDescription([container])
```

### Step 3: Configure RViz for Multi-Sensor Visualization

Create `perception_pipeline.rviz`:
```yaml
Panels:
  - Class: rviz_common/Displays
    Displays:
      - Class: rviz_default_plugins/TF
      - Class: rviz_default_plugins/Path
        Topic: /visual_slam/tracking/slam_path
      - Class: rviz_default_plugins/PointCloud2
        Topic: /stereo/points2
      - Class: rviz_default_plugins/Image
        Topic: /yolo/image_with_boxes
```

### Step 4: Run and Verify

```bash
# Terminal 1: Launch Isaac Sim with humanoid
# (From Chapter 3.1)

# Terminal 2: Launch perception pipeline
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch humanoid_multi_sensor.launch.py

# Terminal 3: Teleop robot to move around
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Observe in RViz:
# - SLAM path (blue line showing robot trajectory)
# - Point cloud (3D obstacles)
# - Object detection boxes (if objects in scene)
```

---

## Challenge: Test Your Understanding

Try these exercises to reinforce your learning:

1. **Basic**: Modify stereo depth parameters to change detection range (min/max depth)
   - *Hint*: Adjust `max_disparity` to control minimum depth threshold

2. **Intermediate**: Create a ROS 2 node that subscribes to SLAM pose and stereo point cloud, publishes "obstacle ahead" warning if points detected within 1m
   - *Hint*: Filter point cloud in base_link frame, count points in front cone

3. **Advanced**: Integrate a custom-trained YOLO model to detect humanoid-specific objects (e.g., doorknobs, elevator buttons)
   - *Hint*: Train YOLOv5 on custom dataset, convert to TensorRT, update DNN inference node config

---

## Summary

In this chapter, you learned:

- **Isaac ROS architecture**: GPU-accelerated perception with NITROS zero-copy transport (10-100x speedups)
- **cuVSLAM**: Visual SLAM for real-time localization at 60+ FPS using stereo cameras
- **Stereo depth pipelines**: Dense 3D reconstruction for obstacle avoidance and navigation
- **TensorRT DNN inference**: Real-time object detection (YOLO) and semantic segmentation at 30-60 FPS

**Key Commands**:

```bash
# Launch cuVSLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Launch stereo depth pipeline
ros2 launch isaac_ros_stereo_image_proc isaac_ros_stereo_image_pipeline.launch.py

# Check GPU utilization
nvidia-smi

# Monitor ROS 2 topics
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /stereo/points2
```

**Core Concepts**:
- **NITROS**: Zero-copy GPU memory sharing between ROS 2 nodes (avoids CPU bottlenecks)
- **cuVSLAM**: ORB feature tracking + bundle adjustment for drift-free localization
- **Stereo Matching**: Disparity computation via Semi-Global Matching (SGM) on GPU
- **TensorRT**: Optimized DNN inference with layer fusion and mixed precision

---

## Further Reading

Official Documentation:
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [cuVSLAM Technical Details](https://developer.nvidia.com/isaac-vslam)
- [NITROS Transport Whitepaper](https://docs.nvidia.com/isaac/ros/nitros/index.html)

Tutorials and Examples:
- [Isaac ROS Tutorials (GitHub)](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
- [Stereo Depth with Isaac ROS](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_image_pipeline/isaac_ros_stereo_image_proc/index.html)
- [TensorRT Best Practices](https://docs.nvidia.com/deeplearning/tensorrt/best-practices/index.html)

Research Papers:
- "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM" (Campos et al., 2021)
- "Semi-Global Matching for Stereo Vision" (Hirschmuller, 2008) - Algorithm used by Isaac ROS

---

## Next Steps

You're now ready to move on to the next chapter!

**Next Chapter**: [Chapter 3.3: Navigation for Humanoids](./ch3-navigation-humanoids.md)

In the next chapter, you'll learn how to integrate Isaac ROS perception with the Nav2 navigation stack, implement bipedal-specific path planners, and deploy full autonomous navigation for humanoid robots.

**Optional Practice**:
- Benchmark cuVSLAM on your GPU and compare to CPU SLAM (ORB-SLAM3)
- Collect a rosbag of stereo images in Isaac Sim, replay offline for testing
- Train a custom object detector for your humanoid's task (e.g., detect cups for grasping)
