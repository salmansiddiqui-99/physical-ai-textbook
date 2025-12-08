---
id: ch3-unity-visualization
title: "Chapter 2.3: Unity Visualization for Humanoid Robots"
sidebar_label: "2.3 Unity Visualization"
sidebar_position: 6
description: Creating realistic 3D visualizations and human-robot interaction environments with Unity and ROS 2 including VR telepresence.
learning_objectives:
  - Configure Unity with ROS 2 integration using Unity Robotics Hub
  - Import and visualize humanoid robots from URDF descriptions
  - Create realistic HRI environments with lighting and physics
  - Implement VR telepresence interfaces for robot control
---

# Chapter 2.3: Unity Visualization for Humanoid Robots

**Learning Objectives:**
- Set up Unity with ROS 2 integration (Unity Robotics Hub)
- Import and visualize humanoid robots from URDF
- Create realistic environments for human-robot interaction
- Synchronize Unity scenes with Gazebo physics
- Implement VR/AR visualizations for telepresence
- Build interactive demonstrations and user interfaces

**Estimated Time**: 150 minutes

---

## Introduction: Why Unity for Robotics?

While Gazebo excels at physics simulation, **Unity** provides:
- **Photorealistic rendering**: High-quality graphics for presentations and HRI
- **Cross-platform support**: Windows, macOS, Linux, mobile, VR/AR
- **Rich asset ecosystem**: Pre-made characters, environments, and animations
- **Powerful UI tools**: Build control panels, dashboards, and interactive demos
- **VR/AR native support**: Telepresence and immersive robot control

**Common Use Cases:**
- Human-robot interaction research (HRI)
- Marketing demos and visualizations
- VR telepresence for remote robot operation
- Data visualization overlays (sensor fusion, planning paths)
- User studies with non-technical participants

**Architecture:**
```
ROS 2 (Physics) ←→ Unity (Visualization)
   Gazebo               Realistic graphics
   Navigation           User interaction
   Control              VR/AR interfaces
```

---

## 1. Unity Robotics Hub Setup

### What is Unity Robotics Hub?

**Unity Robotics Hub** is Unity's official ROS integration, providing:
- **ROS-TCP-Connector**: Bidirectional communication between Unity and ROS 2
- **URDF Importer**: Convert URDF robot descriptions to Unity GameObjects
- **Visualization tools**: TF frames, sensor overlays, trajectory paths

### Installation Requirements

**System Requirements:**
- Unity Hub (latest version)
- Unity Editor 2022.3 LTS or newer
- ROS 2 Humble or Iron
- Ubuntu 22.04 or Windows 10/11

**Disk Space:** ~10 GB for Unity + packages

### Step 1: Install Unity Hub and Editor

**Ubuntu:**
```bash
# Download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage

# Run Unity Hub
./UnityHub.AppImage
```

**Windows:**
- Download Unity Hub from [unity.com/download](https://unity.com/download)
- Run installer
- Sign in or create Unity account (free for personal use)

**Install Unity Editor:**
1. Open Unity Hub
2. Go to "Installs" tab
3. Click "Install Editor"
4. Select **Unity 2022.3 LTS** (Long Term Support)
5. Add modules:
   - Linux Build Support (if on Windows)
   - Windows Build Support (if on Linux)
   - WebGL Build Support (for browser demos)

### Step 2: Create Unity Project

**In Unity Hub:**
1. Click "New Project"
2. Select template: **3D (URP)** (Universal Render Pipeline for better graphics)
3. Project name: `HumanoidVisualization`
4. Location: Choose directory
5. Click "Create Project"

**Wait for project initialization** (~5 minutes for first project).

### Step 3: Install Unity Robotics Hub Packages

**In Unity Editor:**

**A. Install ROS-TCP-Connector:**
1. Open **Window → Package Manager**
2. Click **+ → Add package from git URL**
3. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Click "Add"
5. Wait for installation

**B. Install URDF Importer:**
1. Package Manager → **+ → Add package from git URL**
2. Enter: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`
3. Click "Add"

**C. Install Visualizations (optional):**
1. Package Manager → **+ → Add package from git URL**
2. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations`
3. Click "Add"

### Step 4: Configure ROS-TCP-Endpoint (ROS 2 Side)

**On Ubuntu (ROS 2 machine):**

```bash
# Create workspace (if not exists)
mkdir -p ~/ros2_unity_ws/src
cd ~/ros2_unity_ws/src

# Clone ROS-TCP-Endpoint
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build
cd ~/ros2_unity_ws
colcon build --packages-select ros_tcp_endpoint

# Source workspace
source install/setup.bash
```

**Launch ROS-TCP-Endpoint:**
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Expected output:**
```
[INFO] [ros_tcp_endpoint]: ROS-TCP-Endpoint starting on 0.0.0.0:10000
```

This server bridges Unity and ROS 2 topics.

### Step 5: Connect Unity to ROS 2

**In Unity Editor:**

1. **Robotics → ROS Settings**
2. Set parameters:
   - **ROS IP Address**: `127.0.0.1` (localhost, or your ROS machine IP)
   - **ROS Port**: `10000` (default)
   - **Protocol**: `ROS2`
3. Click "Test Connection"
4. Should see: **"Connection successful!"**

**Troubleshooting connection issues:**
- Firewall blocking port 10000? Add exception
- ROS machine on different network? Use correct IP address
- ROS-TCP-Endpoint running? Check terminal for server logs

---

## 2. Importing URDF Robots into Unity

### Method 1: Direct URDF Import

**In Unity:**
1. **Assets → Import Robot from URDF**
2. Browse to your humanoid URDF file (e.g., `humanoid_full.urdf`)
3. Import settings:
   - **Axis Type**: Y Axis (Unity uses Y-up, ROS uses Z-up)
   - **Mesh Decomposer**: VHACD (for collision meshes)
4. Click "Import"

**Result:** Robot appears in scene as a GameObject with:
- Links as child GameObjects
- Joints as ArticulationBody components (Unity physics)
- Meshes rendered with materials

### Method 2: Export from Gazebo, Import to Unity

**Step 1: Spawn robot in Gazebo**
```bash
ros2 launch your_package spawn_humanoid.launch.py
```

**Step 2: Export URDF with mesh paths**
```bash
# Generate URDF with resolved mesh paths
ros2 run xacro xacro robot.urdf.xacro > robot_resolved.urdf
```

**Step 3: Copy meshes to Unity**
```bash
# Copy entire robot package to Unity Assets folder
cp -r ~/ros2_ws/src/your_robot_package ~/unity_project/Assets/Robots/
```

**Step 4: Import in Unity**
- Assets → Import Robot from URDF
- Select `robot_resolved.urdf`
- Meshes will be found automatically

### Understanding Unity Robot Structure

**Unity Hierarchy:**
```
humanoid_robot (GameObject)
├── base_link (ArticulationBody)
│   ├── torso (ArticulationBody)
│   │   ├── head (ArticulationBody)
│   │   ├── left_shoulder (ArticulationBody)
│   │   │   └── left_elbow (ArticulationBody)
│   │   └── right_shoulder (ArticulationBody)
│   ├── left_hip (ArticulationBody)
│   │   ├── left_knee (ArticulationBody)
│   │   └── left_ankle (ArticulationBody)
│   └── right_hip (ArticulationBody)
```

**ArticulationBody** is Unity's component for robotic joints:
- Replaces older Rigidbody for multi-joint robots
- Supports all URDF joint types
- Better stability for kinematic chains

---

## 3. Synchronizing Unity with Gazebo

### Architecture: Dual Simulation

**Two-way synchronization:**
```
Gazebo (Physics Authority)
    ↓ Joint states
Unity (Visualization)
    ↑ Teleop commands
```

**Use case:** Run physics in Gazebo (accurate), display in Unity (beautiful).

### Publishing Joint States from Gazebo

**ROS 2 node to bridge joint states:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros_tcp_endpoint import UnityMessage

class GazeboUnityBridge(Node):
    def __init__(self):
        super().__init__('gazebo_unity_bridge')

        # Subscribe to Gazebo joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publisher for Unity
        self.unity_pub = self.create_publisher(
            JointState,
            '/unity/joint_states',
            10
        )

        self.get_logger().info('Gazebo-Unity bridge started')

    def joint_callback(self, msg):
        # Forward joint states to Unity
        self.unity_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboUnityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscribing to Joint States in Unity

**Unity C# Script: `JointStateSubscriber.cs`**

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private ArticulationBody[] articulationBodies;

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/unity/joint_states", UpdateJointStates);

        // Get all articulation bodies in robot
        articulationBodies = GetComponentsInChildren<ArticulationBody>();

        Debug.Log($"Found {articulationBodies.Length} articulation bodies");
    }

    void UpdateJointStates(JointStateMsg msg)
    {
        // Match joint names and update positions
        for (int i = 0; i < msg.name.Length; i++)
        {
            string jointName = msg.name[i];
            float position = (float)msg.position[i];

            // Find matching articulation body
            foreach (var body in articulationBodies)
            {
                if (body.name == jointName)
                {
                    // Set joint target position
                    var drive = body.xDrive;
                    drive.target = position * Mathf.Rad2Deg; // Convert rad to deg
                    body.xDrive = drive;
                    break;
                }
            }
        }
    }
}
```

**Attach script to robot root GameObject** in Unity.

### Testing Synchronization

**Terminal 1:** Launch Gazebo
```bash
ros2 launch your_package spawn_humanoid.launch.py
```

**Terminal 2:** Run bridge node
```bash
python3 gazebo_unity_bridge.py
```

**Terminal 3:** Start ROS-TCP-Endpoint
```bash
ros2 run ros_tcp_endpoint default_server_endpoint
```

**Unity:** Press Play button

**Result:** Unity robot should mirror Gazebo robot movements.

---

## 4. Creating Realistic Environments

### Unity Asset Store

**Access free assets:**
1. Open **Window → Asset Store**
2. Search for:
   - "Indoor office" (office environments)
   - "Modular sci-fi" (futuristic labs)
   - "Nature starter kit" (outdoor scenes)
3. Click "Add to My Assets" → "Import"

**Recommended free packages:**
- **Polygonal Environments**: Low-poly stylized worlds
- **Standard Assets**: Furniture, props, characters
- **Skybox Series**: Realistic skies

### Building a Human-Robot Interaction Scene

**Example: Home Environment for Service Robot**

**Step 1: Create floor and walls**
```
1. GameObject → 3D Object → Plane (floor)
   - Scale: (5, 1, 5) for 50m x 50m room
   - Material: Wood or tile texture

2. GameObject → 3D Object → Cube (walls)
   - Create 4 cubes for walls
   - Position around floor perimeter
   - Scale: (0.1, 3, 10) for thin walls
```

**Step 2: Add furniture**
```
1. Import furniture assets from Asset Store
2. Drag prefabs into scene:
   - Tables at (2, 0, 2)
   - Chairs at (2, 0, 2.5)
   - Shelves at (-3, 0, -3)
```

**Step 3: Add lighting**
```
1. GameObject → Light → Directional Light (sunlight)
   - Rotation: (50, -30, 0)
   - Intensity: 1.5

2. GameObject → Light → Point Light (ceiling lights)
   - Position: (0, 3, 0)
   - Range: 10
   - Intensity: 5
```

**Step 4: Add human characters**
```
1. Asset Store → Search "Animated Character"
2. Import and add to scene
3. Use Animator component for walking/sitting animations
```

### Lighting for Realism

**Global Illumination:**
- **Window → Rendering → Lighting Settings**
- **Realtime Global Illumination**: On (for dynamic lighting)
- **Baked Global Illumination**: On (for static objects)
- **Auto Generate**: Click to bake lighting

**Post-Processing (URP):**
```
1. GameObject → Volume → Global Volume
2. Add Override → Bloom (glow effects)
3. Add Override → Ambient Occlusion (shadows in corners)
4. Add Override → Color Grading (adjust mood)
```

**Result:** Studio-quality rendering.

---

## 5. Interactive UI and Dashboards

### Unity UI Toolkit

**Create a robot control panel:**

**Step 1: Create Canvas**
```
1. GameObject → UI → Canvas
2. Canvas settings:
   - Render Mode: Screen Space - Overlay
   - UI Scale Mode: Scale With Screen Size
```

**Step 2: Add control elements**
```
1. Right-click Canvas → UI → Button
   - Text: "Move Forward"
   - Position: (100, -50, 0)

2. Right-click Canvas → UI → Slider
   - Name: "Speed Control"
   - Min Value: 0
   - Max Value: 2
   - Value: 0.5
```

**Step 3: Add script to handle button clicks**

**`RobotControlPanel.cs`:**

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotControlPanel : MonoBehaviour
{
    public Button moveForwardButton;
    public Slider speedSlider;
    private ROSConnection ros;
    private string topicName = "/cmd_vel";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);

        // Attach button click handler
        moveForwardButton.onClick.AddListener(SendMoveForwardCommand);
    }

    void SendMoveForwardCommand()
    {
        float speed = speedSlider.value;

        TwistMsg msg = new TwistMsg
        {
            linear = new Vector3Msg { x = speed, y = 0, z = 0 },
            angular = new Vector3Msg { x = 0, y = 0, z = 0 }
        };

        ros.Publish(topicName, msg);
        Debug.Log($"Sent move forward command: {speed} m/s");
    }
}
```

**Attach script to Canvas** and link Button and Slider in Inspector.

### Sensor Data Visualization

**Display IMU data in UI:**

**`IMUDisplay.cs`:**

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class IMUDisplay : MonoBehaviour
{
    public Text rollText;
    public Text pitchText;
    public Text yawText;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ImuMsg>("/humanoid/imu", UpdateIMUDisplay);
    }

    void UpdateIMUDisplay(ImuMsg msg)
    {
        // Convert quaternion to Euler angles
        Quaternion q = new Quaternion(
            (float)msg.orientation.x,
            (float)msg.orientation.y,
            (float)msg.orientation.z,
            (float)msg.orientation.w
        );

        Vector3 euler = q.eulerAngles;

        // Update UI text
        rollText.text = $"Roll: {euler.x:F1}°";
        pitchText.text = $"Pitch: {euler.y:F1}°";
        yawText.text = $"Yaw: {euler.z:F1}°";
    }
}
```

**UI Setup:**
```
1. Canvas → UI → Text (x3 for roll, pitch, yaw)
2. Attach IMUDisplay.cs to Canvas
3. Drag Text elements to script fields in Inspector
```

---

## 6. VR Telepresence for Robot Control

### Unity XR Plugin Setup

**Install VR support:**
1. **Edit → Project Settings → XR Plug-in Management**
2. Check platforms:
   - **Oculus** (Meta Quest)
   - **OpenVR** (HTC Vive, Valve Index)
   - **Windows Mixed Reality**
3. Click "Install XR Plugin Management"

### VR Camera Setup

**Replace main camera with VR rig:**
```
1. Delete Main Camera
2. GameObject → XR → XR Origin (Action-based)
   - Includes camera, left/right controllers
3. Add tracked pose driver components (automatic)
```

### VR Robot Control

**Control robot with VR hand controllers:**

**`VRRobotControl.cs`:**

```csharp
using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VRRobotControl : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/cmd_vel";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        // Get right controller joystick input
        InputDevice rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);

        Vector2 joystick;
        if (rightController.TryGetFeatureValue(CommonUsages.primary2DAxis, out joystick))
        {
            // Send velocity command
            TwistMsg msg = new TwistMsg
            {
                linear = new Vector3Msg { x = joystick.y, y = 0, z = 0 },
                angular = new Vector3Msg { x = 0, y = 0, z = joystick.x }
            };

            ros.Publish(topicName, msg);
        }
    }
}
```

**Attach to XR Origin GameObject.**

### VR First-Person View

**Stream robot camera feed to VR headset:**

**Step 1: Subscribe to camera topic in Unity**

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RobotCameraDisplay : MonoBehaviour
{
    private ROSConnection ros;
    public Renderer displayRenderer; // Quad or plane to show image

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ImageMsg>("/humanoid/camera/image_raw", UpdateCameraFeed);
    }

    void UpdateCameraFeed(ImageMsg msg)
    {
        // Convert ROS image to Unity texture
        Texture2D tex = new Texture2D((int)msg.width, (int)msg.height, TextureFormat.RGB24, false);
        tex.LoadRawTextureData(msg.data);
        tex.Apply();

        // Display on renderer
        displayRenderer.material.mainTexture = tex;
    }
}
```

**Step 2: Create display in VR**
```
1. GameObject → 3D Object → Quad
2. Position in VR space (e.g., floating screen)
3. Attach RobotCameraDisplay.cs
4. Link Quad's Renderer in Inspector
```

---

## 7. Performance Optimization

### Reducing Draw Calls

**Batching:**
- Static objects: Enable **Static** flag in Inspector
- Dynamic batching: Edit → Project Settings → Player → Other Settings → Dynamic Batching

**LOD (Level of Detail):**
```
1. Select robot mesh
2. Add Component → LOD Group
3. Add LOD levels:
   - LOD0: Full detail (close-up)
   - LOD1: Medium detail
   - LOD2: Low detail (distant)
```

### Frame Rate Optimization

**Target frame rate:**
```csharp
void Start()
{
    Application.targetFrameRate = 60; // Cap at 60 FPS
    QualitySettings.vSyncCount = 1;   // Sync to monitor refresh
}
```

**Profiler:**
- **Window → Analysis → Profiler**
- Identify bottlenecks (CPU, GPU, rendering)
- Optimize heavy scripts or meshes

### Network Latency

**ROS-TCP-Connector settings:**
- Reduce message frequency for non-critical topics
- Use QoS settings (best effort vs. reliable)
- Compress large messages (images, point clouds)

---

## 8. Hands-On Project: Interactive HRI Demo

### Project Goal

Create a demonstration where a user can:
1. Control a humanoid robot in a home environment
2. See sensor data in real-time dashboard
3. Switch between third-person and first-person (robot POV) views

### Step 1: Scene Setup

**Create environment:**
```
1. Import "Simple Home Interior" from Asset Store
2. Add humanoid robot (URDF import)
3. Position at (0, 0, 0)
```

### Step 2: Add Cameras

**Camera 1: Third-Person (default)**
```
- Position: (5, 3, -5)
- Rotation: (20, -30, 0)
- Look at robot
```

**Camera 2: First-Person (robot POV)**
```
- Attach to robot head link
- Position: (0.1, 0, 0) relative to head
- Forward direction: Z-axis
```

### Step 3: Create UI Dashboard

**Canvas with:**
- Speed slider (0-2 m/s)
- Direction buttons (Forward, Back, Left, Right, Stop)
- IMU readout (roll, pitch, yaw)
- Camera feed display
- Camera switch button

### Step 4: Add Control Scripts

**Combine previous scripts:**
- `JointStateSubscriber.cs` (sync with Gazebo)
- `RobotControlPanel.cs` (UI controls)
- `IMUDisplay.cs` (sensor data)
- `CameraSwitcher.cs` (toggle views)

**`CameraSwitcher.cs`:**

```csharp
using UnityEngine;
using UnityEngine.UI;

public class CameraSwitcher : MonoBehaviour
{
    public Camera thirdPersonCamera;
    public Camera firstPersonCamera;
    public Button switchButton;

    private bool isFirstPerson = false;

    void Start()
    {
        switchButton.onClick.AddListener(SwitchCamera);
        SetActiveCamera(thirdPersonCamera);
    }

    void SwitchCamera()
    {
        isFirstPerson = !isFirstPerson;

        if (isFirstPerson)
        {
            SetActiveCamera(firstPersonCamera);
            switchButton.GetComponentInChildren<Text>().text = "Third Person";
        }
        else
        {
            SetActiveCamera(thirdPersonCamera);
            switchButton.GetComponentInChildren<Text>().text = "First Person";
        }
    }

    void SetActiveCamera(Camera cam)
    {
        thirdPersonCamera.enabled = (cam == thirdPersonCamera);
        firstPersonCamera.enabled = (cam == firstPersonCamera);
    }
}
```

### Step 5: Test Full System

**Terminal 1:** Gazebo simulation
```bash
ros2 launch your_package spawn_humanoid.launch.py
```

**Terminal 2:** ROS-TCP-Endpoint
```bash
ros2 run ros_tcp_endpoint default_server_endpoint
```

**Unity:** Press Play

**Test:**
- Click direction buttons → robot moves in Gazebo and Unity
- Adjust speed slider → movement speed changes
- Click "First Person" → see robot's view
- IMU display updates in real-time

---

## 9. Advanced Topics

### Unity Machine Learning Agents (ML-Agents)

**Train robots with reinforcement learning:**
- Install ML-Agents Toolkit: `pip install mlagents`
- Define observations (sensor inputs)
- Define actions (joint commands)
- Define rewards (task completion)
- Train with PPO algorithm

**Use case:** Train walking gaits in Unity, deploy to real robot.

### WebGL Deployment

**Build for browser:**
1. **File → Build Settings**
2. Select **WebGL** platform
3. Click "Build"
4. Upload to web server or Unity Play

**Use case:** Public-facing demos, no installation required.

### Photogrammetry Integration

**Use real-world scans:**
1. Capture environment with smartphone (apps: Polycam, RealityScan)
2. Export as OBJ or FBX mesh
3. Import to Unity Assets
4. Add colliders for physics

**Use case:** Train robots in digital twins of real spaces.

---

## 10. Unity vs. Gazebo: When to Use Each

| Feature                | Gazebo                        | Unity                          |
|------------------------|-------------------------------|--------------------------------|
| **Physics accuracy**   | ✅ Excellent (ODE, Bullet)    | ⚠️ Good (PhysX, ArticulationBody) |
| **Graphics quality**   | ⚠️ Basic                      | ✅ Photorealistic (URP, HDRP)  |
| **ROS integration**    | ✅ Native                     | ⚠️ Via ROS-TCP-Connector       |
| **VR/AR support**      | ❌ None                       | ✅ Native                      |
| **Sensor simulation**  | ✅ Extensive plugins          | ⚠️ Limited (requires custom)   |
| **Cross-platform**     | 🐧 Linux primary              | ✅ Windows, macOS, Linux, mobile |
| **Learning curve**     | ⚠️ Moderate (roboticists)     | ⚠️ Moderate (game developers)  |
| **Best for**           | Algorithm development         | Visualization, HRI, demos      |

**Hybrid Approach (Recommended):**
- **Gazebo**: Physics simulation, control development
- **Unity**: Visualization, user interfaces, VR telepresence
- **Bridge**: ROS-TCP-Connector for real-time sync

---

## Summary

You've learned to use Unity for humanoid robot visualization:

**Unity Robotics Hub:**
- ROS-TCP-Connector for bidirectional ROS 2 communication
- URDF Importer to bring robots from ROS to Unity
- ArticulationBody for physics-based joint control

**Synchronization:**
- Bridge joint states from Gazebo to Unity
- Publish commands from Unity to ROS 2
- Maintain Gazebo as physics authority

**Realistic Environments:**
- Asset Store for pre-made environments
- Lighting and post-processing for photorealism
- Human characters for HRI simulations

**Interactive UIs:**
- Unity UI Toolkit for control panels
- Real-time sensor data displays
- Custom dashboards for demos

**VR/AR:**
- XR Plugin Management for VR headset support
- First-person robot control with hand controllers
- Camera feed streaming to VR displays

**Best Practices:**
- Use Unity for visualization, Gazebo for physics
- Optimize performance (LOD, batching, frame caps)
- Test with real ROS 2 workflows before deployment

---

## Exercises

### Exercise 1: Multi-Robot Visualization

Import two humanoid robots into Unity scene:
- Robot 1 at (2, 0, 0)
- Robot 2 at (-2, 0, 0)
- Subscribe to separate joint state topics
- Control independently

**Test:** Make them wave at each other.

### Exercise 2: Custom Sensor Visualization

Create a script that:
- Subscribes to `/humanoid/scan` (LiDAR)
- Draws laser rays as LineRenderer in Unity
- Colors rays red when obstacle detected (&lt;1m)

**Hint:** Use `LineRenderer` component with dynamic vertex count.

### Exercise 3: VR Teleoperation

Set up full VR control:
- Left joystick: Robot movement (linear, angular)
- Right trigger: Open/close gripper
- Head pose: Control robot head orientation
- Display first-person camera feed in VR

**Test:** Pick up an object in VR.

### Exercise 4: Web-Based Dashboard

Build a WebGL application that:
- Shows robot in 3D environment
- Has control buttons (browser UI)
- Connects to ROS 2 via WebSocket (rosbridge)
- Runs in browser without installation

---

## Troubleshooting

### Problem: Unity Can't Connect to ROS 2

**Check:**
```bash
# Is ROS-TCP-Endpoint running?
ros2 node list | grep tcp_endpoint

# Is port 10000 open?
sudo netstat -tuln | grep 10000
```

**Solutions:**
- Restart ROS-TCP-Endpoint
- Check firewall settings
- Verify IP address in Unity ROS Settings

### Problem: Robot Appears Distorted in Unity

**Cause:** URDF axis mismatch (ROS uses Z-up, Unity uses Y-up)

**Solution:**
- Re-import URDF with **Axis Type: Y Axis**
- Or rotate root GameObject by -90° around X-axis

### Problem: Joints Don't Move

**Check:**
```csharp
// In JointStateSubscriber.cs, add debug:
Debug.Log($"Received joint: {jointName} at {position}");
```

**Common issues:**
- Joint names don't match between URDF and messages
- Drive limits too restrictive
- ArticulationBody locked (check "Immovable" flag)

### Problem: Low Frame Rate in Unity

**Solutions:**
- Reduce mesh complexity (use LOD)
- Lower camera resolution
- Disable shadows on non-critical objects
- Use Profiler to find bottlenecks

---

## Additional Resources

**Unity Robotics Hub:**
- [GitHub Repository](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Tutorials](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/README.md)

**Unity Documentation:**
- [XR Interaction Toolkit](https://docs.unity3d.com/Packages/com.unity.xr.interaction.toolkit@latest)
- [ArticulationBody API](https://docs.unity3d.com/ScriptReference/ArticulationBody.html)

**Community:**
- [Unity Robotics Forum](https://forum.unity.com/forums/robotics.617/)
- [ROS Discourse - Unity](https://discourse.ros.org/tag/unity)

**Videos:**
- "ROS 2 + Unity Integration Tutorial" (Unity YouTube)
- "Building VR Robot Telepresence in Unity"

---

**Next Module**: Module 3: NVIDIA Isaac Platform - coming soon

**Previous Chapter**: [2.2 Sensor Simulation](./ch2-sensor-simulation.md)

**Module Navigation**: [Course Home](../intro.md)

---

**Estimated Completion Time**: 2.5 hours
**Prerequisites**: Module 1, Chapters 2.1-2.2, Unity Editor installed
**Difficulty**: Intermediate to Advanced
