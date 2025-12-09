# Unity + ROS 2 Setup Guide for Humanoid Visualization

This guide walks you through setting up Unity with ROS 2 integration for humanoid robot visualization.

## Prerequisites

- **Operating System**: Windows 10/11, Ubuntu 22.04, or macOS 12+
- **Unity Hub**: Latest version ([download](https://unity.com/download))
- **Unity Editor**: 2022.3 LTS or newer
- **ROS 2**: Humble or Iron distribution
- **Disk Space**: ~10 GB for Unity + packages

## Part 1: Install Unity

### Step 1.1: Download and Install Unity Hub

**Ubuntu:**
```bash
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage
chmod +x UnityHub.AppImage
./UnityHub.AppImage
```

**Windows/macOS:**
- Download from [unity.com/download](https://unity.com/download)
- Run installer
- Sign in or create free Unity account

### Step 1.2: Install Unity Editor

1. Open Unity Hub
2. Go to **Installs** tab
3. Click **Install Editor**
4. Select **Unity 2022.3 LTS**
5. Add modules:
   - ✅ Linux Build Support (if on Windows/Mac)
   - ✅ Windows Build Support (if on Linux/Mac)
   - ✅ WebGL Build Support (for browser demos)
   - ✅ Visual Studio / Visual Studio Code (for scripting)

Installation time: ~15-30 minutes

## Part 2: Create Unity Project

### Step 2.1: New Project

1. In Unity Hub, click **New Project**
2. Select template: **3D (URP)** (Universal Render Pipeline)
   - URP provides better graphics than built-in pipeline
   - Optimized for cross-platform performance
3. Project settings:
   - **Project Name**: `HumanoidVisualization`
   - **Location**: Choose accessible directory
4. Click **Create Project**

First-time project creation: ~5 minutes

### Step 2.2: Verify Project Setup

Unity Editor should open with:
- Scene view (3D viewport)
- Hierarchy panel (GameObjects tree)
- Inspector panel (properties)
- Project panel (assets)

## Part 3: Install Unity Robotics Hub

Unity Robotics Hub provides ROS 2 integration via three packages:
1. **ROS-TCP-Connector**: Communication between Unity and ROS 2
2. **URDF Importer**: Import robot models from ROS
3. **Visualizations**: TF frames, sensor overlays (optional)

### Step 3.1: Install ROS-TCP-Connector

1. Open **Window → Package Manager**
2. Click **+ (plus icon) → Add package from git URL**
3. Enter:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```
4. Click **Add**
5. Wait for installation (~2 minutes)

**Verify**: Check Console for "Package 'ROS TCP Connector' imported successfully"

### Step 3.2: Install URDF Importer

1. Package Manager → **+ → Add package from git URL**
2. Enter:
   ```
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```
3. Click **Add**
4. Wait for installation (~3 minutes)

**Verify**: Menu **Assets → Import Robot from URDF** should appear

### Step 3.3: Install Visualizations (Optional)

1. Package Manager → **+ → Add package from git URL**
2. Enter:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations
   ```
3. Click **Add**

## Part 4: Configure ROS 2 Side

### Step 4.1: Install ROS-TCP-Endpoint

On your ROS 2 machine (Ubuntu):

```bash
# Create workspace
mkdir -p ~/ros2_unity_ws/src
cd ~/ros2_unity_ws/src

# Clone repository
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build
cd ~/ros2_unity_ws
colcon build --packages-select ros_tcp_endpoint

# Source workspace
source install/setup.bash

# Add to ~/.bashrc for persistence
echo "source ~/ros2_unity_ws/install/setup.bash" >> ~/.bashrc
```

### Step 4.2: Launch ROS-TCP-Endpoint Server

```bash
# Start the TCP bridge server
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Expected output:**
```
[INFO] [ros_tcp_endpoint]: ROS-TCP-Endpoint starting on 0.0.0.0:10000
[INFO] [ros_tcp_endpoint]: Waiting for connection...
```

**Leave this running** - it bridges Unity and ROS 2.

## Part 5: Connect Unity to ROS 2

### Step 5.1: Configure Connection

In Unity Editor:

1. Go to **Robotics → ROS Settings**
2. Configure parameters:
   - **ROS IP Address**:
     - `127.0.0.1` (if Unity and ROS on same machine)
     - Or your ROS machine's IP (e.g., `192.168.1.100`)
   - **ROS Port**: `10000` (default)
   - **Protocol**: `ROS2`
3. Click **Test Connection**

**Success**: You should see "Connection successful!"

### Step 5.2: Troubleshooting Connection

**Problem: "Connection failed"**

1. **Firewall blocking?**
   ```bash
   # Ubuntu: Allow port 10000
   sudo ufw allow 10000

   # Windows: Add firewall exception for port 10000
   ```

2. **Wrong IP address?**
   ```bash
   # Find your IP on Linux/Mac:
   ip addr show

   # Find your IP on Windows:
   ipconfig
   ```

3. **ROS-TCP-Endpoint not running?**
   - Check terminal for server logs
   - Restart: `ros2 run ros_tcp_endpoint default_server_endpoint`

4. **Network issues?**
   - Ping ROS machine from Unity machine
   - Ensure both on same network (or VPN/tunnel configured)

## Part 6: Import Your First Robot

### Step 6.1: Prepare URDF

Use the humanoid URDF from Module 1:

```bash
# On ROS machine, copy URDF and meshes to a shared location
cp ~/path/to/humanoid_full.urdf /tmp/
cp -r ~/path/to/meshes /tmp/
```

Transfer to Unity machine (if different from ROS machine):
```bash
# Example: SCP transfer
scp user@ros-machine:/tmp/humanoid_full.urdf ~/Desktop/
scp -r user@ros-machine:/tmp/meshes ~/Desktop/
```

### Step 6.2: Import in Unity

1. In Unity, go to **Assets → Import Robot from URDF**
2. Select your URDF file
3. Import settings:
   - **Axis Type**: **Y Axis** (Unity uses Y-up, ROS uses Z-up)
   - **Mesh Decomposer**: **VHACD** (for collision meshes)
4. Click **Import**

**Result**: Robot appears in Scene view as GameObject hierarchy

### Step 6.3: Test Visualization

1. Press **Play** button (top center)
2. Robot should appear in Game view
3. Use mouse to rotate camera:
   - Right-click + drag: rotate
   - Middle-click + drag: pan
   - Scroll: zoom

## Part 7: Test Communication

### Step 7.1: Publisher Test (ROS → Unity)

**ROS side:**
```bash
# Publish test message
ros2 topic pub /test_topic std_msgs/msg/String "data: 'Hello Unity'" -r 10
```

**Unity side (C# script):**

Create **Assets/Scripts/TestSubscriber.cs**:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class TestSubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>("/test_topic", ReceiveMessage);
    }

    void ReceiveMessage(StringMsg msg)
    {
        Debug.Log($"Received from ROS: {msg.data}");
    }
}
```

Attach script to any GameObject → Press Play → Check Console for messages.

### Step 7.2: Publisher Test (Unity → ROS)

**Unity side (C# script):**

Create **Assets/Scripts/TestPublisher.cs**:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class TestPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private float timer = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>("/unity_test");
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer > 1.0f)
        {
            StringMsg msg = new StringMsg { data = "Hello from Unity!" };
            ros.Publish("/unity_test", msg);
            timer = 0f;
        }
    }
}
```

**ROS side:**
```bash
# Listen for Unity messages
ros2 topic echo /unity_test
```

## Part 8: Common Issues and Solutions

### Issue 1: "Packages not found" when importing

**Solution**: Ensure Package Manager installation completed successfully. Check **Window → Package Manager → In Project** for:
- ROS TCP Connector
- URDF Importer

### Issue 2: Robot imports but looks wrong

**Problem**: Incorrect axis conversion

**Solution**: Re-import URDF with **Axis Type: Y Axis**

### Issue 3: Meshes not loading

**Problem**: Mesh file paths in URDF not found

**Solution**:
1. Check URDF mesh paths (should be `package://` or absolute)
2. Copy meshes to Unity project: `Assets/Robots/meshes/`
3. Update URDF paths or use mesh search feature

### Issue 4: Unity hangs when connecting

**Problem**: Network timeout or firewall

**Solution**:
- Reduce timeout in ROS Settings
- Check firewall rules
- Verify IP address and port

### Issue 5: Messages not appearing

**Problem**: Topic names mismatch or QoS settings

**Solution**:
```bash
# Check topic names match exactly
ros2 topic list

# Check QoS settings in ROS-TCP-Endpoint configuration
```

## Part 9: Next Steps

Once setup is complete:

1. **Import your humanoid**: Use the URDF from Module 1
2. **Sync with Gazebo**: Bridge joint states from Gazebo to Unity
3. **Add UI controls**: Create dashboard with buttons and sliders
4. **VR integration**: Install XR Plugin Management for VR telepresence
5. **Build environment**: Add furniture, lighting, and props

## Additional Resources

**Official Documentation:**
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
- [Unity Manual](https://docs.unity3d.com/Manual/)

**Tutorials:**
- [Pick-and-Place Tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/README.md)
- [URDF Importer Guide](https://github.com/Unity-Technologies/URDF-Importer/blob/main/documentation~/UrdfImporter.md)

**Community:**
- [Unity Robotics Forum](https://forum.unity.com/forums/robotics.617/)
- [ROS Discourse - Unity Tag](https://discourse.ros.org/tag/unity)

## Troubleshooting Checklist

Before asking for help, verify:

- [ ] Unity Editor 2022.3 LTS or newer installed
- [ ] ROS-TCP-Connector package imported successfully
- [ ] URDF Importer package imported successfully
- [ ] ROS-TCP-Endpoint running on ROS 2 machine
- [ ] Firewall allows port 10000
- [ ] Correct IP address configured in Unity ROS Settings
- [ ] Connection test passes in Unity
- [ ] ROS 2 sourced: `source /opt/ros/humble/setup.bash`
- [ ] Workspace sourced: `source ~/ros2_unity_ws/install/setup.bash`

---

**Setup Complete!** You're now ready to visualize humanoid robots in Unity with ROS 2 integration.
