"""
Test suite for Module 2 code examples.

This test file validates:
- Python launch file syntax
- Gazebo world file (SDF) structure
- URDF/Xacro sensor plugin configurations
- Markdown documentation formatting

Run with: pytest tests/code-examples/test_module2.py -v
"""

import os
import sys
import ast
import xml.etree.ElementTree as ET
import pytest
from pathlib import Path

# Path to Module 2 code examples
CODE_DIR = Path(__file__).parent.parent.parent / "static" / "code" / "module2"


# ===== Python Launch File Tests =====

@pytest.mark.parametrize("filename", [
    "spawn_humanoid.launch.py",
])
def test_python_syntax_valid(filename):
    """Test that Python launch files have valid syntax."""
    file_path = CODE_DIR / filename
    assert file_path.exists(), f"File {filename} not found"

    content = file_path.read_text(encoding='utf-8')
    try:
        ast.parse(content)
    except SyntaxError as e:
        pytest.fail(f"Syntax error in {filename}: {e}")


def test_launch_file_has_launch_description():
    """Test that launch file defines generate_launch_description function."""
    file_path = CODE_DIR / "spawn_humanoid.launch.py"
    content = file_path.read_text(encoding='utf-8')

    assert "def generate_launch_description" in content, \
        "Launch file must define generate_launch_description()"
    assert "return LaunchDescription" in content, \
        "Launch file must return LaunchDescription object"


def test_launch_file_imports():
    """Test that launch file has necessary ROS 2 imports."""
    file_path = CODE_DIR / "spawn_humanoid.launch.py"
    content = file_path.read_text(encoding='utf-8')

    required_imports = [
        "from launch import LaunchDescription",
        "from launch_ros.actions import Node",
    ]

    for import_stmt in required_imports:
        assert import_stmt in content, f"Missing import: {import_stmt}"


def test_launch_file_has_arguments():
    """Test that launch file defines expected arguments."""
    file_path = CODE_DIR / "spawn_humanoid.launch.py"
    content = file_path.read_text(encoding='utf-8')

    expected_args = ["world", "robot_name", "x", "y", "z"]

    for arg in expected_args:
        assert f"'{arg}'" in content or f'"{arg}"' in content, \
            f"Launch file should define argument: {arg}"


# ===== Gazebo World File Tests =====

@pytest.mark.parametrize("filename", [
    "simple_world.world",
    "humanoid_world.world",
])
def test_world_file_valid_xml(filename):
    """Test that world files are valid XML."""
    file_path = CODE_DIR / filename
    assert file_path.exists(), f"World file {filename} not found"

    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        assert root.tag == "sdf", f"Root element should be <sdf>, got <{root.tag}>"
    except ET.ParseError as e:
        pytest.fail(f"XML parse error in {filename}: {e}")


@pytest.mark.parametrize("filename", [
    "simple_world.world",
    "humanoid_world.world",
])
def test_world_file_has_physics(filename):
    """Test that world files define physics configuration."""
    file_path = CODE_DIR / filename
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Find <world><physics> element
    world = root.find("world")
    assert world is not None, f"{filename} must contain <world> element"

    physics = world.find("physics")
    assert physics is not None, f"{filename} must contain <physics> element"

    # Check for gravity
    gravity = physics.find("gravity")
    assert gravity is not None, f"{filename} physics must define <gravity>"


@pytest.mark.parametrize("filename", [
    "simple_world.world",
    "humanoid_world.world",
])
def test_world_file_has_ground_plane(filename):
    """Test that world files have a ground plane model."""
    file_path = CODE_DIR / filename
    tree = ET.parse(file_path)
    root = tree.getroot()

    world = root.find("world")
    models = world.findall("model")

    ground_plane_found = any(
        model.get("name") == "ground_plane" for model in models
    )

    assert ground_plane_found, f"{filename} should have ground_plane model"


def test_humanoid_world_has_obstacles():
    """Test that humanoid_world.world contains test obstacles."""
    file_path = CODE_DIR / "humanoid_world.world"
    tree = ET.parse(file_path)
    root = tree.getroot()

    world = root.find("world")
    models = world.findall("model")

    # Should have more than just ground plane
    assert len(models) > 1, "humanoid_world should contain obstacles"

    # Check for expected obstacle names
    model_names = [model.get("name") for model in models]
    expected_obstacles = ["test_box_1", "cylinder_obstacle"]

    for obstacle in expected_obstacles:
        assert obstacle in model_names, \
            f"humanoid_world should contain {obstacle}"


# ===== URDF/Xacro Sensor Plugin Tests =====

@pytest.mark.parametrize("filename", [
    "lidar_example.urdf.xacro",
    "depth_camera.urdf.xacro",
    "imu_sensor.urdf.xacro",
])
def test_xacro_file_valid_xml(filename):
    """Test that xacro files are valid XML."""
    file_path = CODE_DIR / filename
    assert file_path.exists(), f"Xacro file {filename} not found"

    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        assert root.tag == "robot", f"Root should be <robot>, got <{root.tag}>"
    except ET.ParseError as e:
        pytest.fail(f"XML parse error in {filename}: {e}")


@pytest.mark.parametrize("filename", [
    "lidar_example.urdf.xacro",
    "depth_camera.urdf.xacro",
    "imu_sensor.urdf.xacro",
])
def test_xacro_has_namespace(filename):
    """Test that xacro files define xacro namespace."""
    file_path = CODE_DIR / filename
    content = file_path.read_text(encoding='utf-8')

    # Check for xacro namespace in file content
    # Valid forms: xmlns:xacro="..." or <xacro:macro> usage
    xacro_ns_found = (
        'xmlns:xacro=' in content or
        '<xacro:macro' in content or
        'http://www.ros.org/wiki/xacro' in content
    )

    assert xacro_ns_found, f"{filename} should define or use xacro namespace"


def test_lidar_xacro_has_macro():
    """Test that lidar xacro defines sensor macro."""
    file_path = CODE_DIR / "lidar_example.urdf.xacro"
    content = file_path.read_text(encoding='utf-8')

    assert "<xacro:macro" in content, "Should define xacro macro"
    assert "name=\"lidar_sensor\"" in content or "name='lidar_sensor'" in content, \
        "Should define lidar_sensor macro"
    assert "params=" in content, "Macro should have parameters"


def test_depth_camera_xacro_has_optical_frame():
    """Test that depth camera defines optical frame (ROS convention)."""
    file_path = CODE_DIR / "depth_camera.urdf.xacro"
    content = file_path.read_text(encoding='utf-8')

    assert "optical_frame" in content, \
        "Depth camera should define optical frame"
    assert "rpy=\"-1.5707963 0 -1.5707963\"" in content or \
           "rpy='-1.5707963 0 -1.5707963'" in content, \
        "Optical frame should have standard ROS camera rotation"


def test_imu_xacro_has_noise_config():
    """Test that IMU xacro defines realistic noise parameters."""
    file_path = CODE_DIR / "imu_sensor.urdf.xacro"
    content = file_path.read_text(encoding='utf-8')

    assert "<noise" in content, "IMU should define noise model"
    assert "gaussian" in content, "IMU should use Gaussian noise"
    assert "stddev" in content, "IMU should define standard deviation"
    assert "bias_mean" in content, "IMU should define bias"


@pytest.mark.parametrize("filename,plugin_name", [
    ("lidar_example.urdf.xacro", "libgazebo_ros_ray_sensor.so"),
    ("depth_camera.urdf.xacro", "libgazebo_ros_camera.so"),
    ("imu_sensor.urdf.xacro", "libgazebo_ros_imu_sensor.so"),
])
def test_sensor_has_gazebo_plugin(filename, plugin_name):
    """Test that sensor xacro files reference correct Gazebo plugin."""
    file_path = CODE_DIR / filename
    content = file_path.read_text(encoding='utf-8')

    assert "<plugin" in content, f"{filename} should define Gazebo plugin"
    assert plugin_name in content, \
        f"{filename} should reference {plugin_name}"


# ===== Sensor Configuration Tests =====

def test_lidar_has_reasonable_range():
    """Test that LiDAR configuration has reasonable min/max range."""
    file_path = CODE_DIR / "lidar_example.urdf.xacro"
    tree = ET.parse(file_path)

    # Find <range> element
    content = file_path.read_text()
    assert "<range>" in content, "LiDAR should define range"
    assert "<min>" in content and "<max>" in content, \
        "LiDAR range should have min and max"

    # Check values are reasonable (min < 1m, max > 1m)
    import re
    min_match = re.search(r"<min>([\d.]+)</min>", content)
    max_match = re.search(r"<max>([\d.]+)</max>", content)

    if min_match and max_match:
        min_val = float(min_match.group(1))
        max_val = float(max_match.group(1))
        assert 0.01 <= min_val <= 1.0, f"LiDAR min range unusual: {min_val}"
        assert 1.0 <= max_val <= 200.0, f"LiDAR max range unusual: {max_val}"


def test_depth_camera_has_reasonable_fov():
    """Test that depth camera FOV is within reasonable range."""
    file_path = CODE_DIR / "depth_camera.urdf.xacro"
    content = file_path.read_text()

    assert "<horizontal_fov>" in content, \
        "Depth camera should define horizontal FOV"

    import re
    fov_match = re.search(r"<horizontal_fov>([\d.]+)</horizontal_fov>", content)

    if fov_match:
        fov = float(fov_match.group(1))
        # FOV in radians: 30° to 180° (0.52 to 3.14 rad)
        assert 0.5 <= fov <= 3.2, f"Camera FOV unusual: {fov} rad"


def test_imu_update_rate():
    """Test that IMU update rate is appropriate for control."""
    file_path = CODE_DIR / "imu_sensor.urdf.xacro"
    content = file_path.read_text()

    assert "<update_rate>" in content, "IMU should define update rate"

    import re
    rate_match = re.search(r"<update_rate>([\d.]+)</update_rate>", content)

    if rate_match:
        rate = float(rate_match.group(1))
        # IMU should be high-rate (typically 50-500 Hz)
        assert 50 <= rate <= 1000, f"IMU update rate unusual: {rate} Hz"


# ===== Documentation File Test =====

def test_unity_setup_exists():
    """Test that Unity setup documentation exists."""
    file_path = CODE_DIR / "unity_setup.md"
    assert file_path.exists(), "unity_setup.md should exist"

    content = file_path.read_text(encoding='utf-8')
    assert len(content) > 1000, "unity_setup.md should have substantial content"


def test_unity_setup_has_sections():
    """Test that Unity setup guide has expected sections."""
    file_path = CODE_DIR / "unity_setup.md"
    content = file_path.read_text(encoding='utf-8')

    expected_sections = [
        "Prerequisites",
        "Install Unity",
        "Create Unity Project",
        "Unity Robotics Hub",
        "ROS 2",
        "Connect Unity to ROS",
    ]

    for section in expected_sections:
        assert section in content, \
            f"unity_setup.md should have section about {section}"


# ===== Integration Tests =====

def test_all_required_files_exist():
    """Test that all Module 2 code files exist."""
    required_files = [
        "simple_world.world",
        "humanoid_world.world",
        "spawn_humanoid.launch.py",
        "lidar_example.urdf.xacro",
        "depth_camera.urdf.xacro",
        "imu_sensor.urdf.xacro",
        "unity_setup.md",
    ]

    for filename in required_files:
        file_path = CODE_DIR / filename
        assert file_path.exists(), f"Required file missing: {filename}"


def test_file_sizes_reasonable():
    """Test that files have reasonable sizes (not empty, not too large)."""
    files_to_check = [
        ("simple_world.world", 500, 10000),
        ("humanoid_world.world", 2000, 50000),
        ("spawn_humanoid.launch.py", 1000, 20000),
        ("lidar_example.urdf.xacro", 1000, 30000),
        ("depth_camera.urdf.xacro", 1000, 30000),
        ("imu_sensor.urdf.xacro", 1000, 30000),
        ("unity_setup.md", 2000, 50000),
    ]

    for filename, min_size, max_size in files_to_check:
        file_path = CODE_DIR / filename
        file_size = file_path.stat().st_size
        assert min_size <= file_size <= max_size, \
            f"{filename} size unusual: {file_size} bytes (expected {min_size}-{max_size})"


def test_no_absolute_paths_in_configs():
    """Test that configuration files don't contain hardcoded absolute paths."""
    files_to_check = [
        "spawn_humanoid.launch.py",
        "lidar_example.urdf.xacro",
        "depth_camera.urdf.xacro",
        "imu_sensor.urdf.xacro",
    ]

    for filename in files_to_check:
        file_path = CODE_DIR / filename
        content = file_path.read_text(encoding='utf-8')

        # Check for common absolute path patterns
        problematic_patterns = [
            "/home/username",
            "C:\\Users\\",
            "/Users/username",
        ]

        for pattern in problematic_patterns:
            assert pattern not in content, \
                f"{filename} should not contain hardcoded path: {pattern}"


# ===== Summary Test =====

def test_module2_complete():
    """Summary test to verify Module 2 code examples are complete."""
    print("\n=== Module 2 Code Examples Summary ===")
    print(f"Location: {CODE_DIR}")

    files = list(CODE_DIR.glob("*"))
    print(f"Total files: {len(files)}")

    for file in sorted(files):
        size_kb = file.stat().st_size / 1024
        print(f"  - {file.name} ({size_kb:.1f} KB)")

    assert len(files) >= 7, "Should have at least 7 Module 2 files"
    print("✓ Module 2 code examples complete!")


if __name__ == "__main__":
    # Run tests when executed directly
    pytest.main([__file__, "-v", "--tb=short"])
