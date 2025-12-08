"""
Module 1 Code Examples Validation Tests

This module validates Python code examples in Module 1 (ROS 2 Foundations)
for syntax correctness, import availability, and basic structural checks.

Tests:
- Python syntax validation (ast.parse)
- Import statement checks
- Basic code structure verification (classes, functions)
- URDF XML validation

Run with:
    pytest tests/code-examples/test_module1.py -v
"""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest


# Get repository root
REPO_ROOT = Path(__file__).parent.parent.parent
CODE_DIR = REPO_ROOT / 'static' / 'code' / 'module1'


def test_code_directory_exists():
    """Test that Module 1 code directory exists."""
    assert CODE_DIR.exists(), f"Code directory not found: {CODE_DIR}"
    assert CODE_DIR.is_dir(), f"Code path is not a directory: {CODE_DIR}"


@pytest.mark.parametrize("filename", [
    "minimal_publisher.py",
    "minimal_subscriber.py",
    "add_two_ints_service.py",
    "add_two_ints_client.py",
    "humanoid_action_server.py"
])
def test_python_files_exist(filename):
    """Test that all required Python example files exist."""
    file_path = CODE_DIR / filename
    assert file_path.exists(), f"Code file not found: {file_path}"


@pytest.mark.parametrize("filename", [
    "simple_humanoid.urdf",
    "humanoid_full.urdf"
])
def test_urdf_files_exist(filename):
    """Test that all required URDF example files exist."""
    file_path = CODE_DIR / filename
    assert file_path.exists(), f"URDF file not found: {file_path}"


@pytest.mark.parametrize("filename", [
    "minimal_publisher.py",
    "minimal_subscriber.py",
    "add_two_ints_service.py",
    "add_two_ints_client.py",
    "humanoid_action_server.py"
])
def test_python_syntax_valid(filename):
    """Test that Python files have valid syntax."""
    file_path = CODE_DIR / filename
    content = file_path.read_text(encoding='utf-8')

    try:
        ast.parse(content)
    except SyntaxError as e:
        pytest.fail(f"Syntax error in {filename}: {e}")


@pytest.mark.parametrize("filename,expected_imports", [
    ("minimal_publisher.py", ["rclpy", "rclpy.node", "std_msgs.msg"]),
    ("minimal_subscriber.py", ["rclpy", "rclpy.node", "std_msgs.msg"]),
    ("add_two_ints_service.py", ["rclpy", "rclpy.node", "example_interfaces.srv"]),
    ("add_two_ints_client.py", ["sys", "rclpy", "rclpy.node", "example_interfaces.srv"]),
    ("humanoid_action_server.py", ["time", "rclpy", "rclpy.action", "rclpy.node"])
])
def test_python_imports_present(filename, expected_imports):
    """Test that Python files import required modules."""
    file_path = CODE_DIR / filename
    content = file_path.read_text(encoding='utf-8')
    tree = ast.parse(content)

    # Extract all import statements
    imports = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.add(alias.name)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imports.add(node.module)

    # Check that expected imports are present
    for expected in expected_imports:
        assert expected in imports, f"Missing import '{expected}' in {filename}"


@pytest.mark.parametrize("filename,expected_class", [
    ("minimal_publisher.py", "MinimalPublisher"),
    ("minimal_subscriber.py", "MinimalSubscriber"),
    ("add_two_ints_service.py", "AdditionService"),
    ("add_two_ints_client.py", "AdditionClient"),
    ("humanoid_action_server.py", "WalkForwardActionServer")
])
def test_python_class_defined(filename, expected_class):
    """Test that Python files define expected classes."""
    file_path = CODE_DIR / filename
    content = file_path.read_text(encoding='utf-8')
    tree = ast.parse(content)

    # Extract class names
    class_names = [node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)]

    assert expected_class in class_names, f"Class '{expected_class}' not found in {filename}"


def test_python_main_function_present():
    """Test that all Python files have a main() function."""
    python_files = [
        "minimal_publisher.py",
        "minimal_subscriber.py",
        "add_two_ints_service.py",
        "add_two_ints_client.py",
        "humanoid_action_server.py"
    ]

    for filename in python_files:
        file_path = CODE_DIR / filename
        content = file_path.read_text(encoding='utf-8')
        tree = ast.parse(content)

        # Extract function names
        function_names = [node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)]

        assert 'main' in function_names, f"main() function not found in {filename}"


@pytest.mark.parametrize("filename", [
    "simple_humanoid.urdf",
    "humanoid_full.urdf"
])
def test_urdf_xml_valid(filename):
    """Test that URDF files are valid XML."""
    file_path = CODE_DIR / filename
    content = file_path.read_text(encoding='utf-8')

    try:
        ET.fromstring(content)
    except ET.ParseError as e:
        pytest.fail(f"XML parse error in {filename}: {e}")


@pytest.mark.parametrize("filename", [
    "simple_humanoid.urdf",
    "humanoid_full.urdf"
])
def test_urdf_has_robot_root(filename):
    """Test that URDF files have <robot> as root element."""
    file_path = CODE_DIR / filename
    content = file_path.read_text(encoding='utf-8')
    root = ET.fromstring(content)

    assert root.tag == 'robot', f"Root element is not <robot> in {filename}"


@pytest.mark.parametrize("filename,expected_links", [
    ("simple_humanoid.urdf", ["base_link", "left_thigh", "left_shin", "left_foot"]),
    ("humanoid_full.urdf", ["base_link", "left_thigh", "left_shin", "left_foot",
                            "right_thigh", "right_shin", "right_foot"])
])
def test_urdf_links_present(filename, expected_links):
    """Test that URDF files define expected links."""
    file_path = CODE_DIR / filename
    content = file_path.read_text(encoding='utf-8')
    root = ET.fromstring(content)

    # Extract link names
    link_names = [link.get('name') for link in root.findall('link')]

    for expected_link in expected_links:
        assert expected_link in link_names, f"Link '{expected_link}' not found in {filename}"


@pytest.mark.parametrize("filename,expected_joints", [
    ("simple_humanoid.urdf", ["left_hip_pitch", "left_knee", "left_ankle"]),
    ("humanoid_full.urdf", ["left_hip_pitch", "left_knee", "left_ankle_pitch",
                            "right_hip_pitch", "right_knee", "right_ankle_pitch"])
])
def test_urdf_joints_present(filename, expected_joints):
    """Test that URDF files define expected joints."""
    file_path = CODE_DIR / filename
    content = file_path.read_text(encoding='utf-8')
    root = ET.fromstring(content)

    # Extract joint names
    joint_names = [joint.get('name') for joint in root.findall('joint')]

    for expected_joint in expected_joints:
        assert expected_joint in joint_names, f"Joint '{expected_joint}' not found in {filename}"


def test_urdf_joint_types_valid():
    """Test that all joints in URDF files have valid types."""
    valid_types = ['revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar']

    urdf_files = ["simple_humanoid.urdf", "humanoid_full.urdf"]

    for filename in urdf_files:
        file_path = CODE_DIR / filename
        content = file_path.read_text(encoding='utf-8')
        root = ET.fromstring(content)

        joints = root.findall('joint')
        for joint in joints:
            joint_type = joint.get('type')
            joint_name = joint.get('name')
            assert joint_type in valid_types, \
                f"Invalid joint type '{joint_type}' for joint '{joint_name}' in {filename}"


def test_python_files_have_docstrings():
    """Test that all Python files have module-level docstrings."""
    python_files = [
        "minimal_publisher.py",
        "minimal_subscriber.py",
        "add_two_ints_service.py",
        "add_two_ints_client.py",
        "humanoid_action_server.py"
    ]

    for filename in python_files:
        file_path = CODE_DIR / filename
        content = file_path.read_text(encoding='utf-8')
        tree = ast.parse(content)

        docstring = ast.get_docstring(tree)
        assert docstring is not None, f"Missing module docstring in {filename}"
        assert len(docstring) > 10, f"Docstring too short in {filename}"


def test_python_files_have_shebang():
    """Test that all Python files have shebang line."""
    python_files = [
        "minimal_publisher.py",
        "minimal_subscriber.py",
        "add_two_ints_service.py",
        "add_two_ints_client.py",
        "humanoid_action_server.py"
    ]

    for filename in python_files:
        file_path = CODE_DIR / filename
        with open(file_path, 'r', encoding='utf-8') as f:
            first_line = f.readline().strip()

        assert first_line.startswith('#!'), f"Missing shebang in {filename}"
        assert 'python' in first_line.lower(), f"Shebang doesn't reference python in {filename}"
