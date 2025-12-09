"""
Test suite for Module 3 code examples (Isaac Sim, Isaac ROS, Nav2)

Purpose: Validate Python scripts, YAML configurations, and USD files
Environment: pytest
Usage: pytest tests/code-examples/test_module3.py -v
"""

import pytest
import os
import yaml
import ast
from pathlib import Path


# Path to Module 3 code directory
MODULE3_CODE = Path("static/code/module3")


class TestUSDFiles:
    """Test USD scene files for Isaac Sim"""

    def test_simple_scene_exists(self):
        """Verify simple_scene.usd exists"""
        assert (MODULE3_CODE / "simple_scene.usd").exists()

    def test_humanoid_perception_exists(self):
        """Verify humanoid_perception.usd exists"""
        assert (MODULE3_CODE / "humanoid_perception.usd").exists()

    def test_usd_syntax(self):
        """Validate basic USD syntax (usda format)"""
        for usd_file in MODULE3_CODE.glob("*.usd"):
            with open(usd_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check for USD header
            assert '#usda' in content, f"{usd_file.name} missing #usda header"

            # Check for key USD elements
            assert 'def ' in content, f"{usd_file.name} missing def statements"
            assert 'defaultPrim' in content or 'def Xform' in content, \
                f"{usd_file.name} missing scene structure"

    def test_usd_has_physics(self):
        """Verify USD files include physics configuration"""
        for usd_file in MODULE3_CODE.glob("*.usd"):
            with open(usd_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check for physics scene
            assert 'PhysicsScene' in content or 'physics:' in content, \
                f"{usd_file.name} missing physics configuration"


class TestPythonScripts:
    """Test Python scripts for Isaac ROS"""

    def test_depth_pipeline_syntax(self):
        """Validate depth_pipeline.py Python syntax"""
        script_path = MODULE3_CODE / "depth_pipeline.py"
        assert script_path.exists(), "depth_pipeline.py not found"

        with open(script_path, 'r', encoding='utf-8') as f:
            code = f.read()

        # Parse Python AST (will raise SyntaxError if invalid)
        ast.parse(code)

    def test_bipedal_planner_syntax(self):
        """Validate bipedal_planner.py Python syntax"""
        script_path = MODULE3_CODE / "bipedal_planner.py"
        assert script_path.exists(), "bipedal_planner.py not found"

        with open(script_path, 'r', encoding='utf-8') as f:
            code = f.read()

        ast.parse(code)

    def test_python_imports(self):
        """Verify Python scripts have correct ROS 2 imports"""
        for py_file in MODULE3_CODE.glob("*.py"):
            with open(py_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check for rclpy import
            assert 'import rclpy' in content, f"{py_file.name} missing rclpy import"

            # Check for Node class
            assert 'from rclpy.node import Node' in content or 'Node' in content, \
                f"{py_file.name} missing Node import"

    def test_python_has_main(self):
        """Verify Python scripts have main() function"""
        for py_file in MODULE3_CODE.glob("*.py"):
            with open(py_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check for main function
            assert 'def main(' in content, f"{py_file.name} missing main() function"

            # Check for if __name__ == '__main__'
            assert "if __name__ == '__main__':" in content, \
                f"{py_file.name} missing __main__ guard"


class TestYAMLConfigurations:
    """Test YAML configuration files"""

    def test_vslam_config_valid(self):
        """Validate vslam_config.yaml structure"""
        config_path = MODULE3_CODE / "vslam_config.yaml"
        assert config_path.exists(), "vslam_config.yaml not found"

        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        # Check for visual_slam node config
        assert 'visual_slam' in config, "Missing visual_slam configuration"
        assert 'ros__parameters' in config['visual_slam'], \
            "Missing ros__parameters in visual_slam"

        # Check key parameters
        params = config['visual_slam']['ros__parameters']
        assert 'enable_imu_fusion' in params, "Missing enable_imu_fusion parameter"
        assert 'map_frame' in params, "Missing map_frame parameter"
        assert 'odom_frame' in params, "Missing odom_frame parameter"

    def test_nav2_params_valid(self):
        """Validate nav2_params_bipedal.yaml structure"""
        config_path = MODULE3_CODE / "nav2_params_bipedal.yaml"
        assert config_path.exists(), "nav2_params_bipedal.yaml not found"

        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        # Check for key Nav2 servers
        required_servers = [
            'bt_navigator',
            'controller_server',
            'planner_server',
            'local_costmap',
            'global_costmap'
        ]

        for server in required_servers:
            assert server in config, f"Missing {server} configuration"

            # Handle nested structure for costmaps (e.g., local_costmap/local_costmap)
            if server in ['local_costmap', 'global_costmap']:
                assert server in config[server], f"Missing nested {server} structure"
                assert 'ros__parameters' in config[server][server], \
                    f"Missing ros__parameters in {server}"
            else:
                assert 'ros__parameters' in config[server], \
                    f"Missing ros__parameters in {server}"

    def test_nav2_bipedal_constraints(self):
        """Verify bipedal-specific parameters in Nav2 config"""
        config_path = MODULE3_CODE / "nav2_params_bipedal.yaml"

        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        # Check controller parameters for bipedal constraints
        controller_params = config['controller_server']['ros__parameters']['FollowPath']

        # Verify no lateral motion (Y velocity = 0)
        assert controller_params['max_vel_y'] == 0.0, \
            "Bipedal robot should not have lateral motion"
        assert controller_params['min_vel_y'] == 0.0

        # Verify slow turning
        assert controller_params['max_vel_theta'] <= 1.0, \
            "Bipedal turning should be slow (≤1.0 rad/s)"

        # Verify gentle acceleration
        assert controller_params['acc_lim_x'] <= 0.5, \
            "Bipedal acceleration should be gentle (≤0.5 m/s²)"

    def test_nav2_costmap_3d(self):
        """Verify 3D voxel layer for humanoid height detection"""
        config_path = MODULE3_CODE / "nav2_params_bipedal.yaml"

        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        # Check local costmap has voxel layer
        local_costmap = config['local_costmap']['local_costmap']['ros__parameters']
        assert 'voxel_layer' in local_costmap['plugins'], \
            "Local costmap should have voxel_layer for 3D obstacles"

        # Check voxel layer parameters
        voxel_params = local_costmap['voxel_layer']
        assert voxel_params['enabled'] is True
        assert 'z_voxels' in voxel_params, "Missing z_voxels (vertical resolution)"


class TestDiagrams:
    """Test that diagram files exist"""

    def test_isaac_sim_architecture_diagram(self):
        """Verify isaac-sim-architecture.svg exists"""
        diagram = Path("static/img/module3/isaac-sim-architecture.svg")
        assert diagram.exists(), "isaac-sim-architecture.svg not found"

    def test_vslam_pipeline_diagram(self):
        """Verify vslam-pipeline.svg exists"""
        diagram = Path("static/img/module3/vslam-pipeline.svg")
        assert diagram.exists(), "vslam-pipeline.svg not found"

    def test_nav2_bipedal_diagram(self):
        """Verify nav2-bipedal.svg exists"""
        diagram = Path("static/img/module3/nav2-bipedal.svg")
        assert diagram.exists(), "nav2-bipedal.svg not found"


class TestCodeQuality:
    """Test code quality and best practices"""

    def test_python_has_docstrings(self):
        """Verify Python files have module-level docstrings"""
        for py_file in MODULE3_CODE.glob("*.py"):
            with open(py_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check for triple-quoted docstring in first 10 lines
            lines = content.split('\n')[:10]
            docstring_found = any('"""' in line or "'''" in line for line in lines)
            assert docstring_found, f"{py_file.name} missing module docstring"

    def test_yaml_comments(self):
        """Verify YAML files have explanatory comments"""
        for yaml_file in MODULE3_CODE.glob("*.yaml"):
            with open(yaml_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check for at least 5 comment lines
            comment_lines = [line for line in content.split('\n') if line.strip().startswith('#')]
            assert len(comment_lines) >= 5, \
                f"{yaml_file.name} should have more comments (found {len(comment_lines)})"

    def test_usd_comments(self):
        """Verify USD files have descriptive comments"""
        for usd_file in MODULE3_CODE.glob("*.usd"):
            with open(usd_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check for doc attribute
            assert 'doc = ' in content or '(doc = ' in content, \
                f"{usd_file.name} should have descriptive doc attributes"


# Run all tests
if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
