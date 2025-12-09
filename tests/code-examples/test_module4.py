#!/usr/bin/env python3
"""
Test suite for Module 4 code examples

Purpose: Validate all VLA and capstone code examples
"""

import pytest
import os
import ast
import re
from pathlib import Path

# Path to code examples
MODULE4_CODE_DIR = Path(__file__).parent.parent.parent / 'static' / 'code' / 'module4'
MODULE4_IMG_DIR = Path(__file__).parent.parent.parent / 'static' / 'img' / 'module4'


class TestPythonScripts:
    """Test all Python scripts in Module 4"""

    def test_whisper_voice_node_exists(self):
        """Test that whisper_voice_node.py exists"""
        file_path = MODULE4_CODE_DIR / 'whisper_voice_node.py'
        assert file_path.exists(), f"File not found: {file_path}"

    def test_whisper_voice_node_syntax(self):
        """Test Python syntax in whisper_voice_node.py"""
        file_path = MODULE4_CODE_DIR / 'whisper_voice_node.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            ast.parse(code)  # Will raise SyntaxError if invalid

    def test_whisper_voice_node_ros2_imports(self):
        """Test that whisper_voice_node.py imports ROS 2 correctly"""
        file_path = MODULE4_CODE_DIR / 'whisper_voice_node.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            assert 'import rclpy' in code
            assert 'from rclpy.node import Node' in code
            assert 'import whisper' in code

    def test_command_parser_exists(self):
        """Test that command_parser.py exists"""
        file_path = MODULE4_CODE_DIR / 'command_parser.py'
        assert file_path.exists(), f"File not found: {file_path}"

    def test_command_parser_syntax(self):
        """Test Python syntax in command_parser.py"""
        file_path = MODULE4_CODE_DIR / 'command_parser.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            ast.parse(code)

    def test_command_parser_patterns(self):
        """Test that command_parser.py has intent patterns"""
        file_path = MODULE4_CODE_DIR / 'command_parser.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            # Should have NAVIGATE, ROTATE, GRASP intents
            assert 'NAVIGATE' in code
            assert 'ROTATE' in code
            assert 'GRASP' in code
            # Should use regex
            assert 'import re' in code

    def test_llm_task_planner_exists(self):
        """Test that llm_task_planner.py exists"""
        file_path = MODULE4_CODE_DIR / 'llm_task_planner.py'
        assert file_path.exists(), f"File not found: {file_path}"

    def test_llm_task_planner_syntax(self):
        """Test Python syntax in llm_task_planner.py"""
        file_path = MODULE4_CODE_DIR / 'llm_task_planner.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            ast.parse(code)

    def test_llm_task_planner_providers(self):
        """Test that llm_task_planner.py supports both OpenAI and Anthropic"""
        file_path = MODULE4_CODE_DIR / 'llm_task_planner.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            assert 'openai' in code
            assert 'anthropic' in code
            assert 'def _plan_openai' in code or 'def plan_openai' in code
            assert 'def _plan_anthropic' in code or 'def plan_anthropic' in code

    def test_action_graph_executor_exists(self):
        """Test that action_graph_executor.py exists"""
        file_path = MODULE4_CODE_DIR / 'action_graph_executor.py'
        assert file_path.exists(), f"File not found: {file_path}"

    def test_action_graph_executor_syntax(self):
        """Test Python syntax in action_graph_executor.py"""
        file_path = MODULE4_CODE_DIR / 'action_graph_executor.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            ast.parse(code)

    def test_action_graph_executor_actions(self):
        """Test that action_graph_executor.py implements required actions"""
        file_path = MODULE4_CODE_DIR / 'action_graph_executor.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            # Should implement navigation, grasp, place actions
            assert 'navigate' in code.lower()
            assert 'grasp' in code.lower()
            assert 'place' in code.lower() or 'drop' in code.lower()

    def test_safety_monitor_exists(self):
        """Test that safety_monitor.py exists"""
        file_path = MODULE4_CODE_DIR / 'safety_monitor.py'
        assert file_path.exists(), f"File not found: {file_path}"

    def test_safety_monitor_syntax(self):
        """Test Python syntax in safety_monitor.py"""
        file_path = MODULE4_CODE_DIR / 'safety_monitor.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            ast.parse(code)

    def test_safety_monitor_features(self):
        """Test that safety_monitor.py has required safety features"""
        file_path = MODULE4_CODE_DIR / 'safety_monitor.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            # Should monitor battery, obstacles, emergency stop
            assert 'battery' in code.lower()
            assert 'obstacle' in code.lower()
            assert 'emergency' in code.lower() or 'estop' in code.lower()


class TestLaunchFiles:
    """Test launch files"""

    def test_capstone_launch_exists(self):
        """Test that capstone_bringup.launch.py exists"""
        file_path = MODULE4_CODE_DIR / 'capstone_bringup.launch.py'
        assert file_path.exists(), f"File not found: {file_path}"

    def test_capstone_launch_syntax(self):
        """Test Python syntax in capstone_bringup.launch.py"""
        file_path = MODULE4_CODE_DIR / 'capstone_bringup.launch.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            ast.parse(code)

    def test_capstone_launch_includes_all_modules(self):
        """Test that capstone launch file includes all required components"""
        file_path = MODULE4_CODE_DIR / 'capstone_bringup.launch.py'
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
            # Should include Isaac ROS, Nav2, Voice, Cognitive Planning, Safety
            assert 'isaac_ros' in code.lower()
            assert 'nav2' in code.lower()
            assert 'voice' in code.lower() or 'whisper' in code.lower()
            assert 'cognitive' in code.lower() or 'llm' in code.lower() or 'planner' in code.lower()
            assert 'safety' in code.lower()


class TestDiagrams:
    """Test that all required diagrams exist"""

    def test_vla_architecture_diagram_exists(self):
        """Test that vla-architecture.svg exists"""
        file_path = MODULE4_IMG_DIR / 'vla-architecture.svg'
        assert file_path.exists(), f"Diagram not found: {file_path}"

    def test_vla_architecture_is_svg(self):
        """Test that vla-architecture.svg is valid SVG"""
        file_path = MODULE4_IMG_DIR / 'vla-architecture.svg'
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            assert '<svg' in content
            assert '</svg>' in content

    def test_llm_planning_flow_exists(self):
        """Test that llm-planning-flow.svg exists"""
        file_path = MODULE4_IMG_DIR / 'llm-planning-flow.svg'
        assert file_path.exists(), f"Diagram not found: {file_path}"

    def test_llm_planning_flow_is_svg(self):
        """Test that llm-planning-flow.svg is valid SVG"""
        file_path = MODULE4_IMG_DIR / 'llm-planning-flow.svg'
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            assert '<svg' in content
            assert '</svg>' in content

    def test_capstone_integration_exists(self):
        """Test that capstone-integration.svg exists"""
        file_path = MODULE4_IMG_DIR / 'capstone-integration.svg'
        assert file_path.exists(), f"Diagram not found: {file_path}"

    def test_capstone_integration_is_svg(self):
        """Test that capstone-integration.svg is valid SVG"""
        file_path = MODULE4_IMG_DIR / 'capstone-integration.svg'
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            assert '<svg' in content
            assert '</svg>' in content


class TestCodeQuality:
    """Test code quality aspects"""

    def test_all_python_files_have_docstrings(self):
        """Test that all Python files have module docstrings"""
        python_files = list(MODULE4_CODE_DIR.glob('*.py'))
        assert len(python_files) > 0, "No Python files found"

        for file_path in python_files:
            if file_path.name.startswith('__'):
                continue
            with open(file_path, 'r', encoding='utf-8') as f:
                code = f.read()
                # Should have a docstring (triple quotes near the top)
                assert '"""' in code[:500], f"No docstring in {file_path.name}"

    def test_all_python_files_have_main_or_class(self):
        """Test that all Python files have main() or a class"""
        python_files = list(MODULE4_CODE_DIR.glob('*.py'))

        for file_path in python_files:
            if file_path.name.startswith('__'):
                continue
            with open(file_path, 'r', encoding='utf-8') as f:
                code = f.read()
                # Should have main() or a class definition
                has_main = "if __name__ == '__main__'" in code or 'def main(' in code
                has_class = 'class ' in code
                assert has_main or has_class, f"No main() or class in {file_path.name}"

    def test_ros2_nodes_have_proper_structure(self):
        """Test that ROS 2 nodes follow proper structure"""
        ros2_files = [
            'whisper_voice_node.py',
            'action_graph_executor.py',
            'safety_monitor.py'
        ]

        for filename in ros2_files:
            file_path = MODULE4_CODE_DIR / filename
            if not file_path.exists():
                continue

            with open(file_path, 'r', encoding='utf-8') as f:
                code = f.read()
                # Should inherit from Node
                assert 'Node' in code
                # Should have rclpy.init and rclpy.spin
                assert 'rclpy.init' in code
                assert 'rclpy.spin' in code or 'spin(' in code


class TestIntegration:
    """Test integration between components"""

    def test_action_executor_can_handle_llm_output(self):
        """Test that action executor expects JSON from LLM planner"""
        executor_path = MODULE4_CODE_DIR / 'action_graph_executor.py'
        planner_path = MODULE4_CODE_DIR / 'llm_task_planner.py'

        if not (executor_path.exists() and planner_path.exists()):
            pytest.skip("Required files not found")

        with open(executor_path, 'r', encoding='utf-8') as f:
            executor_code = f.read()
        with open(planner_path, 'r', encoding='utf-8') as f:
            planner_code = f.read()

        # Both should use JSON
        assert 'json' in executor_code.lower()
        assert 'json' in planner_code.lower()

    def test_safety_monitor_publishes_emergency_stop(self):
        """Test that safety monitor can stop the robot"""
        file_path = MODULE4_CODE_DIR / 'safety_monitor.py'

        if not file_path.exists():
            pytest.skip("safety_monitor.py not found")

        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()

        # Should publish Twist or Bool for emergency stop
        assert 'Twist' in code or 'Bool' in code
        assert 'publish' in code.lower()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
