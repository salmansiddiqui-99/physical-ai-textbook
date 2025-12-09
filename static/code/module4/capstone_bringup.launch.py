#!/usr/bin/env python3
"""
Complete Capstone System Launch File

Purpose: Bring up fully autonomous humanoid robot system
Components: Isaac ROS, Nav2, Voice Control, Cognitive Planning, Safety
Usage: ros2 launch humanoid_capstone capstone_bringup.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate complete system launch description"""

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default='warehouse.yaml')
    params_file = LaunchConfiguration('params_file', default='nav2_bipedal_params.yaml')

    # API keys from environment
    openai_key = os.environ.get('OPENAI_API_KEY', '')
    if not openai_key:
        print("WARNING: OPENAI_API_KEY not set in environment")

    return LaunchDescription([
        # ========== Launch Arguments ==========
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Isaac Sim) clock'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='warehouse.yaml',
            description='Map file for localization'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='nav2_bipedal_params.yaml',
            description='Nav2 parameters for bipedal robot'
        ),

        # ========== 1. Isaac ROS Visual SLAM ==========
        # Provides localization using stereo cameras + IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('isaac_ros_visual_slam'),
                    'launch',
                    'isaac_ros_visual_slam.launch.py'
                ])
            ]),
            launch_arguments={
                'enable_imu_fusion': 'true',
                'enable_ground_constraint_in_odometry': 'false',  # Allow full 6DOF
                'enable_localization_n_mapping': 'true',
            }.items()
        ),

        # ========== 2. Isaac ROS Stereo Depth ==========
        # GPU-accelerated depth perception for obstacle avoidance
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('isaac_ros_ess'),
                    'launch',
                    'isaac_ros_ess_stereo.launch.py'
                ])
            ]),
            launch_arguments={
                'engine_file_path': '/tmp/ess_engine.plan',
                'threshold': '0.35',  # Confidence threshold
            }.items()
        ),

        # ========== 3. Nav2 Navigation Stack ==========
        # Path planning and control for bipedal locomotion
        TimerAction(
            period=5.0,  # Wait for SLAM to initialize
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('nav2_bringup'),
                            'launch',
                            'bringup_launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'map': map_file,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'autostart': 'true',
                    }.items()
                )
            ]
        ),

        # ========== 4. Voice Control System ==========
        # Whisper speech recognition + command parsing
        Node(
            package='voice_control',
            executable='whisper_voice_node',
            name='whisper_voice_node',
            output='screen',
            parameters=[{
                'model_size': 'base',  # Options: tiny, base, small, medium, large
                'language': 'en',
                'sample_rate': 16000,
                'use_sim_time': use_sim_time,
            }]
        ),

        Node(
            package='voice_control',
            executable='command_parser_node',
            name='command_parser_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),

        # ========== 5. Cognitive Planning System ==========
        # LLM-based task decomposition (GPT-4 or Claude)
        Node(
            package='cognitive_control',
            executable='llm_task_planner_node',
            name='llm_task_planner_node',
            output='screen',
            parameters=[{
                'openai_api_key': openai_key,
                'model': 'gpt-4',
                'provider': 'openai',
                'use_sim_time': use_sim_time,
            }],
            # Remap topics
            remappings=[
                ('/voice/command', '/voice/transcription'),
                ('/cognitive/action_plan', '/executor/plan')
            ]
        ),

        # ========== 6. Action Graph Executor ==========
        # Execute LLM plans via ROS 2 action clients
        Node(
            package='action_execution',
            executable='action_graph_executor',
            name='action_graph_executor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('/cognitive/action_plan', '/executor/plan')
            ]
        ),

        # ========== 7. Safety Monitor ==========
        # Emergency stop, collision detection, battery monitoring
        Node(
            package='safety',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen',
            parameters=[{
                'min_obstacle_distance': 0.3,  # meters
                'min_battery_percent': 15.0,
                'use_sim_time': use_sim_time,
            }]
        ),

        # ========== 8. State Manager ==========
        # Track robot state: IDLE, LISTENING, PLANNING, EXECUTING, ERROR
        Node(
            package='state_management',
            executable='state_manager_node',
            name='state_manager_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),

        # ========== 9. Visualization (Optional) ==========
        # RViz2 with custom config for humanoid monitoring
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('humanoid_capstone'),
                'rviz',
                'capstone_view.rviz'
            ])],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            condition=lambda context: LaunchConfiguration('use_rviz', default='true')
        ),

        # ========== 10. Diagnostics ==========
        # Monitor system health
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),

        # ========== 11. TF Static Transforms ==========
        # Publish static transforms for sensors
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_left',
            arguments=['0.05', '0.06', '1.5', '0', '0', '0', 'base_link', 'camera_left']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_right',
            arguments=['0.05', '-0.06', '1.5', '0', '0', '0', 'base_link', 'camera_right']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0.0', '0.0', '0.8', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # ========== Startup Message ==========
        ExecuteProcess(
            cmd=['echo', '========================================'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['echo', 'Autonomous Humanoid Robot System Starting...'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['echo', 'Modules: Isaac ROS + Nav2 + Voice + LLM + Safety'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['echo', '========================================'],
            output='screen'
        ),
    ])


# Additional helper function for deployment
def generate_simulation_launch_description():
    """
    Alternative launch description for Isaac Sim integration

    Usage:
        # Terminal 1: Start Isaac Sim
        ./isaac-sim.sh

        # Terminal 2: Load humanoid scene
        python load_humanoid_scene.py

        # Terminal 3: Launch this file
        ros2 launch humanoid_capstone capstone_bringup.launch.py
    """
    # Would include Isaac Sim bridge configuration
    pass


if __name__ == '__main__':
    generate_launch_description()
