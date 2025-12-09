#!/usr/bin/env python3
"""
Launch file to spawn a humanoid robot in Gazebo.

This launch file:
1. Starts Gazebo with a specified world file
2. Spawns a humanoid robot from URDF
3. Sets up robot_state_publisher for TF transforms
4. Configures initial robot pose

Usage:
    ros2 launch spawn_humanoid.launch.py world:=simple_world.world
    ros2 launch spawn_humanoid.launch.py world:=humanoid_world.world
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_world.world',
        description='World file name (in same directory as this launch file)'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid',
        description='Name of the robot in Gazebo'
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='X position to spawn robot'
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='Y position to spawn robot'
    )

    z_arg = DeclareLaunchArgument(
        'z', default_value='1.0',
        description='Z position to spawn robot (1.0 = standing on ground)'
    )

    # Get launch configurations
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')

    # Path to world file (assuming it's in the same directory as launch file)
    # In a real package, you'd use get_package_share_directory
    world_path = PathJoinSubstitution([
        os.path.dirname(__file__),
        world
    ])

    # Path to robot URDF (using the simple_humanoid.urdf from module1)
    # In production, this should be in a proper ROS 2 package
    urdf_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'module1',
        'simple_humanoid.urdf'
    )

    # Read URDF file
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # Start Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen'
    )

    # Start Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot state publisher (publishes TF transforms from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_humanoid',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-topic', '/robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ]
    )

    # Joint state publisher (for movable joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Launch arguments
        world_arg,
        robot_name_arg,
        x_arg,
        y_arg,
        z_arg,

        # Nodes and processes
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        joint_state_publisher,
    ])
