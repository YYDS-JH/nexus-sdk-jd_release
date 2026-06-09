#!/usr/bin/env python3
"""
AR5 夹爪版从臂控制层 — AR5-gripper (7-DOF arm + 1-DOF gripper)

对接 mujoco_sim /ar5_gripper/robot/...

Usage:
  ros2 launch robot_controller slave_single_ar5_gripper.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_controller_share = FindPackageShare('robot_controller')

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)',
    )

    robot_config = PathJoinSubstitution([
        robot_controller_share,
        'config',
        'slave_single_ar5_gripper_config.yaml',
    ])
    controller_config = PathJoinSubstitution([
        robot_controller_share,
        'config',
        'arm_control_ar5_gripper.yaml',
    ])
    gripper_config = PathJoinSubstitution([
        robot_controller_share,
        'config',
        'gripper_config_ar5.yaml',
    ])

    arm_control_node = Node(
        package='robot_controller',
        executable='arm_control_node',
        name='slave_arm_control',
        output='screen',
        parameters=[robot_config, controller_config, gripper_config],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=False,
    )

    return LaunchDescription([
        declare_log_level,
        arm_control_node,
    ])
