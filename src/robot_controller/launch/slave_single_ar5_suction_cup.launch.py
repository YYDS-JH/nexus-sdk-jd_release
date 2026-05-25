#!/usr/bin/env python3
"""
AR5 Suction Cup 从臂控制系统启动文件

启动 arm_control_node 作为 AR5 吸盘版从臂控制器。
加载 7-DOF arm + 1-DOF suction cup 配置。

Usage:
  ros2 launch robot_controller slave_single_ar5_suction_cup.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_controller_share = FindPackageShare('robot_controller')

    # ========== Declare Launch Arguments ==========

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)'
    )

    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable GDB debugging (true/false)'
    )

    # ========== Configuration Files ==========

    robot_config = PathJoinSubstitution([
        robot_controller_share,
        'config',
        'slave_single_ar5_suction_cup_config.yaml'
    ])

    controller_config = PathJoinSubstitution([
        robot_controller_share,
        'config',
        'arm_control_ar5_suction_cup.yaml'
    ])

    gripper_config = PathJoinSubstitution([
        robot_controller_share,
        'config',
        'gripper_config_ar5_suction_cup.yaml'
    ])

    # ========== Launch Nodes ==========

    slave_arm_control_node = Node(
        package='robot_controller',
        executable='arm_control_node',
        name='slave_arm_control',
        output='screen',
        parameters=[
            robot_config,
            controller_config,
            gripper_config
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        prefix=['gdb -ex run --args'] if LaunchConfiguration('debug') == 'true' else [],
        respawn=False,
    )

    return LaunchDescription([
        declare_log_level,
        declare_debug,
        slave_arm_control_node,
    ])
