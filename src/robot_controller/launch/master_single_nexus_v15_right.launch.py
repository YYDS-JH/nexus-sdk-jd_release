#!/usr/bin/env python3
"""
Nexus-Arm V15 Right 控制系统启动文件（仅控制层）

启动：
- arm_control_node (控制层)

使用 v15 right 专用配置文件

Usage:
  ros2 launch robot_controller master_single_nexus_v15_right.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


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

    # 机器人结构配置（URDF路径、关节配置、话题等）
    robot_config = PathJoinSubstitution([
        robot_controller_share,
        'config',
        'master_single_nexus_v15_right_config.yaml'
    ])

    # 控制器参数配置（Kp/Kd增益、控制频率等）
    controller_config = PathJoinSubstitution([
        robot_controller_share,
        'config',
        'master_arm_control_nexus_v15_right.yaml'
    ])

    # 夹爪配置参数
    gripper_config = PathJoinSubstitution([
        robot_controller_share,
        'config',
        'gripper_config_nexus_v15_right.yaml'
    ])

    # ========== Launch Nodes ==========

    # 修复 prefix 的条件化逻辑，避免在生成描述时调用 perform
    debug_prefix = ['gdb -ex run --args'] if LaunchConfiguration('debug') == 'true' else []

    # Arm Control Node（控制层）
    arm_control_node = Node(
        package='robot_controller',
        executable='arm_control_node',
        name='master_arm_control',
        output='screen',
        parameters=[
            robot_config,       # 机器人结构配置
            controller_config, # 控制器参数配置
            gripper_config     # 夹爪配置参数
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        prefix=debug_prefix,
        respawn=False,
    )

    return LaunchDescription([
        declare_log_level,
        declare_debug,
        arm_control_node,
    ])
