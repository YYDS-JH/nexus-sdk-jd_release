#!/usr/bin/env python3
"""
Launch file for single table arm teleop control
Loads configuration from table_arm_single.yaml and starts the driver node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    base_name = 'master_arm_adapter'
    node_name = f'{robot_id}_{base_name}' if robot_id else base_name

    pkg_dir = get_package_share_directory('teleop_adapter')
    config_file = os.path.join(pkg_dir, 'config', 'master_arm_single.yaml')

    params = [config_file]
    if robot_id:
        params.append({"robot_name": robot_id})

    node = Node(
        package='teleop_adapter',
        executable='teleop_arm_driver_main',
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=params,
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Optional prefix for node name to avoid conflicts'
        ),
        OpaqueFunction(function=launch_setup),
    ])
