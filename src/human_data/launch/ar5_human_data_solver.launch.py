#!/usr/bin/env python3
# Launch human_data_solver_node with YAML config for AR5 robot

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    base_name = 'ar5_human_data_solver'
    node_name = f'{robot_id}_{base_name}' if robot_id else base_name
    config_path = LaunchConfiguration('ar5_human_data_config').perform(context)

    params = [config_path]
    if robot_id:
        params.append({"robot_name": robot_id})

    node = Node(
        package='human_data',
        executable='human_data_solver_node',
        name=node_name,
        parameters=params,
        output='screen',
    )
    return [node]


def generate_launch_description():
    pkg_share = get_package_share_directory('human_data')
    default_config = os.path.join(
        pkg_share, 'config', 'ar5_human_data_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ar5_human_data_config',
            default_value=default_config,
            description='Path to YAML config for ar5_human_data_solver_node'
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Optional prefix for node name to avoid conflicts'
        ),
        OpaqueFunction(function=launch_setup),
    ])
