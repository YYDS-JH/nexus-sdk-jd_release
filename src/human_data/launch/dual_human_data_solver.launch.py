#!/usr/bin/env python3
# Launch two human_data_solver_node instances with different configs

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()

    pkg_share = get_package_share_directory('human_data')
    robot_config = os.path.join(pkg_share, 'config', 'robot_human_data_config.yaml')
    teleop_config = os.path.join(pkg_share, 'config', 'teleop_human_data_config.yaml')

    robot_base = 'robot_human_data_solver'
    teleop_base = 'teleop_human_data_solver'
    robot_node_name = f'{robot_id}_{robot_base}' if robot_id else robot_base
    teleop_node_name = f'{robot_id}_{teleop_base}' if robot_id else teleop_base

    params = [robot_config]
    if robot_id:
        params.append({"robot_name": robot_id})

    robot_solver_node = Node(
        package='human_data',
        executable='human_data_solver_node',
        name=robot_node_name,
        parameters=params,
        output='screen',
    )
    teleop_params = [teleop_config]
    if robot_id:
        teleop_params.append({"robot_name": robot_id})

    teleop_solver_node = Node(
        package='human_data',
        executable='human_data_solver_node',
        name=teleop_node_name,
        parameters=teleop_params,
        output='screen',
    )
    return [robot_solver_node, teleop_solver_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Optional prefix for node name to avoid conflicts'
        ),
        OpaqueFunction(function=launch_setup),
    ])
