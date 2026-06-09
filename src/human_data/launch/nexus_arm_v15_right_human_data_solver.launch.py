#!/usr/bin/env python3
# Launch human_data_solver_node with Nexus-Arm V15 Right YAML config

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('human_data')
    default_config = os.path.join(pkg_share, 'config', 'nexus-arm_v15_right_human_data_config.yaml')

    config_arg = DeclareLaunchArgument(
        'nexus_arm_config',
        default_value=default_config,
        description='Path to YAML config for nexus_arm_human_data_solver_node'
    )

    nexus_human_data_solver_node = Node(
        package='human_data',
        executable='human_data_solver_node',
        name='nexus_arm_human_data_solver',
        parameters=[LaunchConfiguration('nexus_arm_config')],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        nexus_human_data_solver_node
    ])
