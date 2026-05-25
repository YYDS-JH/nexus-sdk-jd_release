#!/usr/bin/env python3
# Launch human_data_solver_node with YAML config for AR5 gripper robot

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('human_data')
    default_config = os.path.join(pkg_share, 'config', 'ar5_gripper_human_data_config.yaml')

    ar5_gripper_human_data_config_arg = DeclareLaunchArgument(
        'ar5_gripper_human_data_config',
        default_value=default_config,
        description='Path to YAML config for ar5_gripper_human_data_solver_node'
    )

    ar5_gripper_human_data_solver_node = Node(
        package='human_data',
        executable='human_data_solver_node',
        name='ar5_gripper_human_data_solver',
        parameters=[LaunchConfiguration('ar5_gripper_human_data_config')],
        output='screen'
    )

    return LaunchDescription([
        ar5_gripper_human_data_config_arg,
        ar5_gripper_human_data_solver_node
    ])
