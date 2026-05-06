#!/usr/bin/env python3
"""
Launch file for single table arm teleop control
Loads configuration from slave_arm_single.yaml and starts the driver node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('teleop_adapter')
    
    # Path to the configuration file
    config_file = os.path.join(pkg_dir, 'config', 'slave_y1_single.yaml')
    
    # Teleop Arm Driver Node
    teleop_arm_driver_node = Node(
        package='teleop_adapter',
        executable='teleop_arm_driver_main',
        name='slave_arm_adapter',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        teleop_arm_driver_node
    ])
