#!/usr/bin/env python3
"""独立启动 slave_stack_supervisor（从臂 Orin 常驻进程）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('target_car', default_value=''),
        DeclareLaunchArgument('yj_device_id', default_value='YJ-RC-005'),
        DeclareLaunchArgument('yj_target_ip', default_value=''),
        DeclareLaunchArgument('auto_start_on_boot', default_value='false'),
        Node(
            package='nexus_manage',
            executable='slave_stack_supervisor_node',
            name='slave_stack_supervisor',
            output='screen',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'target_car': LaunchConfiguration('target_car'),
                'yj_device_id': LaunchConfiguration('yj_device_id'),
                'yj_target_ip': LaunchConfiguration('yj_target_ip'),
                'auto_start_on_boot': LaunchConfiguration('auto_start_on_boot'),
                'stack_launch_package': 'nexus_manage',
                'stack_launch_file': 'slave_stack_only.launch.py',
            }],
        ),
    ])
