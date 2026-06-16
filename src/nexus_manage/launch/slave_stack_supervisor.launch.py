#!/usr/bin/env python3
"""独立启动 slave_stack_supervisor（从臂 Orin 常驻进程）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('nexus_manage')
    script = os.path.join(share, 'scripts', 'slave_stack_supervisor_node.py')

    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    yj_device_id = LaunchConfiguration('yj_device_id').perform(context).strip()
    yj_target_ip = LaunchConfiguration('yj_target_ip').perform(context).strip()
    auto_start = LaunchConfiguration('auto_start_on_boot').perform(context).strip()

    cmd = [
        'python3', script,
        '--ros-args',
        '-r', '__node:=slave_stack_supervisor',
        '-p', 'stack_launch_package:=nexus_manage',
        '-p', 'stack_launch_file:=slave_stack_only.launch.py',
        f'-p', f'auto_start_on_boot:={auto_start}',
        f'-p', f'robot_id:={robot_id}',
        f'-p', f'target_car:={robot_id}',
        f'-p', f'yj_device_id:={yj_device_id}',
        f'-p', f'yj_target_ip:={yj_target_ip}',
    ]
    return [ExecuteProcess(cmd=cmd, output='screen', shell=False)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('yj_device_id', default_value='YJ-RC-005'),
        DeclareLaunchArgument('yj_target_ip', default_value=''),
        DeclareLaunchArgument('auto_start_on_boot', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
