#!/usr/bin/env python3
"""独立启动 slave_stack_supervisor（从臂 Orin 常驻进程）。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from supervisor_launch import resolve_supervisor_cmd


def _launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    yj_device_id = LaunchConfiguration('yj_device_id').perform(context).strip()
    yj_target_ip = LaunchConfiguration('yj_target_ip').perform(context).strip()
    auto_start = LaunchConfiguration('auto_start_on_boot').perform(context).strip()

    cmd = resolve_supervisor_cmd(robot_id)
    cmd.extend([
        '-p', f'yj_device_id:={yj_device_id}',
        '-p', f'yj_target_ip:={yj_target_ip}',
        '-p', f'auto_start_on_boot:={auto_start}',
    ])
    return [ExecuteProcess(cmd=cmd, output='screen', shell=False)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('yj_device_id', default_value='YJ-RC-005'),
        DeclareLaunchArgument('yj_target_ip', default_value=''),
        DeclareLaunchArgument('auto_start_on_boot', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
