#!/usr/bin/env python3
"""
从臂遥操栈（不含 supervisor）— 由 slave_stack_supervisor 子进程启动。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include_stack(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    startup_move_to_init = LaunchConfiguration('startup_move_to_init').perform(context).strip()
    launch_args = {
        'role': 'slave',
        'supervisor_mode': 'false',
        'stack_only': 'true',
        'startup_move_to_init': startup_move_to_init,
    }
    if robot_id:
        launch_args['robot_id'] = robot_id
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nexus_manage'),
                    'launch',
                    'nexus_arm_v18_right_to_ar5_real_system_cross_subnet.launch.py',
                ]),
            ]),
            launch_arguments=launch_args.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('startup_move_to_init', default_value='false'),
        OpaqueFunction(function=_include_stack),
    ])
