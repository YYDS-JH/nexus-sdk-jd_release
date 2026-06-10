#!/usr/bin/env python3
"""
Launch file for single AR5 08L arm with suction cup teleop control
Loads configuration from slave_ar5_08l_suction_cup_single.yaml and starts the driver node.
AR5 uses Rokae SDK torque control with MIT formula conversion.
Suction cup (SuckerAngle, SuckerDuty) controlled via PeripheralsClient HTTP API.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    base_name = 'slave_arm_adapter'
    node_name = f'{robot_id}_{base_name}' if robot_id else base_name

    pkg_share = get_package_share_directory('teleop_adapter')
    pkg_lib = os.path.normpath(os.path.join(pkg_share, '..', '..', 'lib'))

    config_file = os.path.join(
        pkg_share, 'config', 'slave_ar5_08l_suction_cup_single.yaml')

    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    ld_path = pkg_lib + ':' + existing_ld if existing_ld else pkg_lib

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
        additional_env={'LD_LIBRARY_PATH': ld_path},
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
