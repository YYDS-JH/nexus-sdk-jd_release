#!/usr/bin/env python3
"""
Launch file for single AR5 arm with gripper teleop control
Loads configuration from slave_ar5_gripper_single.yaml and starts the driver node.
AR5 uses Rokae SDK torque control with MIT formula conversion.
Gripper (setGripperPosition/getGripperPosition) controlled via PeripheralsClient HTTP API.
"""

import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('teleop_adapter')
    pkg_lib = os.path.normpath(os.path.join(pkg_share, '..', '..', 'lib'))

    config_file = os.path.join(pkg_share, 'config', 'slave_ar5_gripper_single.yaml')

    # 确保运行时链接器能找到 Rokae AR5 SDK 的 libxCoreSDK.so.0
    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    ld_path = pkg_lib + ':' + existing_ld if existing_ld else pkg_lib

    ar5_gripper_arm_driver_node = Node(
        package='teleop_adapter',
        executable='teleop_arm_driver_main',
        name='slave_arm_adapter',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info'],
        additional_env={'LD_LIBRARY_PATH': ld_path},
    )

    return LaunchDescription([
        ar5_gripper_arm_driver_node
    ])
