"""
Teleop Manager Node Launch File - Nexus-Arm V15 Right to AR5 08L Suction Cup
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    base_name = 'nexus_arm_v15_right_to_ar5_08l_suction_cup_manager_node'
    node_name = f'{robot_id}_{base_name}' if robot_id else base_name

    config_path = LaunchConfiguration('nexus_manage_config_file').perform(context)

    params = [config_path]
    if robot_id:
        params.append({"robot_name": robot_id})

    node = Node(
        package='nexus_manage',
        executable='teleop_manager_node',
        name=node_name,
        output='screen',
        parameters=params,
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0,
    )
    return [node]


def generate_launch_description():
    pkg_share = get_package_share_directory('nexus_manage')
    default_config = os.path.join(
        pkg_share, 'config',
        'nexus-arm_v15_right_to_ar5_08l_suction_cup_manage.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'nexus_manage_config_file',
            default_value=default_config,
            description='Path to the teleop manager configuration file'
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Optional prefix for node name to avoid conflicts'
        ),
        OpaqueFunction(function=launch_setup),
    ])
