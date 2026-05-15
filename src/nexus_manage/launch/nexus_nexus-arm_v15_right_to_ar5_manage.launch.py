"""
Teleop Manager Node Launch File - Nexus-Arm V15 Right to AR5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'nexus_manage_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nexus_manage'),
            'config',
            'nexus-arm_v15_right_to_ar5_manage.yaml'
        ]),
        description='Path to the teleop manager configuration file'
    )

    nexus_arm_to_ar5_manager_node = Node(
        package='nexus_manage',
        executable='teleop_manager_node',
        name='nexus_arm_to_ar5_manager_node',
        output='screen',
        parameters=[LaunchConfiguration('nexus_manage_config_file')],
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0
    )

    return LaunchDescription([
        config_file_arg,
        nexus_arm_to_ar5_manager_node
    ])
