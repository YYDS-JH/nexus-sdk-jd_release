"""
Teleop Manager Node Launch File
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'nexus_manage_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nexus_manage'),
            'config',
            'teleop-manager.yaml'
        ]),
        description='Path to the teleop manager configuration file'
    )
    
    # Teleop Manager节点
    teleop_manager_node = Node(
        package='nexus_manage',
        executable='teleop_manager_node',
        name='teleop_manager_node',
        output='screen',
        parameters=[LaunchConfiguration('nexus_manage_config_file')],
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        config_file_arg,
        teleop_manager_node
    ])
