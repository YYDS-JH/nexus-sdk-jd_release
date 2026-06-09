#!/usr/bin/env python3
# Launch two human_data_solver_node instances with different configs

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('human_data')
    robot_config = os.path.join(pkg_share, 'config', 'robot_human_data_config.yaml')
    teleop_config = os.path.join(pkg_share, 'config', 'teleop_human_data_config.yaml')
    
    # Robot human data solver node
    robot_solver_node = Node(
        package='human_data',
        executable='human_data_solver_node',
        name='robot_human_data_solver',
        parameters=[robot_config],
        output='screen'
    )

    # Teleop human data solver node
    teleop_solver_node = Node(
        package='human_data',
        executable='human_data_solver_node',
        name='teleop_human_data_solver',
        parameters=[teleop_config],
        output='screen'
    )

    return LaunchDescription([
        robot_solver_node,
        teleop_solver_node
    ])
