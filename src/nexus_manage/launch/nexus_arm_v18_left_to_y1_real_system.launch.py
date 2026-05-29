#!/usr/bin/env python3
"""
主启动文件 - 一键启动整个远程操控系统（Nexus-Arm V18 Left → Y1 实物）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    y1_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('teleop_adapter'),
                'launch',
                'slave_y1_single.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
        )

    nexus_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('teleop_adapter'),
                'launch',
                'master_nexus_single.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
        )

    y1_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'y1_human_data_solver.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
        )

    nexus_arm_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'nexus_arm_v18_left_human_data_solver.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
        )
    
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gripper_keyboard'),
                'launch',
                'nexus-arm_left_gripper.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
        )
    
    master_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_controller'),
                'launch',
                'master_single_nexus.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
        )
    
    slave_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_controller'),
                'launch',
                'slave_single_y1.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
        )
    
    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nexus_manage'),
                'launch',
                'nexus_nexus-arm_v18_to_y1_manage.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Optional prefix for node name to avoid conflicts'
        ),

        y1_arm_launch,
        nexus_arm_launch,
        y1_human_data_solver_launch,
        nexus_arm_human_data_solver_launch,
        # gripper_launch,
        master_controller_launch,
        slave_controller_launch,
        manager_launch,
    ])
