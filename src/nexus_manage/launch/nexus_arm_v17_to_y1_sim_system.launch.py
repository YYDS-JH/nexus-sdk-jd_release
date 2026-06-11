#!/usr/bin/env python3
"""
主启动文件 - 一键启动整个远程操控系统（Nexus-Arm V17 → Y1 仿真）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 1. Y1 仿真器 Launch
    y1_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mujoco_sim'),
                'launch',
                'y1_slave_sim.launch.py'
            ])
        ]),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id')}.items(),
    )

    # 2. Nexus-Arm V17 仿真器 Launch
    nexus_arm_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mujoco_sim'),
                'launch',
                'nexus_arm_v17_sim.launch.py'
            ])
        ]),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id')}.items(),
    )

    # 3. Human Data Solver Launch
    y1_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'y1_human_data_solver.launch.py'
            ])
        ]),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id')}.items(),
    )

    nexus_arm_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'nexus_arm_v17_human_data_solver.launch.py'
            ])
        ]),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id')}.items(),
    )

    # 4. Gripper Keyboard Launch
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gripper_keyboard'),
                'launch',
                'nexus-arm_left_gripper.launch.py'
            ])
        ]),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id')}.items(),
    )

    # 5. Robot Controller Launch - Master (Nexus-Arm)
    master_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_controller'),
                'launch',
                'master_single_nexus.launch.py'
            ])
        ]),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id')}.items(),
    )

    # 6. Robot Controller Launch - Slave (Y1)
    slave_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_controller'),
                'launch',
                'slave_single_y1.launch.py'
            ])
        ]),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id')}.items(),
    )

    # 7. Nexus Manage Launch
    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nexus_manage'),
                'launch',
                'nexus_nexus-arm_v17_to_y1_manage.launch.py'
            ])
        ]),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Optional prefix for node name to avoid conflicts'
        ),
        y1_sim_launch,
        nexus_arm_sim_launch,
        y1_human_data_solver_launch,
        nexus_arm_human_data_solver_launch,
        gripper_launch,
        master_controller_launch,
        slave_controller_launch,
        manager_launch,
    ])
