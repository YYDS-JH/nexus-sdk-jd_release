#!/usr/bin/env python3
"""
主启动文件 - Nexus-Arm V15 Right 控制 AR5 Suction Cup 仿真系统
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 1. AR5 Suction Cup 仿真器 Launch
    ar5_suction_cup_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mujoco_sim'),
                'launch',
                'ar5_suction_cup_sim.launch.py'
            ])
        ])
    )

    # 2. Nexus-Arm V15 Right 仿真器 Launch
    nexus_arm_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mujoco_sim'),
                'launch',
                'nexus_arm_v15_right_sim.launch.py'
            ])
        ])
    )

    # 3. Human Data Solver Launch
    ar5_suction_cup_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'ar5_suction_cup_human_data_solver.launch.py'
            ])
        ])
    )

    nexus_arm_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'nexus_arm_v15_right_human_data_solver.launch.py'
            ])
        ])
    )

    # 4. Gripper Keyboard Launch
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gripper_keyboard'),
                'launch',
                'nexus-arm_left_gripper.launch.py'
            ])
        ])
    )

    # 5. Robot Controller Launch - Master (Nexus-Arm V15 Right)
    master_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_controller'),
                'launch',
                'master_single_nexus_v15_right.launch.py'
            ])
        ])
    )

    # 6. Robot Controller Launch - Slave (AR5 Suction Cup)
    slave_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_controller'),
                'launch',
                'slave_single_ar5_suction_cup.launch.py'
            ])
        ])
    )

    # 7. Nexus Manage Launch
    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nexus_manage'),
                'launch',
                'nexus_nexus-arm_v15_right_to_ar5_suction_cup_manage.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        ar5_suction_cup_sim_launch,
        nexus_arm_sim_launch,
        ar5_suction_cup_human_data_solver_launch,
        nexus_arm_human_data_solver_launch,
        gripper_launch,
        master_controller_launch,
        slave_controller_launch,
        manager_launch,
    ])
