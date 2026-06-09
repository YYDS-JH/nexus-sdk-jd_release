#!/usr/bin/env python3
"""
主启动文件 - 一键启动整个远程操控系统
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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
        ])
    )

    # 2. Nexus-Arm 仿真器 Launch
    nexus_arm_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mujoco_sim'),
                'launch',
                'nexus_arm_sim.launch.py'
            ])
        ])
    )

    # 3. Human Data Solver Launch
    y1_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'y1_human_data_solver.launch.py'
            ])
        ])
    )

    nexus_arm_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'nexus_arm_human_data_solver.launch.py'
            ])
        ])
    )
    
    # 3. Gripper Keyboard Launch
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gripper_keyboard'),
                'launch',
                'nexus-arm_left_gripper.launch.py'
            ])
        ])
    )
    
    # 4. Robot Controller Launch - Master (Nexus-Arm)
    master_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_controller'),
                'launch',
                'master_single_nexus.launch.py'
            ])
        ])
    )
    
    # 5. Robot Controller Launch - Slave (Y1)
    slave_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_controller'),
                'launch',
                'slave_single_y1.launch.py'
            ])
        ])
    )
    
    # 6. Nexus Manage Launch
    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nexus_manage'),
                'launch',
                'nexus_nexus-arm_to_y1_manage.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        # 启动所有组件（按依赖顺序）
        y1_sim_launch,           # Y1 仿真器
        nexus_arm_sim_launch,   # Nexus-Arm 仿真器
        y1_human_data_solver_launch,        # Y1 Human Data Solver
        nexus_arm_human_data_solver_launch, # Nexus-Arm Human Data Solver
        gripper_launch,          # 右手夹爪控制
        master_controller_launch, # Master 控制器 (Nexus-Arm)
        slave_controller_launch,  # Slave 控制器 (Y1)
        manager_launch,          # Nexus-Arm to Y1 管理器
    ])
