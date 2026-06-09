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
    
    # 1. 仿真器 Launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mujoco_sim'),
                'launch',
                'y1_dual_sim.launch.py'
            ])
        ])
    )
    
    # 2. Human Data Solver Launch
    solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'dual_human_data_solver.launch.py'
            ])
        ])
    )
    
    # 3. Gripper Keyboard Launch
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gripper_keyboard'),
                'launch',
                'left_gripper.launch.py'
            ])
        ])
    )
    
    # 4. Nexus Manage Launch
    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nexus_manage'),
                'launch',
                'nexus_manage.launch.py'
            ])
        ])
    )
    
    # 5. Master Arm Single Launch
    master_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('teleop_adapter'),
                'launch',
                'master_y1_single.launch.py'
            ])
        ])
    )
    
    # 6. Slave Arm Single Launch
    slave_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('teleop_adapter'),
                'launch',
                'slave_y1_single.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        # 启动所有组件（按依赖顺序）
        sim_launch,           # 仿真器
        solver_launch,        # Human Data Solver
        gripper_launch,       # 右手夹爪控制
        manager_launch,       # Teleop 管理器
        # master_arm_launch,    # 主控臂适配器
        # slave_arm_launch,     # 从控臂适配器
    ])
