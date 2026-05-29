#!/usr/bin/env python3
"""
主启动文件 - Nexus-Arm V05 控制 AR5 仿真系统
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # 1. AR5 仿真器 Launch
    ar5_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mujoco_sim'),
                'launch',
                'ar5_sim.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
    )

    # 2. Nexus-Arm V05 仿真器 Launch
    nexus_arm_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mujoco_sim'),
                'launch',
                'nexus_arm_v05_sim.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
    )

    # 3. Human Data Solver Launch
    ar5_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'ar5_human_data_solver.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
    )

    nexus_arm_human_data_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('human_data'),
                'launch',
                'nexus_arm_v05_human_data_solver.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
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
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
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
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
    )
    
    # 6. Robot Controller Launch - Slave (AR5) → robot_controller/slave_single_ar5.launch.py
    slave_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_controller'),
                'launch',
                'slave_single_ar5.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
    )
    
    # 7. Nexus Manage Launch
    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nexus_manage'),
                'launch',
                'nexus_nexus-arm_v05_to_ar5_manage.launch.py'
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

        ar5_sim_launch,                     # AR5 仿真器
        nexus_arm_sim_launch,               # Nexus-Arm V05 仿真器
        ar5_human_data_solver_launch,       # AR5 Human Data Solver
        nexus_arm_human_data_solver_launch, # Nexus-Arm V05 Human Data Solver
        gripper_launch,                     # 夹爪控制
        master_controller_launch,           # Master 控制器 (Nexus-Arm)
        slave_controller_launch,            # Slave 控制器 (AR5)
        manager_launch,                     # Nexus-Arm V05 to AR5 管理器
    ])

