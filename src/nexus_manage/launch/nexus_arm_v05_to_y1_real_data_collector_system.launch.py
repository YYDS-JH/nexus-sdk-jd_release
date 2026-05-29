#!/usr/bin/env python3
"""
主启动文件 - 一键启动整个远程操控系统（Nexus-Arm V15 → Y1）+ 数据采集
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # 1. Y1 Launch
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

    # 2. Nexus-Arm Launch
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

    # 3. Human Data Solver Launch
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
    
    # 6. Robot Controller Launch - Slave (Y1)
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
    
    # 7. Nexus Manage Launch
    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nexus_manage'),
                'launch',
                'nexus_nexus-arm_v05_to_y1_manage.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
    )
    
    # 8. Data Collector Launch
    data_collector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('data_collector'),
                'launch',
                'data_collector.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
    )
    
    # 9. Collector Test Tool Launch
    collector_test_tool_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('collector_test_tool'),
                'launch',
                'collector_test_tool.launch.py'
            ])
        ]),
        launch_arguments={"robot_id": LaunchConfiguration("robot_id")}.items(),
    )

    # 10. Vision Data Hub Launch
    vision_data_hub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vision_data_hub'),
                'launch',
                'vision_hub.launch.py'
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

        # 启动所有组件（按依赖顺序）
        y1_arm_launch,           # Y1
        nexus_arm_launch,        # Nexus-Arm
        y1_human_data_solver_launch,        # Y1 Human Data Solver
        nexus_arm_human_data_solver_launch, # Nexus-Arm V15 Human Data Solver
        # gripper_launch,          # 左手夹爪控制
        master_controller_launch, # Master 控制器 (Nexus-Arm)
        slave_controller_launch,  # Slave 控制器 (Y1)
        manager_launch,          # Nexus-Arm V15 to Y1 管理器
        data_collector_launch,   # 数据采集器
        collector_test_tool_launch,  # 数据采集测试工具
        vision_data_hub_launch,  # 视觉数据中心
    ])

