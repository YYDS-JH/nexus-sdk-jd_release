#!/usr/bin/env python3
"""
主启动文件 - 一键启动整个远程操控系统（Nexus-Arm V15 Right → AR5）

用法:
  ros2 launch nexus_manage nexus_arm_v15_right_to_ar5_real_system.launch.py
  ros2 launch nexus_manage nexus_arm_v15_right_to_ar5_real_system.launch.py role:=slave
  ros2 launch nexus_manage nexus_arm_v15_right_to_ar5_real_system.launch.py role:=master
  ros2 launch nexus_manage nexus_arm_v15_right_to_ar5_real_system.launch.py role:=slave domain_id:=42
  ros2 launch nexus_manage nexus_arm_v15_right_to_ar5_real_system.launch.py role:=master network_interface:=enp3s0
  ros2 launch nexus_manage nexus_arm_v15_right_to_ar5_real_system.launch.py role:=slave  network_interface:=eth0
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

_CYCLONEDDS_XML_TEMPLATE = """\
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config
              https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="{network_interface}" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address="10.18.20.25"/>
        <Peer Address="10.18.64.46"/>
      </Peers>
    </Discovery>
    <Internal>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
      <OutputFile>stderr</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
"""


def _write_cyclonedds_config(network_interface: str = 'auto') -> str:
    xml_content = _CYCLONEDDS_XML_TEMPLATE.format(network_interface=network_interface)
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='cyclonedds_nexus_',
        suffix='.xml',
        delete=False,
    )
    tmp.write(xml_content)
    tmp.flush()
    tmp.close()
    return tmp.name


def _include(package, launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
        ])
    )


def launch_setup(context, *args, **kwargs):
    role = LaunchConfiguration('role').perform(context).strip().lower()
    domain_id = LaunchConfiguration('domain_id').perform(context).strip()
    network_interface = LaunchConfiguration('network_interface').perform(context).strip()

    cyclonedds_xml_path = _write_cyclonedds_config(network_interface)

    env_actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + cyclonedds_xml_path),
    ]

    ar5_arm_launch = _include('teleop_adapter', 'slave_ar5_single.launch.py')
    ar5_human_data_solver_launch = _include('human_data', 'ar5_human_data_solver.launch.py')
    slave_controller_launch = _include('robot_controller', 'slave_single_ar5.launch.py')

    nexus_arm_launch = _include('teleop_adapter', 'master_nexus_single.launch.py')
    nexus_arm_human_data_solver_launch = _include('human_data', 'nexus_arm_v15_right_human_data_solver.launch.py')
    master_controller_launch = _include('robot_controller', 'master_single_nexus_v15_right.launch.py')
    manager_launch = _include('nexus_manage', 'nexus_nexus-arm_v15_right_to_ar5_manage.launch.py')

    if role == 'slave':
        launch_actions = [
            ar5_arm_launch,
            ar5_human_data_solver_launch,
            slave_controller_launch,
        ]
    elif role == 'master':
        launch_actions = [
            nexus_arm_launch,
            nexus_arm_human_data_solver_launch,
            master_controller_launch,
            manager_launch,
        ]
    else:
        launch_actions = [
            ar5_arm_launch,
            nexus_arm_launch,
            ar5_human_data_solver_launch,
            nexus_arm_human_data_solver_launch,
            master_controller_launch,
            slave_controller_launch,
            manager_launch,
        ]

    return env_actions + launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'role',
            default_value='',
            description=(
                'Launch role: "slave" (AR5 side), "master" (Nexus-Arm side), '
                'or empty string to launch all components on a single machine.'
            )
        ),
        DeclareLaunchArgument(
            'domain_id',
            default_value='18',
            description='ROS_DOMAIN_ID'
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='auto',
            description='用于 DDS 跨机通信的网卡名或 IP。'
        ),
        OpaqueFunction(function=launch_setup),
    ])
