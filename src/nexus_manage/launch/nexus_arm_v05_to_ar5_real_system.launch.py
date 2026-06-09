#!/usr/bin/env python3
"""
主启动文件 - 一键启动整个远程操控系统（Nexus-Arm v05 → AR5）

用法:
  # 全部启动（默认，domain_id=0）
  ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py

  # 仅启动 Slave 侧（AR5 机器上运行）
  ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py role:=slave

  # 仅启动 Master 侧（Nexus-Arm 机器上运行）
  ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py role:=master

  # 指定域 ID（无需修改 ~/.bashrc）
  ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py role:=slave domain_id:=42

  # 指定 DDS 通信网卡（多网卡机器必须显式指定，否则可能选错网卡导致跨机发现失败）
  # 查看可用网卡：ip -4 addr | grep "10.18"
  ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py role:=master network_interface:=enp3s0
  ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py role:=slave  network_interface:=eth0
  # 也可以直接填本机 IP（与网卡名等效）
  ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py role:=master network_interface:=10.18.64.46
  ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py role:=slave  network_interface:=10.18.20.25

中间件: CycloneDDS（rmw_cyclonedds_cpp），配置内联于本文件
Slave  侧包含: ar5_arm_adapter、ar5_human_data_solver、slave_controller
Master 侧包含: nexus_arm_adapter、nexus_arm_human_data_solver、master_controller、manager
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# ── CycloneDDS 内联配置 ────────────────────────────────────────────────────────
# 修改此处以调整 DDS 行为，无需外部 XML 文件
# {network_interface} 占位符由 _write_cyclonedds_config() 在运行时替换
_CYCLONEDDS_XML_TEMPLATE = """\
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config
              https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain>
    <General>
      <Interfaces>
        <!-- 显式指定用于 DDS 通信的网卡，避免多网卡场景下选错接口 -->
        <!-- 传入 "auto" 则自动选择；也可传入网卡名如 eth0/enp3s0，或 IP 如 10.18.20.25 -->
        <NetworkInterface name="{network_interface}" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <!-- 支持同一主机上最多 50 个 ROS 2 进程 -->
      <MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>
      <!-- 静态 Peer 列表：master 与 slave 机器的 IP -->
      <Peers>
        <Peer Address="10.18.20.25"/>
        <Peer Address="10.18.64.46"/>
      </Peers>
    </Discovery>
    <Internal>
      <Watermarks>
        <!-- 写队列高水位，调大可缓解高频话题丢包 -->
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <!-- 只输出 warning 及以上级别，屏蔽 tev:/ddsi_ 等底层重试噪音 -->
      <Verbosity>warning</Verbosity>
      <OutputFile>stderr</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
"""


def _write_cyclonedds_config(network_interface: str = 'auto') -> str:
    """将内联 XML 写入临时文件，返回文件路径。"""
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

    # ── 环境变量（固定使用 CycloneDDS，配置来自内联 XML）────────────────────
    env_actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + cyclonedds_xml_path),
    ]

    # ── Slave 侧组件 ──────────────────────────────────────────────────────────
    ar5_arm_launch = _include('teleop_adapter', 'slave_ar5_single.launch.py')
    ar5_human_data_solver_launch = _include('human_data', 'ar5_human_data_solver.launch.py')
    slave_controller_launch = _include('robot_controller', 'slave_single_ar5.launch.py')

    # ── Master 侧组件 ─────────────────────────────────────────────────────────
    nexus_arm_launch = _include('teleop_adapter', 'master_nexus_single.launch.py')
    nexus_arm_human_data_solver_launch = _include('human_data', 'nexus_arm_v05_human_data_solver.launch.py')
    master_controller_launch = _include('robot_controller', 'master_single_nexus.launch.py')
    manager_launch = _include('nexus_manage', 'nexus_nexus-arm_v05_to_ar5_manage.launch.py')

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
        # 默认全部启动
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
            default_value='7',
            description='ROS_DOMAIN_ID，同一局域网内各机器必须一致。'
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='auto',
            description=(
                '用于 DDS 跨机通信的网卡名或 IP。'
                '多网卡时必须显式指定，例如：network_interface:=enp3s0 或 network_interface:=10.18.20.25。'
                '填 "auto" 则由 CycloneDDS 自动选择（多网卡场景下可能选错）。'
            )
        ),
        OpaqueFunction(function=launch_setup),
    ])
