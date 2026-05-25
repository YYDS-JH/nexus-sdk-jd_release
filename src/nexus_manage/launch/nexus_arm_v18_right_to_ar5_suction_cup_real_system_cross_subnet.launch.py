#!/usr/bin/env python3
"""
跨网段启动文件 — Nexus-Arm V18 Right → AR5 Suction Cup

通过路由器 NAT 端口转发实现跨子网 DDS 通信。
参考 dds-test/src/dds_multi_node_test 的多节点跨网段方案。

用法:
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_suction_cup_real_system_cross_subnet.launch.py
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_suction_cup_real_system_cross_subnet.launch.py role:=slave
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_suction_cup_real_system_cross_subnet.launch.py role:=master

端口规则 (ParticipantIndex=auto, DomainGain=0):
  Participant port = _PORT_BASE + ParticipantIndex
  每个节点自动分配一个独立端口。
"""

import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# ═══════════════════════════════════════════════════════════════════════════════
# 部署配置（部署前修改此处）
# ═══════════════════════════════════════════════════════════════════════════════

_DOMAIN_ID = '18'

# Master 侧（外网，Nexus-Arm 端）
_MASTER_NETWORK_INTERFACE = "enp0s31f6"
_MASTER_EXTERNAL_IP = "192.168.8.74"

# Slave 侧（AGV 内网，AR5 端）
_SLAVE_NETWORK_INTERFACE = "eno1"
_SLAVE_EXTERNAL_IP = "192.168.8.78"

# DDS 端口基数（主从共用）
_PORT_BASE = 7000

# ParticipantIndex 最大分配值（预留余量，避免加节点后改路由器转发规则）
_MAX_AUTO_PARTICIPANT_INDEX = 50

# ═══════════════════════════════════════════════════════════════════════════════
# 节点列表（新增节点只需在对应列表追加一行 (package, launch_file)）
# ═══════════════════════════════════════════════════════════════════════════════

_MASTER_NODES = [
    ('teleop_adapter',   'master_nexus_single.launch.py'),
    ('human_data',       'nexus_arm_v18_right_human_data_solver.launch.py'),
    ('robot_controller', 'master_single_nexus.launch.py'),
    ('nexus_manage',     'nexus_nexus-arm_v18_to_ar5_suction_cup_manage.launch.py'),
]

_SLAVE_NODES = [
    ('teleop_adapter',   'slave_ar5_suction_cup_single.launch.py'),
    ('human_data',       'ar5_suction_cup_human_data_solver.launch.py'),
    ('robot_controller', 'slave_single_ar5_suction_cup.launch.py'),
]

# ═══════════════════════════════════════════════════════════════════════════════
# CycloneDDS XML 生成
# ═══════════════════════════════════════════════════════════════════════════════

def _make_peers_xml(local_count: int, remote_count: int, peer_ip: str) -> str:
    """生成 Peers 段：本地 localhost + 远端对等节点。"""
    lines = []
    for i in range(local_count):
        lines.append(f'        <Peer Address="127.0.0.1:{_PORT_BASE + i}"/>')
    for i in range(remote_count):
        lines.append(f'        <Peer Address="{peer_ip}:{_PORT_BASE + i}"/>')
    return '\n'.join(lines)


def _make_cyclonedds_xml(network_interface: str, external_ip: str,
                         local_count: int, peer_ip: str,
                         remote_count: int) -> str:
    # ExternalNetworkAddress 仅在跨网段时设置
    external_addr_xml = (
        f'      <ExternalNetworkAddress>{external_ip}</ExternalNetworkAddress>'
        if external_ip else ''
    )

    peers_xml = _make_peers_xml(local_count, remote_count, peer_ip)

    return f"""\
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
{external_addr_xml}
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>{_MAX_AUTO_PARTICIPANT_INDEX}</MaxAutoParticipantIndex>
      <Peers>
{peers_xml}
      </Peers>
      <Ports>
        <Base>{_PORT_BASE}</Base>
        <DomainGain>0</DomainGain>
        <ParticipantGain>1</ParticipantGain>
        <UnicastMetaOffset>0</UnicastMetaOffset>
        <UnicastDataOffset>0</UnicastDataOffset>
      </Ports>
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


def _write_cyclonedds_config(xml_content: str) -> str:
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='cyclonedds_nexus_cross_subnet_',
        suffix='.xml',
        delete=False,
    )
    tmp.write(xml_content)
    tmp.flush()
    tmp.close()
    return tmp.name


# ═══════════════════════════════════════════════════════════════════════════════
# Launch 组装
# ═══════════════════════════════════════════════════════════════════════════════

def _include(package: str, launch_file: str) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
        ])
    )


def launch_setup(context, *args, **kwargs):
    role = LaunchConfiguration('role').perform(context).strip().lower()

    # 根据 role 选取配置
    if role == 'master':
        network_interface = _MASTER_NETWORK_INTERFACE
        external_ip = _MASTER_EXTERNAL_IP
        peer_ip = _SLAVE_EXTERNAL_IP
        local_nodes = _MASTER_NODES
        remote_nodes = _SLAVE_NODES
    elif role == 'slave':
        network_interface = _SLAVE_NETWORK_INTERFACE
        external_ip = _SLAVE_EXTERNAL_IP
        peer_ip = _MASTER_EXTERNAL_IP
        local_nodes = _SLAVE_NODES
        remote_nodes = _MASTER_NODES
    else:
        # 单机模式：全部节点本地回环通信，不设 ExternalNetworkAddress
        network_interface = 'auto'
        external_ip = ''
        peer_ip = ''
        local_nodes = _MASTER_NODES + _SLAVE_NODES
        remote_nodes = []

    local_count = len(local_nodes)
    remote_count = len(remote_nodes)

    xml_content = _make_cyclonedds_xml(
        network_interface, external_ip, local_count, peer_ip, remote_count
    )
    cyclonedds_xml_path = _write_cyclonedds_config(xml_content)

    env_actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', _DOMAIN_ID),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + cyclonedds_xml_path),
    ]

    # 组装节点
    if role == 'master':
        launch_actions = [
            _include(pkg, launch_file) for pkg, launch_file in _MASTER_NODES
        ]
    elif role == 'slave':
        launch_actions = [
            _include(pkg, launch_file) for pkg, launch_file in _SLAVE_NODES
        ]
    else:
        launch_actions = [
            _include(pkg, launch_file)
            for pkg, launch_file in (_MASTER_NODES + _SLAVE_NODES)
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
        OpaqueFunction(function=launch_setup),
    ])
