#!/usr/bin/env python3
"""
跨网段启动文件 — Nexus-Arm V18 Right → AR5 08L Suction Cup（一控多部署）

通过路由器 NAT 端口转发实现跨子网 DDS 通信。
支持一台 Master（Nexus-Arm）同时遥控多台 Slave（AR5），
运行时由 nexus_manage 根据 /cl/operator_status 动态切换从臂。

用法:
  # === Master 侧（Nexus-Arm 主机，只启动一次）===
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_08l_suction_cup_real_system_cross_subnet.launch.py role:=master
  # 注意：role=master 时 robot_id 参数被忽略，
  # nexus_manage 通过 switchToRobot() 在运行时动态路由到不同从臂。

  # === Slave 侧（每台 AR5 AGX Orin 各启动一次）===
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_08l_suction_cup_real_system_cross_subnet.launch.py role:=slave robot_id:=ar5_01
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_08l_suction_cup_real_system_cross_subnet.launch.py role:=slave robot_id:=ar5_02
  # robot_id 必须与 slave_registry.yaml 中对应从臂的 id 字段一致，
  # 用于节点名去重和 topic namespace 隔离。

  # === 单机测试模式 ===
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_08l_suction_cup_real_system_cross_subnet.launch.py

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

from slave_registry import find_slave_by_ip, generate_peers_xml, load_registry

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
    ('nexus_manage',     'nexus_nexus-arm_v18_to_ar5_08l_suction_cup_manage.launch.py'),
]

_SLAVE_NODES = [
    ('teleop_adapter',   'slave_ar5_08l_suction_cup_single.launch.py'),
    ('human_data',       'ar5_08l_suction_cup_human_data_solver.launch.py'),
    ('robot_controller', 'slave_single_ar5_08l_suction_cup.launch.py'),
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
                         peers_xml: str) -> str:
    # ExternalNetworkAddress 仅在跨网段时设置
    external_addr_xml = (
        f'      <ExternalNetworkAddress>{external_ip}</ExternalNetworkAddress>'
        if external_ip else ''
    )

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

def _include(package: str, launch_file: str, launch_arguments=None):
    kwargs = {}
    if launch_arguments:
        kwargs['launch_arguments'] = launch_arguments.items()
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
        ]),
        **kwargs
    )
def _resolve_registry_path(context):
    return PathJoinSubstitution([
        FindPackageShare('nexus_manage'), 'config', 'slave_registry.yaml',
    ]).perform(context)


def launch_setup(context, *args, **kwargs):
    role = LaunchConfiguration('role').perform(context).strip().lower()
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()

    # Attempt to load slave registry
    registry_path = _resolve_registry_path(context)
    slaves = load_registry(registry_path)

    if slaves:
        # ── Multi-slave mode ──
        if not role:
            matched = find_slave_by_ip(slaves)
            role = 'slave' if matched else 'master'

        if role == 'master':
            network_interface = _MASTER_NETWORK_INTERFACE
            external_ip = _MASTER_EXTERNAL_IP
            local_nodes = _MASTER_NODES
            remote_nodes = _SLAVE_NODES
            peers_xml = generate_peers_xml(
                slaves,
                local_count=len(_MASTER_NODES),
                remote_count=len(_SLAVE_NODES),
                port_base=_PORT_BASE,
            )
        elif role == 'slave':
            matched = find_slave_by_ip(slaves)
            if not matched:
                raise RuntimeError(
                    'Cannot determine slave identity: '
                    'no registry entry matches local IPs')
            network_interface = matched['network_interface']
            external_ip = matched['ip']
            local_nodes = _SLAVE_NODES
            remote_nodes = _MASTER_NODES
            peers_xml = _make_peers_xml(
                len(_SLAVE_NODES), len(_MASTER_NODES), _MASTER_EXTERNAL_IP)
        else:
            raise RuntimeError(f'Unknown role: {role}')
    else:
        # ── Legacy mode (no registry) ──
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
            network_interface = 'auto'
            external_ip = ''
            peer_ip = ''
            local_nodes = _MASTER_NODES + _SLAVE_NODES
            remote_nodes = []

        peers_xml = _make_peers_xml(
            len(local_nodes), len(remote_nodes), peer_ip)

    xml_content = _make_cyclonedds_xml(
        network_interface, external_ip, peers_xml,
    )
    cyclonedds_xml_path = _write_cyclonedds_config(xml_content)

    env_actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', _DOMAIN_ID),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + cyclonedds_xml_path),
    ]

    # Master role: nexus_manage handles multi-slave via dynamic switchToRobot(),
    # so robot_id must NOT be baked into node names at launch time.
    launch_args = {'robot_id': robot_id} if (role != 'master' and robot_id) else None

    if role == 'master':
        launch_actions = [
            _include(pkg, launch_file, launch_args) for pkg, launch_file in _MASTER_NODES
        ]
    elif role == 'slave':
        launch_actions = [
            _include(pkg, launch_file, launch_args) for pkg, launch_file in _SLAVE_NODES
        ]
    else:
        launch_actions = [
            _include(pkg, launch_file, launch_args)
            for pkg, launch_file in (_MASTER_NODES + _SLAVE_NODES)
        ]

    return env_actions + launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Optional prefix for node name to avoid conflicts'
        ),
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
