#!/usr/bin/env python3
"""
跨网段启动文件 — Nexus-Arm V18 Right → AR5 Suction Cup（一控多部署）

通过路由器 NAT 端口转发实现跨子网 DDS 通信。
支持一台 Master（Nexus-Arm）同时遥控多台 Slave（AR5 吸盘），
运行时由 nexus_manage 根据 /cl/operator_status 动态切换从臂。

用法:
  # === Master 侧（Nexus-Arm 主机，只启动一次）===
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_suction_cup_real_system_cross_subnet.launch.py role:=master
  # 注意：role=master 时 robot_id 参数被忽略，
  # nexus_manage 通过 switchToRobot() 在运行时动态路由到不同从臂。

  # === Legacy 单臂 + supervisor（slave_registry.yaml 中 slaves: []）===
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_suction_cup_real_system_cross_subnet.launch.py role:=slave supervisor_mode:=true
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_suction_cup_real_system_cross_subnet.launch.py role:=master

  # === 多从臂（registry + robot_id + scheduler car 三者一致）===
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_suction_cup_real_system_cross_subnet.launch.py role:=slave robot_id:=ar5_01 supervisor_mode:=true
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_suction_cup_real_system_cross_subnet.launch.py role:=slave robot_id:=ar5_02 supervisor_mode:=true

  # === 单机测试模式 ===
  ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_suction_cup_real_system_cross_subnet.launch.py

端口规则 (ParticipantIndex=auto, DomainGain=0):
  Participant port = _PORT_BASE + ParticipantIndex
  每个节点自动分配一个独立端口；supervisor 固定占用 7003。
"""

import glob
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from slave_registry import find_slave_by_ip, generate_peers_xml, load_registry
from supervisor_launch import resolve_supervisor_cmd

_STACK_LAUNCH_FILE = 'slave_stack_suction_cup_only.launch.py'

# ═══════════════════════════════════════════════════════════════════════════════
# 部署配置（部署前修改此处）
# ═══════════════════════════════════════════════════════════════════════════════

_DOMAIN_ID = '32'

_MASTER_NETWORK_INTERFACE = "wlo1"
_MASTER_EXTERNAL_IP = "192.168.9.6"

_SLAVE_NETWORK_INTERFACE = "eno1"
_SLAVE_EXTERNAL_IP = "192.168.9.4"

_PORT_BASE = 7000
_MAX_AUTO_PARTICIPANT_INDEX = 50
_NEXUS_MANAGE_START_DELAY_SEC = 8.0
_SUPERVISOR_PARTICIPANT_INDEX = 3
_DDS_STACK_CONFIG_PREFIX = 'cyclonedds_nexus_cross_subnet_v2_stack_'
_DDS_SUPERVISOR_CONFIG_PREFIX = 'cyclonedds_nexus_cross_subnet_v2_supervisor_'

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


def _make_peers_xml(local_count: int, remote_count: int, peer_ip: str,
                    local_ip: str = '127.0.0.1') -> str:
    lines = []
    for i in range(local_count):
        lines.append(f'        <Peer Address="{local_ip}:{_PORT_BASE + i}"/>')
    for i in range(remote_count):
        lines.append(f'        <Peer Address="{peer_ip}:{_PORT_BASE + i}"/>')
    return '\n'.join(lines)


def _make_cyclonedds_xml(network_interface: str, external_ip: str,
                         peers_xml: str,
                         participant_index='auto') -> str:
    external_addr_xml = (
        f'      <ExternalNetworkAddress>{external_ip}</ExternalNetworkAddress>'
        if external_ip else ''
    )
    if participant_index == 'auto':
        participant_index_xml = (
            f'      <ParticipantIndex>auto</ParticipantIndex>\n'
            f'      <MaxAutoParticipantIndex>{_MAX_AUTO_PARTICIPANT_INDEX}</MaxAutoParticipantIndex>'
        )
    else:
        participant_index_xml = (
            f'      <ParticipantIndex>{participant_index}</ParticipantIndex>'
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
{participant_index_xml}
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


def _write_cyclonedds_config(xml_content: str, prefix: str) -> str:
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=prefix,
        suffix='.xml',
        delete=False,
    )
    tmp.write(xml_content)
    tmp.flush()
    tmp.close()
    return tmp.name


def _find_existing_stack_cyclonedds_xml():
    pattern = f'/tmp/{_DDS_STACK_CONFIG_PREFIX}*.xml'
    latest = None
    for f in glob.glob(pattern):
        if latest is None or os.path.getmtime(f) > os.path.getmtime(latest):
            latest = f
    return latest


def _find_existing_supervisor_cyclonedds_xml():
    pattern = f'/tmp/{_DDS_SUPERVISOR_CONFIG_PREFIX}*.xml'
    latest = None
    for f in glob.glob(pattern):
        if latest is None or os.path.getmtime(f) > os.path.getmtime(latest):
            latest = f
    return latest


def _legacy_peer_counts(role, local_nodes, remote_nodes):
    if role == 'master':
        return len(local_nodes), len(remote_nodes)
    if role == 'slave':
        return len(local_nodes), len(remote_nodes)
    return len(local_nodes), len(remote_nodes)


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


def _compose_launch_actions(node_specs, launch_args_by_pkg=None):
    launch_args_by_pkg = launch_args_by_pkg or {}
    regular_actions = []
    delayed_manage_action = None

    for pkg, launch_file in node_specs:
        launch_args = launch_args_by_pkg.get(pkg)
        action = _include(pkg, launch_file, launch_args)
        if pkg == 'nexus_manage':
            delayed_manage_action = TimerAction(
                period=_NEXUS_MANAGE_START_DELAY_SEC,
                actions=[action],
            )
        else:
            regular_actions.append(action)

    if delayed_manage_action is not None:
        regular_actions.append(delayed_manage_action)

    return regular_actions


def _resolve_registry_path(context):
    return PathJoinSubstitution([
        FindPackageShare('nexus_manage'), 'config', 'slave_registry.yaml',
    ]).perform(context)


def _launch_args_by_role(role, robot_id, legacy_single_slave, startup_move_to_init):
    args_by_pkg = {}
    if role == 'slave':
        effective_robot_id = robot_id or 'ar5'
        teleop_args = {'startup_move_to_init': startup_move_to_init, 'robot_id': effective_robot_id}
        args_by_pkg['teleop_adapter'] = teleop_args
        for pkg, _ in _SLAVE_NODES:
            if pkg != 'teleop_adapter':
                args_by_pkg[pkg] = {'robot_id': effective_robot_id}
    elif role == 'master':
        for pkg, _ in _MASTER_NODES:
            args_by_pkg[pkg] = {'robot_id': ''}
        if legacy_single_slave:
            args_by_pkg['nexus_manage'] = {
                'boot_selfcheck_await_scheduler': 'false',
                'robot_id': '',
            }
    return args_by_pkg


def _node_actions_for_role(role, args_by_pkg):
    if role == 'master':
        return _compose_launch_actions(_MASTER_NODES, args_by_pkg)
    if role == 'slave':
        return _compose_launch_actions(_SLAVE_NODES, args_by_pkg)
    return _compose_launch_actions(_MASTER_NODES + _SLAVE_NODES, args_by_pkg)


def _supervisor_node_action(robot_id, supervisor_dds_path, stack_dds_path):
    from launch.actions import ExecuteProcess

    cmd = resolve_supervisor_cmd(
        robot_id, stack_dds_path, stack_launch_file=_STACK_LAUNCH_FILE)
    return ExecuteProcess(
        cmd=cmd,
        output='screen',
        shell=False,
        additional_env={
            'ROS_DOMAIN_ID': _DOMAIN_ID,
            'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
            'CYCLONEDDS_URI': 'file://' + supervisor_dds_path,
        },
    )


def _resolve_stack_launch_actions(role, args_by_pkg, supervisor_mode, stack_only,
                                  robot_id, supervisor_dds_path, stack_dds_path):
    use_supervisor = role == 'slave' and supervisor_mode in ('true', '1', 'yes')
    only_stack = stack_only in ('true', '1', 'yes')
    if use_supervisor and not only_stack:
        if not supervisor_dds_path or not stack_dds_path:
            raise RuntimeError('supervisor DDS config paths missing')
        return [_supervisor_node_action(robot_id, supervisor_dds_path, stack_dds_path)]
    if role == 'slave' and only_stack:
        return _compose_launch_actions(_SLAVE_NODES, args_by_pkg)
    return _node_actions_for_role(role, args_by_pkg)


def _build_legacy_dds_configs(role, supervisor_mode, stack_only,
                                network_interface, external_ip, peer_ip,
                                local_nodes, remote_nodes):
    local_count, remote_count = _legacy_peer_counts(
        role, local_nodes, remote_nodes)
    peers_xml = _make_peers_xml(
        local_count, remote_count, peer_ip, local_ip='127.0.0.1')

    stack_xml = _make_cyclonedds_xml(
        network_interface, external_ip, peers_xml, participant_index='auto')
    stack_path = _write_cyclonedds_config(stack_xml, _DDS_STACK_CONFIG_PREFIX)

    supervisor_path = None
    use_supervisor = (
        role == 'slave'
        and supervisor_mode not in ('false', '0', 'no')
        and stack_only not in ('true', '1', 'yes')
    )
    if use_supervisor:
        supervisor_xml = _make_cyclonedds_xml(
            network_interface, external_ip, peers_xml,
            participant_index=_SUPERVISOR_PARTICIPANT_INDEX)
        supervisor_path = _write_cyclonedds_config(
            supervisor_xml, _DDS_SUPERVISOR_CONFIG_PREFIX)
        print(
            f'[cross_subnet] stack ports {_PORT_BASE}-{_PORT_BASE + len(_SLAVE_NODES) - 1} '
            f'(auto), supervisor fixed port '
            f'{_PORT_BASE + _SUPERVISOR_PARTICIPANT_INDEX} '
            f'(ParticipantIndex={_SUPERVISOR_PARTICIPANT_INDEX})'
        )
    else:
        print(
            f'[cross_subnet] DDS peers: local={local_count} remote={remote_count} '
            f'(stack ports {_PORT_BASE}-{_PORT_BASE + local_count - 1})'
        )

    return stack_path, supervisor_path


def launch_setup(context, *args, **kwargs):
    role = LaunchConfiguration('role').perform(context).strip().lower()
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    startup_move_to_init = LaunchConfiguration('startup_move_to_init').perform(context).strip()
    supervisor_mode = LaunchConfiguration('supervisor_mode').perform(context).strip().lower()
    stack_only = LaunchConfiguration('stack_only').perform(context).strip().lower()

    registry_path = _resolve_registry_path(context)
    slaves = load_registry(registry_path)
    legacy_single_slave = not slaves

    use_supervisor = (
        role == 'slave'
        and supervisor_mode not in ('false', '0', 'no')
        and stack_only not in ('true', '1', 'yes')
    )

    stack_dds_path = _find_existing_stack_cyclonedds_xml()
    supervisor_dds_path = _find_existing_supervisor_cyclonedds_xml() if use_supervisor else None

    if stack_dds_path:
        print(f'[cross_subnet] 复用 stack DDS 配置: {stack_dds_path}')
        if use_supervisor:
            if supervisor_dds_path:
                print(f'[cross_subnet] 复用 supervisor DDS 配置: {supervisor_dds_path}')
            else:
                print('[cross_subnet] 未找到 supervisor DDS 配置，将重新生成 (port 7003)')

    if not stack_dds_path:
        if slaves:
            if not role:
                matched = find_slave_by_ip(slaves)
                role = 'slave' if matched else 'master'

            if role == 'master':
                network_interface = _MASTER_NETWORK_INTERFACE
                external_ip = _MASTER_EXTERNAL_IP
                peers_xml = generate_peers_xml(
                    slaves,
                    local_count=len(_MASTER_NODES),
                    remote_count=len(_SLAVE_NODES),
                    port_base=_PORT_BASE,
                    local_ip='127.0.0.1',
                )
                stack_xml = _make_cyclonedds_xml(
                    network_interface, external_ip, peers_xml)
                stack_dds_path = _write_cyclonedds_config(
                    stack_xml, _DDS_STACK_CONFIG_PREFIX)
                supervisor_dds_path = None
            elif role == 'slave':
                matched = find_slave_by_ip(slaves)
                if not matched:
                    raise RuntimeError(
                        'Cannot determine slave identity: '
                        'no registry entry matches local IPs')
                network_interface = matched['network_interface']
                external_ip = matched['ip']
                peers_xml = _make_peers_xml(
                    len(_SLAVE_NODES), len(_MASTER_NODES), _MASTER_EXTERNAL_IP,
                    local_ip='127.0.0.1')
                stack_xml = _make_cyclonedds_xml(
                    network_interface, external_ip, peers_xml)
                stack_dds_path = _write_cyclonedds_config(
                    stack_xml, _DDS_STACK_CONFIG_PREFIX)
                if use_supervisor:
                    supervisor_xml = _make_cyclonedds_xml(
                        network_interface, external_ip, peers_xml,
                        participant_index=_SUPERVISOR_PARTICIPANT_INDEX)
                    supervisor_dds_path = _write_cyclonedds_config(
                        supervisor_xml, _DDS_SUPERVISOR_CONFIG_PREFIX)
            else:
                raise RuntimeError(f'Unknown role: {role}')
        else:
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

            stack_dds_path, supervisor_dds_path = _build_legacy_dds_configs(
                role, supervisor_mode, stack_only,
                network_interface, external_ip, peer_ip,
                local_nodes, remote_nodes)

    elif use_supervisor and not supervisor_dds_path:
        peers_xml = _make_peers_xml(
            len(_SLAVE_NODES), len(_MASTER_NODES), _MASTER_EXTERNAL_IP,
            local_ip='127.0.0.1')
        supervisor_xml = _make_cyclonedds_xml(
            _SLAVE_NETWORK_INTERFACE, _SLAVE_EXTERNAL_IP, peers_xml,
            participant_index=_SUPERVISOR_PARTICIPANT_INDEX)
        supervisor_dds_path = _write_cyclonedds_config(
            supervisor_xml, _DDS_SUPERVISOR_CONFIG_PREFIX)
        print(
            f'[cross_subnet] 补建 supervisor DDS (port '
            f'{_PORT_BASE + _SUPERVISOR_PARTICIPANT_INDEX}): {supervisor_dds_path}'
        )

    env_actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', _DOMAIN_ID),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
    ]

    if not use_supervisor:
        env_actions.append(
            SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + stack_dds_path))

    args_by_pkg = _launch_args_by_role(
        role, robot_id, legacy_single_slave, startup_move_to_init)
    launch_actions = _resolve_stack_launch_actions(
        role, args_by_pkg, supervisor_mode, stack_only, robot_id,
        supervisor_dds_path, stack_dds_path)

    return env_actions + launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description=(
                'Slave only: namespace prefix (default ar5 when role=slave). '
                'Ignored on master — do not pass robot_id when role:=master.'
            ),
        ),
        DeclareLaunchArgument(
            'role',
            default_value='',
            description=(
                'Launch role: "slave" (AR5 side), "master" (Nexus-Arm side), '
                'or empty string to launch all components on a single machine.'
            )
        ),
        DeclareLaunchArgument(
            'startup_move_to_init',
            default_value='false',
            description=(
                'Slave only: if true, MoveJ to init pose on startup (debug); '
                'if false, keep current pose (field deployment).'
            ),
        ),
        DeclareLaunchArgument(
            'supervisor_mode',
            default_value='true',
            description=(
                'Slave only: if true, launch slave_stack_supervisor instead of '
                'full teleop stack (scheduler-driven start/stop).'
            ),
        ),
        DeclareLaunchArgument(
            'stack_only',
            default_value='false',
            description='Internal: launch slave stack without supervisor (supervisor subprocess).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
