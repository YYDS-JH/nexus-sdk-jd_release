"""Resolve slave_stack_supervisor entrypoint across install layouts."""

from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_prefix


def _exists(path: str) -> bool:
    return bool(path) and os.path.isfile(path)


def resolve_supervisor_cmd(robot_id: str = '') -> list[str]:
    """Return argv prefix to start slave_stack_supervisor (before --ros-args)."""
    prefix = get_package_prefix('nexus_manage')
    share_script = os.path.join(
        prefix, 'share', 'nexus_manage', 'scripts', 'slave_stack_supervisor_node.py')
    lib_exec = os.path.join(prefix, 'lib', 'nexus_manage', 'slave_stack_supervisor_node')

    if _exists(share_script):
        entry = ['python3', share_script]
    elif _exists(lib_exec):
        entry = [lib_exec]
    else:
        # 开发/未 install 时回退到源码树（launch 目录的上级 scripts/）
        launch_dir = os.path.dirname(os.path.abspath(__file__))
        src_script = os.path.normpath(
            os.path.join(launch_dir, '..', 'scripts', 'slave_stack_supervisor_node.py'))
        if _exists(src_script):
            entry = ['python3', src_script]
        else:
            raise RuntimeError(
                'slave_stack_supervisor not found. Rebuild on this machine:\n'
                '  rm -rf build/nexus_manage install/nexus_manage\n'
                '  colcon build --packages-select nexus_manage\n'
                f'  expected one of:\n'
                f'    {share_script}\n'
                f'    {lib_exec}')

    ros_args = [
        '--ros-args',
        '-r', '__node:=slave_stack_supervisor',
        '-p', 'stack_launch_package:=nexus_manage',
        '-p', 'stack_launch_file:=slave_stack_only.launch.py',
        '-p', 'auto_start_on_boot:=false',
    ]
    if robot_id:
        ros_args.extend(['-p', f'robot_id:={robot_id}', '-p', f'target_car:={robot_id}'])
    return entry + ros_args
