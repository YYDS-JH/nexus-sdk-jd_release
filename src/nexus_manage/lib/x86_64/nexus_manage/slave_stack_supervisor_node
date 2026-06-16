#!/usr/bin/env python3
"""
从臂栈监管节点 — 根据调度 /cl/operator_status 启停从臂 ROS 进程，并发布 /cl/yj/operator。

方案：
  connected=false + mode=idle  → 释放 RCI → 停止从臂栈 → status=online
  connected=true  + mode=teleop → 启动从臂栈 → 就绪后 status=offline

监管节点自身常驻，不被停止；管理的子进程为 teleop_adapter + human_data + robot_controller。
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from infra_msg.srv import RciControl
except ImportError:  # pragma: no cover
    RciControl = None

OPERATOR_STATUS_TOPIC = '/cl/operator_status'
YJ_OPERATOR_TOPIC = '/cl/yj/operator'


class StackPhase(str, Enum):
    STOPPED = 'stopped'
    STARTING = 'starting'
    RUNNING = 'running'
    STOPPING = 'stopping'


class SlaveStackSupervisor(Node):
  def __init__(self):
    super().__init__('slave_stack_supervisor')

    self.declare_parameter('operator_status_topic', OPERATOR_STATUS_TOPIC)
    self.declare_parameter('yj_operator_topic', YJ_OPERATOR_TOPIC)
    self.declare_parameter('stack_launch_package', 'nexus_manage')
    self.declare_parameter('stack_launch_file', 'slave_stack_only.launch.py')
    self.declare_parameter('robot_id', '')
    self.declare_parameter('target_car', '')
    self.declare_parameter('yj_device_id', 'YJ-RC-005')
    self.declare_parameter('yj_target_ip', '')
    self.declare_parameter('rci_control_service', 'robot/rci_control')
    self.declare_parameter('stack_ready_service', 'robot/robot_controller_config')
    self.declare_parameter('stack_start_timeout_sec', 90.0)
    self.declare_parameter('stack_stop_timeout_sec', 15.0)
    self.declare_parameter('publish_hz', 1.0)
    self.declare_parameter('auto_start_on_boot', False)

    self._operator_topic = self.get_parameter('operator_status_topic').value
    self._yj_topic = self.get_parameter('yj_operator_topic').value
    self._stack_pkg = self.get_parameter('stack_launch_package').value
    self._stack_launch = self.get_parameter('stack_launch_file').value
    self._robot_id = str(self.get_parameter('robot_id').value).strip()
    self._target_car = str(self.get_parameter('target_car').value).strip()
    self._yj_device_id = self.get_parameter('yj_device_id').value
    self._yj_target_ip = str(self.get_parameter('yj_target_ip').value).strip()
    self._rci_service = self._resolve_service_name(
        str(self.get_parameter('rci_control_service').value))
    self._ready_service = self._resolve_service_name(
        str(self.get_parameter('stack_ready_service').value))
    self._start_timeout = float(self.get_parameter('stack_start_timeout_sec').value)
    self._stop_timeout = float(self.get_parameter('stack_stop_timeout_sec').value)
    self._auto_start = bool(self.get_parameter('auto_start_on_boot').value)

    self._phase = StackPhase.STOPPED
    self._stack_proc: Optional[subprocess.Popen] = None
    self._lock = threading.Lock()
    self._last_operator: dict = {}
    self._last_connected = False
    self._last_mode = ''
    self._last_car = ''
    self._last_ip = ''
    self._error_code = 0
    self._task_state = 'idle'
    self._desired_stack: Optional[str] = None  # 'start' | 'stop' | None

    self._pub_yj = self.create_publisher(String, self._yj_topic, 10)
    self.create_subscription(String, self._operator_topic, self._on_operator_status, 10)

    hz = max(float(self.get_parameter('publish_hz').value), 0.1)
    self.create_timer(1.0 / hz, self._publish_yj_status)

    self.get_logger().info(
        f'Slave stack supervisor ready: stack={self._stack_pkg}/{self._stack_launch} '
        f'robot_id={self._robot_id or "(default)"} rci={self._rci_service}')

    if self._auto_start:
      self.get_logger().warn('auto_start_on_boot=true: starting slave stack immediately')
      self._start_stack_async()

  def _resolve_service_name(self, suffix: str) -> str:
    suffix = suffix.lstrip('/')
    if self._robot_id:
      return f'/{self._robot_id}/{suffix}'
    return f'/{suffix}'

  def _on_operator_status(self, msg: String):
    try:
      data = json.loads(msg.data)
    except json.JSONDecodeError:
      self.get_logger().warn('Invalid JSON on operator_status, ignoring')
      return

    connected = bool(data.get('connected', False))
    mode = str(data.get('mode', ''))
    car = str(data.get('car', ''))
    ip = str(data.get('ip', ''))

    if self._target_car and car and car != self._target_car:
      return

    self._last_operator = data
    self._last_connected = connected
    self._last_mode = mode
    self._last_car = car
    if ip:
      self._last_ip = ip

    if (not connected) and mode == 'idle':
      desired = 'stop'
    elif connected and mode == 'teleop':
      desired = 'start'
    else:
      desired = None

    if desired is None or desired == self._desired_stack:
      return
    self._desired_stack = desired

    if desired == 'stop':
      self.get_logger().info(
          f'Scheduler release: connected={connected} mode={mode} car={car}')
      self._stop_stack_async()
    elif desired == 'start':
      self.get_logger().info(
          f'Scheduler acquire: connected={connected} mode={mode} car={car}')
      self._start_stack_async()

  def _start_stack_async(self):
    threading.Thread(target=self._start_stack, daemon=True).start()

  def _stop_stack_async(self):
    threading.Thread(target=self._stop_stack, daemon=True).start()

  def _start_stack(self):
    with self._lock:
      if self._phase in (StackPhase.RUNNING, StackPhase.STARTING):
        self.get_logger().info('Slave stack already running or starting')
        return
      self._phase = StackPhase.STARTING
      self._task_state = 'starting'

    cmd = ['ros2', 'launch', self._stack_pkg, self._stack_launch]
    if self._robot_id:
      cmd.append(f'robot_id:={self._robot_id}')

    self.get_logger().info(f'Starting slave stack: {" ".join(cmd)}')
    try:
      proc = subprocess.Popen(
          cmd,
          stdout=None,
          stderr=None,
          preexec_fn=os.setsid,
          text=True,
      )
    except OSError as exc:
      self.get_logger().error(f'Failed to start slave stack: {exc}')
      with self._lock:
        self._phase = StackPhase.STOPPED
        self._error_code = 1
        self._task_state = 'start_failed'
      return

    with self._lock:
      self._stack_proc = proc

    if not self._wait_stack_ready():
      self.get_logger().error('Slave stack failed readiness check, stopping')
      self._terminate_stack_process()
      with self._lock:
        self._phase = StackPhase.STOPPED
        self._stack_proc = None
        self._error_code = 2
        self._task_state = 'start_failed'
      return

    with self._lock:
      self._phase = StackPhase.RUNNING
      self._error_code = 0
      self._task_state = 'teleop_stack_running'
      self._desired_stack = 'start'
    self.get_logger().info('Slave stack running — yj status=offline')

  def _stop_stack(self):
    with self._lock:
      if self._phase == StackPhase.STOPPED and self._stack_proc is None:
        self.get_logger().info('Slave stack already stopped')
        self._task_state = 'released'
        self._desired_stack = 'stop'
        return
      self._phase = StackPhase.STOPPING
      self._task_state = 'stopping'

    self._release_rci()
    self._terminate_stack_process()

    with self._lock:
      self._phase = StackPhase.STOPPED
      self._stack_proc = None
      self._error_code = 0
      self._task_state = 'released'
      self._desired_stack = 'stop'
    self.get_logger().info('Slave stack stopped — yj status=online')

  def _release_rci(self):
    if RciControl is None:
      self.get_logger().warn('infra_msg RciControl unavailable, skip RCI release')
      return

    client = self.create_client(RciControl, self._rci_service)
    if not client.wait_for_service(timeout_sec=3.0):
      self.get_logger().warn(f'RCI service not available: {self._rci_service}')
      return

    request = RciControl.Request()
    request.command = RciControl.Request.CMD_RELEASE
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
    if not future.done():
      self.get_logger().warn('RCI release call timeout')
      return
    try:
      response = future.result()
    except Exception as exc:  # pragma: no cover
      self.get_logger().warn(f'RCI release failed: {exc}')
      return
    if response and response.success:
      self.get_logger().info(f'RCI released: {response.message}')
    else:
      msg = response.message if response else 'null response'
      self.get_logger().warn(f'RCI release failed: {msg}')

  def _wait_stack_ready(self) -> bool:
    deadline = time.monotonic() + self._start_timeout
    while time.monotonic() < deadline:
      with self._lock:
        proc = self._stack_proc
      if proc is not None and proc.poll() is not None:
        self.get_logger().error('Slave stack process exited during startup')
        return False
      if self._service_ready(self._ready_service):
        return True
      time.sleep(0.5)
    self.get_logger().error(
        f'Timeout waiting for {self._ready_service} ({self._start_timeout}s)')
    return False

  def _service_ready(self, service_name: str) -> bool:
    result = subprocess.run(
        ['ros2', 'service', 'list'],
        capture_output=True,
        text=True,
        timeout=5.0,
        check=False,
    )
    if result.returncode != 0:
      return False
    needle = service_name if service_name.startswith('/') else f'/{service_name}'
    return any(line.strip() == needle for line in result.stdout.splitlines())

  def _terminate_stack_process(self):
    with self._lock:
      proc = self._stack_proc
    if proc is None:
      return
    if proc.poll() is not None:
      return
    try:
      os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
      return
    try:
      proc.wait(timeout=self._stop_timeout)
    except subprocess.TimeoutExpired:
      self.get_logger().warn('Slave stack SIGINT timeout, sending SIGKILL')
      try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
      except ProcessLookupError:
        pass
      proc.wait(timeout=5.0)

  def _publish_yj_status(self):
    with self._lock:
      phase = self._phase
      error_code = self._error_code

    # 栈已停止 / 让出控制权 → online；遥操栈运行中 → offline
    stack_active = phase == StackPhase.RUNNING
    status = 'offline' if stack_active else 'online'
    target_car = self._target_car or self._last_car or self._robot_id
    target_ip = self._yj_target_ip or self._last_ip

    payload = {
        'device_id': self._yj_device_id,
        'target_car': target_car,
        'target_ip': target_ip,
        'connected': bool(stack_active and self._last_connected),
        'status': status,
        'signal_dbm': -50,
        'e_stop_pressed': False,
        'control_latency_ms': 0,
        'error_code': error_code,
    }
    msg = String()
    msg.data = json.dumps(payload, ensure_ascii=False)
    self._pub_yj.publish(msg)

  def destroy_node(self):
    self._terminate_stack_process()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = SlaveStackSupervisor()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
