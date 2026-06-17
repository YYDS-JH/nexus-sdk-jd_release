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
    self.declare_parameter('stack_stop_timeout_sec', 30.0)
    self.declare_parameter('stack_stop_settle_sec', 3.0)
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
    self._stop_settle_sec = float(self.get_parameter('stack_stop_settle_sec').value)
    self._auto_start = bool(self.get_parameter('auto_start_on_boot').value)

    self._phase = StackPhase.STOPPED
    self._stack_proc: Optional[subprocess.Popen] = None
    self._lock = threading.Lock()
    self._stack_op_lock = threading.Lock()
    self._last_operator: dict = {}
    self._last_connected = False
    self._last_mode = ''
    self._last_car = ''
    self._last_ip = ''
    self._error_code = 0
    self._task_state = 'idle'
    self._desired_stack: Optional[str] = None  # 'start' | 'stop' | None
    self._start_abort = threading.Event()

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

    if desired is None:
      return
    if desired == self._desired_stack:
      return
    # 启动失败/崩溃后 _desired_stack 仍为 'start' 且已 STOPPED：忽略调度 1Hz 重试
    with self._lock:
      phase = self._phase
      task_state = self._task_state
    if (desired == 'start' and self._desired_stack == 'start'
            and phase == StackPhase.STOPPED
            and task_state in ('start_failed', 'stack_crashed')):
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
    with self._stack_op_lock:
      self._start_stack_impl()

  def _stop_stack(self):
    with self._stack_op_lock:
      self._stop_stack_impl()

  def _start_stack_impl(self):
    with self._lock:
      if self._phase in (StackPhase.RUNNING, StackPhase.STARTING):
        self.get_logger().info('Slave stack already running or starting')
        return
      self._phase = StackPhase.STARTING
      self._task_state = 'starting'
    self._start_abort.clear()

    self._ensure_previous_stack_terminated()

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
        self._desired_stack = 'start'
      return

    with self._lock:
      self._stack_proc = proc

    if not self._wait_stack_ready():
      if self._start_abort.is_set():
        self.get_logger().info('Slave stack start cancelled (stop requested)')
        return
      self.get_logger().error('Slave stack failed readiness check, stopping')
      self._terminate_stack_process()
      self._wait_until_service_gone(timeout=10.0)
      with self._lock:
        self._phase = StackPhase.STOPPED
        self._stack_proc = None
        self._error_code = 2
        self._task_state = 'start_failed'
        self._desired_stack = 'start'
      return

    with self._lock:
      if self._start_abort.is_set():
        return
      self._phase = StackPhase.RUNNING
      self._error_code = 0
      self._task_state = 'teleop_stack_running'
      self._desired_stack = 'start'
    self.get_logger().info('Slave stack running — yj status=offline')

  def _stop_stack_impl(self):
    self._start_abort.set()
    with self._lock:
      if self._phase == StackPhase.STOPPED and self._stack_proc is None:
        self.get_logger().info('Slave stack already stopped')
        self._task_state = 'released'
        self._desired_stack = 'stop'
        self._start_abort.clear()
        return
      self._phase = StackPhase.STOPPING
      self._task_state = 'stopping'

    self._release_rci()
    self._terminate_stack_process()
    self._wait_until_service_gone(timeout=max(self._stop_timeout, 10.0))
    if self._stop_settle_sec > 0:
      time.sleep(self._stop_settle_sec)

    with self._lock:
      self._phase = StackPhase.STOPPED
      self._stack_proc = None
      self._error_code = 0
      self._task_state = 'released'
      self._desired_stack = 'stop'
    self._start_abort.clear()
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

  def _ensure_previous_stack_terminated(self):
    """杀栈后确保旧 launch 进程组已退出，避免重启抢资源或节点名冲突。"""
    with self._lock:
      proc = self._stack_proc
    if proc is not None and proc.poll() is None:
      self.get_logger().warn('Previous stack process still alive, terminating before restart')
      self._terminate_stack_process()
    with self._lock:
      self._stack_proc = None
    self._wait_until_service_gone(timeout=10.0)

  def _wait_stack_ready(self) -> bool:
    deadline = time.monotonic() + self._start_timeout
    while time.monotonic() < deadline:
      if self._start_abort.is_set():
        return False
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

  def _wait_until_service_gone(self, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      if not self._service_ready(self._ready_service):
        return
      time.sleep(0.3)
    self.get_logger().warn(
        f'Service still listed after stop (may be stale DDS): {self._ready_service}')

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
      with self._lock:
        self._stack_proc = None
      return
    try:
      os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
      with self._lock:
        self._stack_proc = None
      return
    try:
      half = max(self._stop_timeout / 2.0, 5.0)
      proc.wait(timeout=half)
    except subprocess.TimeoutExpired:
      self.get_logger().warn('Slave stack SIGINT timeout, sending SIGTERM')
      try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
      except ProcessLookupError:
        pass
      try:
        proc.wait(timeout=half)
      except subprocess.TimeoutExpired:
        self.get_logger().warn('Slave stack SIGTERM timeout, sending SIGKILL')
        try:
          os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
          pass
        proc.wait(timeout=5.0)
    with self._lock:
      self._stack_proc = None

  def _check_stack_process_health(self):
    """栈意外退出时不自动重启，等调度 idle→teleop 再试。"""
    with self._lock:
      proc = self._stack_proc
      phase = self._phase
    if proc is None or phase not in (StackPhase.RUNNING, StackPhase.STARTING):
      return
    if proc.poll() is None:
      return
    self.get_logger().error(
        'Slave stack process exited unexpectedly (code=%s)', proc.returncode)
    with self._lock:
      self._phase = StackPhase.STOPPED
      self._stack_proc = None
      self._error_code = 3
      self._task_state = 'stack_crashed'
      if self._desired_stack != 'stop':
        self._desired_stack = 'start'

  def _publish_yj_status(self):
    self._check_stack_process_health()

    with self._lock:
      phase = self._phase
      task_state = self._task_state
      error_code = self._error_code

    stack_running = phase in (StackPhase.RUNNING, StackPhase.STARTING)
    # 新语义：栈停止=online（让出控制权），栈运行=offline（遥操占用）
    status = 'offline' if stack_running else 'online'
    target_car = self._target_car or self._last_car or self._robot_id
    target_ip = self._yj_target_ip or self._last_ip

    payload = {
        'device_id': self._yj_device_id,
        'target_car': target_car,
        'target_ip': target_ip,
        'connected': self._last_connected and stack_running,
        'status': status,
        'task_state': task_state,
        'scheduler_mode': self._last_mode,
        'stack_phase': phase.value,
        'battery_pct': 100,
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
