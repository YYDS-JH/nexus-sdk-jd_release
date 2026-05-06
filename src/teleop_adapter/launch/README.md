# Teleop Arm Driver Launch 使用说明

## 概述

本 launch 文件用于启动单机械臂远程操控驱动节点，自动加载配置文件并初始化硬件连接。

## 文件说明

### Launch 文件
- **文件路径**: `launch/table_arm_single.launch.py`
- **功能**: 启动 `teleop_arm_driver_main` 节点并加载配置

### 配置文件
- **文件路径**: `config/table_arm_single.yaml`
- **功能**: 定义机械臂参数、串口配置、主题名称等

## 使用方法

### 1. 编译工作空间

```bash
cd /home/qj00431/infra/teleop-control-sdk
colcon build --packages-select teleop_adapter
source install/setup.bash
```

### 2. 启动节点

```bash
ros2 launch teleop_adapter table_arm_single.launch.py
```

### 3. 验证节点运行

查看节点列表：
```bash
ros2 node list
# 应该看到: /teleop_arm_adapter
```

查看发布的主题：
```bash
ros2 topic list
# 应该看到:
# - /rt/teleop/table_arm/joint_state (关节状态反馈)
# - /rt/teleop/table_arm/status (机械臂状态)
```

查看订阅的主题：
```bash
ros2 topic info /rt/teleop/table_arm/joint_cmd
# 应该看到 teleop_arm_adapter 订阅了这个主题
```

### 4. 发送测试命令

```bash
# 发送零位命令
ros2 topic pub --once /rt/teleop/table_arm/joint_cmd teleop_msg/msg/JointMitControl \
"{joint_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  joint_velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  torque: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  kp: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0],
  kd: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]}"
```

### 5. 监控关节状态

```bash
ros2 topic echo /rt/teleop/table_arm/joint_state
```

### 6. 监控机械臂状态

```bash
ros2 topic echo /rt/teleop/table_arm/status
```

## 配置参数说明

### 机械臂配置
- `right_arm_uart_port`: 串口设备路径 (例如: `/dev/ttyS10`)
- `right_arm_feedback_rate`: 反馈频率 (Hz)
- `right_arm_num_of_dofs`: 该机械臂的自由度数量

### 全局参数
- `auto_enable`: 启动时是否自动使能电机
- `control_type`: 控制类型 (`closed_loop` 或 `open_loop`)
- `num_of_dofs`: 总自由度数量
- `default_dof_pos`: 默认关节位置 (弧度)
- `joint_names`: 关节名称列表
- `joint_mapping`: ROS顺序到设备顺序的映射

### 主题配置
- `joint_mit_control_topic`: 关节命令主题
- `joint_state_topic`: 关节状态主题
- `status_topic`: 机械臂状态主题

## 故障排查

### 1. 串口权限问题

如果出现权限错误，添加当前用户到 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

或临时修改串口权限：

```bash
sudo chmod 666 /dev/ttyS10
```

### 2. 节点无法启动

检查配置文件路径是否正确：

```bash
ros2 pkg prefix teleop_adapter
ls $(ros2 pkg prefix teleop_adapter)/share/teleop_adapter/config/
```

### 3. 查看详细日志

```bash
ros2 launch teleop_adapter table_arm_single.launch.py --ros-args --log-level debug
```

### 4. 检查串口设备

```bash
ls -l /dev/ttyS*
# 确认 /dev/ttyS10 是否存在
```

## 扩展使用

### 修改日志级别

编辑 launch 文件中的日志级别参数：

```python
arguments=['--ros-args', '--log-level', 'debug']  # info, warn, error, debug
```

### 使用不同的配置文件

复制并修改 `table_arm_single.yaml`，然后在 launch 文件中更新路径：

```python
config_file = os.path.join(pkg_dir, 'config', 'your_config.yaml')
```

### 多机械臂配置

如需控制多个机械臂，在 YAML 文件中添加额外的机械臂配置：

```yaml
# 在 table_arm_single.yaml 中添加
left_arm_uart_port: '/dev/ttyS11'
left_arm_feedback_rate: 400
left_arm_num_of_dofs: 6

# 并更新全局参数
num_of_dofs: 12  # 6 + 6
joint_names: ["right_joint1", ..., "left_joint1", ...]
joint_mapping: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
```

## 节点架构

```
table_arm_single.launch.py
    ↓
teleop_arm_driver_main
    ↓
TeleopArmDriverNode
    ├── 参数加载 (loadGlobalConfig)
    ├── 机械臂发现 (discoverAndLoadArmConfigs)
    ├── 配置验证 (validateConfiguration)
    └── 适配器初始化 (initializeAdapters)
        └── TeleopArmAdapter
            └── SerialPort (/dev/ttyS10)
```

## 相关命令

```bash
# 停止节点
Ctrl+C

# 查看节点信息
ros2 node info /teleop_arm_adapter

# 查看参数
ros2 param list /teleop_arm_adapter

# 动态修改参数（如果支持）
ros2 param set /teleop_arm_adapter auto_enable false
```
