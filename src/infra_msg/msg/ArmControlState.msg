# ArmControlState.msg
# arm_control_node 控制节点的状态信息

# 消息头
std_msgs/Header header

# === 状态机状态 ===
# 0: 启动自检 (STARTUP_CHECK)
# 1: 空闲 (IDLE)
# 2: 遥操初始化/复位 (TELEOP_INIT)
# 3: 遥操运行 (TELEOP)
# 4: 故障 (FAULT)
uint8 state

# === 控制器状态 ===
# 控制器初始化状态数组（每个机械臂对应一个控制器）
bool[] controller_initialized

# 控制器错误标志数组
bool[] controller_error

# === 设备健康状态 ===
# 主臂关节状态消息是否正常接收
bool joint_state_ok

# 从臂阻力消息是否正常接收
bool cartesian_impedance_ok

# 主臂设备状态消息是否正常接收
bool arm_status_ok

# === 错误信息 ===
# 错误代码 (0表示无错误)
int32 error_code

# 错误描述信息
string error_message

# === 故障恢复标志 ===
# 是否允许从故障状态恢复
bool can_recover

# === 时间戳 ===
# 节点进入当前状态的时间
builtin_interfaces/Time state_entry_time

# === 常量定义 - 状态机状态 ===
uint8 STATE_STARTUP_CHECK = 0
uint8 STATE_IDLE = 1
uint8 STATE_TELEOP_INIT = 2
uint8 STATE_TELEOP = 3
uint8 STATE_FAULT = 4
