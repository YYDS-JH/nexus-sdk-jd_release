# TeleOpArmStatus.msg  
# 机械臂遥操作状态消息，包含设备连接状态、整体状态和各电机详细状态

# 消息头
std_msgs/Header header

# 机械臂名称/ID
string arm_name

# 机械臂型号 (如 "Nexus-Arm V15", "Y1" 等)
string arm_model

# === 设备连接状态 ===
# 0: 无连接 (NO_CONNECTION)
# 1: 有线连接 (WIRED_CONNECTION) 
# 2: 无线连接 (WIRELESS_CONNECTION)
uint8 connection_status

# === 设备整体状态 ===  
# 0: 正常 (NORMAL)
# 1: 警告 (WARNING) 
# 2: 故障 (FAULT)
# 3: 离线 (OFFLINE)
# 4: 初始化中 (INITIALIZING)
uint8 device_status

# === 通讯统计 ===
# 机械臂整体通讯丢包数统计 (累计)
uint64 total_packet_loss_count

# 机械臂通讯总包数统计 (累计)
uint64 total_packet_count  

# 机械臂整体通讯丢包率 (0.0-1.0)
float32 overall_packet_loss_rate

# === 错误信息 ===
# 错误代码 (0表示无错误)
int32 error_code

# 错误描述信息
string error_message

# === 各电机状态数组 ===
# 包含该机械臂所有电机的详细状态
MotorStatus[] motor_status_array

# === 常量定义 - 连接状态 ===
uint8 CONNECTION_NO_CONNECTION = 0
uint8 CONNECTION_WIRED = 1  
uint8 CONNECTION_WIRELESS = 2

# === 常量定义 - 设备状态 ===
uint8 DEVICE_STATUS_NORMAL = 0
uint8 DEVICE_STATUS_WARNING = 1
uint8 DEVICE_STATUS_FAULT = 2
uint8 DEVICE_STATUS_OFFLINE = 3
uint8 DEVICE_STATUS_INITIALIZING = 4