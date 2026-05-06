# MotorStatus.msg
# 单个电机的状态信息

# 电机温度 (摄氏度)
float32 temperature

# 电机是否在线
bool is_online

# 电机当前工作模式
# 0: 空闲模式 (IDLE)
# 1: 位置控制模式 (POSITION_CONTROL) 
# 2: 速度控制模式 (VELOCITY_CONTROL)
# 3: 力矩控制模式 (TORQUE_CONTROL)
# 4: MIT控制模式 (MIT_CONTROL)
# 5: 错误模式 (ERROR)
uint8 current_mode

# 电机通讯丢包率 (0.0-1.0, 0表示无丢包，1表示完全丢包)
float32 packet_loss_rate

# 电机ID或索引
uint8 motor_id

# 电机名称 (可选，用于调试)
string motor_name

# 常量定义 - 电机工作模式
uint8 MODE_IDLE = 0
uint8 MODE_POSITION_CONTROL = 1  
uint8 MODE_VELOCITY_CONTROL = 2
uint8 MODE_TORQUE_CONTROL = 3
uint8 MODE_MIT_CONTROL = 4
uint8 MODE_ERROR = 5