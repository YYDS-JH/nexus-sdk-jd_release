# 遥操作管理器状态消息
# 发布策略: 1) 状态变更时立即发布  2) 每秒强制发布一次(心跳)

std_msgs/Header header

# 状态机状态
uint8 state
uint8 STATE_BOOT_SELF_CHECK = 0
uint8 STATE_IDLE = 1
uint8 STATE_RESET = 2
uint8 STATE_RESET_COMPLETE = 3
uint8 STATE_POSITION_HOLD = 4
uint8 STATE_TELEOP_RUNNING = 5
uint8 STATE_TELEOP_PAUSED = 6
uint8 STATE_FAULT = 7
uint8 STATE_MODEL_INFERENCE = 8

# 状态名称(字符串形式，便于调试)
string state_name

# 错误信息 (仅在 FAULT 状态有效)
int32 error_code
string error_message
