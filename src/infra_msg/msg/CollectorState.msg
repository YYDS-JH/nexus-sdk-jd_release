# 数据采集器状态消息
# 发布策略: 1) 每秒强制发布一次(心跳)  2) 状态变更时立即发布

std_msgs/Header header

# 状态机状态
uint8 state
uint8 STATE_IDLE = 0
uint8 STATE_READY = 1
uint8 STATE_RECORDING = 2
uint8 STATE_FAULT = 3

# 当前任务信息 (仅在有任务时有效)
string dataset_name

# 简要进度 (详细进度通过服务查询)
uint32 total_episodes
uint32 completed_episodes
uint32 remaining_episodes

# 错误信息 (仅在 FAULT 状态有效)
int32 error_code
string error_message
