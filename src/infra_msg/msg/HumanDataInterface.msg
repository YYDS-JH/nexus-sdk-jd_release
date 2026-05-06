# HumanDataInterface.msg
# 人体数据接口消息

# 消息头
std_msgs/Header header

EndPoseData[] robot_end_pos

# 系统事件
uint8 system_event
uint8 EVENT_NONE = 0                  # 无事件
uint8 EVENT_RESET_MASTER_ARM = 1      # 遥操主臂复位