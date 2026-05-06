# RobotEECmd.msg
# 机器人末端执行器命令消息
# 包含多个末端的位姿命令

# 消息头
std_msgs/Header header

# 末端位姿命令数组
# 包含所有需要控制的末端（腕部、夹爪等）的命令
EndPoseCmd[] cmd
