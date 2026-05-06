# EndEffectorConfig.msg
# 机器人末端配置消息
# 用于描述单个末端执行器或腕部关节的配置信息

# 末端名称/标识符
# 例如: "left_wrist", "right_wrist", "left_end_effector", "right_end_effector"
string name

# 末端类型
# 例如: "wrist", "gripper", "end_effector", "hand"
string type

# 位置/值数组
# - 对于腕部(wrist): 关节角度数组，单位: rad，数量取决于关节数（通常6个）
# - 对于末端执行器(gripper/hand): 执行器值，范围: 0.0-1.0，数量取决于自由度
float64[] values
