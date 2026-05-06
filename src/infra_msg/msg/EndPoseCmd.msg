# EndPoseCmd.msg
# 末端位姿命令消息

# 末端名称
# 例如: "left_wrist", "right_wrist", "left_end_effector", "right_end_effector"
string name

# 末端类型标识
# 例如: "ee", "gripper"
string type

# 齐次变换矩阵 (4x4展平为16个元素)
# 表示末端执行器的位置和姿态
# [R11, R12, R13, Tx]
# [R21, R22, R23, Ty]
# [R31, R32, R33, Tz]
# [0,   0,   0,   1 ]
float64[16] homogeneous_matrix

# 夹爪/末端执行器开合度
float64 gripper
