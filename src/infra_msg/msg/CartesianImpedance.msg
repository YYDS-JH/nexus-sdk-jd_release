# CartesianImpedance.msg
# 笛卡尔空间阻力消息，用于传输从臂末端受到的外力和外力矩

# 消息头
std_msgs/Header header

# 机械臂名称/ID
string arm_name

# === 笛卡尔空间力（单位：牛顿 N）===
# X 方向的力
float64 force_x

# Y 方向的力
float64 force_y

# Z 方向的力
float64 force_z

# === 笛卡尔空间力矩（单位：牛顿米 Nm）===
# 绕 X 轴的力矩
float64 torque_x

# 绕 Y 轴的力矩
float64 torque_y

# 绕 Z 轴的力矩
float64 torque_z

#== 夹爪外力 ==
float64 gripper_force

# === 可选：阻力系数 ===
# 线性阻力系数（单位：N/(m/s)）
float64 linear_damping

# 角阻力系数（单位：Nm/(rad/s)）
float64 angular_damping

# === 状态标志 ===
# 数据是否有效（true: 有效, false: 无效）
bool is_valid

# 是否检测到外力（true: 检测到, false: 未检测到）
bool external_force_detected
