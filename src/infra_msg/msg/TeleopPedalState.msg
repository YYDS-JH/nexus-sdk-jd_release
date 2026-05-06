# TeleopPedalState.msg
# 遥操脚踏板状态消息
# 包含所有物理脚踏板的状态信息

# 消息头
std_msgs/Header header

# === 脚踏板状态 ===
# true: 按下, false: 松开
bool reset_pedal           # REST脚踏板 - 触发复位流程
bool suspend_pedal         # suspend脚踏板 - 暂停/恢复遥操运行
bool increment_pedal       # 增量脚踏板 - 切换计算模式（Scaling ↔ 增量）
bool scaling_plus_pedal    # +scaling脚踏板 - 在Scaling模式下增大缩放系数
bool scaling_minus_pedal   # -scaling脚踏板 - 在Scaling模式下减小缩放系数
