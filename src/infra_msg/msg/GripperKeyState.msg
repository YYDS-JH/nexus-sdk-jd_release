# GripperKeyState.msg

string gripper_name

bool teleop_key            # 遥操按键 - 遥操模式切换
bool data_collect_key      # 数采按键 - 持续3秒触发遥操运行（需配合安全脚踏板）
bool marker_key            # 打标按键 - 数据标记
bool safety_key            # 安全按键 - 硬件安全保护，松开时进入位置保持
bool take_over_key         # 接管按键
bool sw_key                # SW按键
bool rocker_key            # 摇杆按键

int16 rocker_x             # 摇杆X轴值，范围 -100 ~ 100
int16 rocker_y             # 摇杆Y轴值，范围 -100 ~ 100