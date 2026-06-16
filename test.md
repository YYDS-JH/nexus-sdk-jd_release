启动方式
从臂 Orin（新方案，默认 supervisor_mode=true）：

ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_real_system_cross_subnet.launch.py \
  role:=slave supervisor_mode:=true

主控 Nexus PC（不变）：

ros2 launch nexus_manage nexus_arm_v18_right_to_ar5_real_system_cross_subnet.launch.py role:=master
