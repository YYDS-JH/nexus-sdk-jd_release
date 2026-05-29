cmake_minimum_required(VERSION 3.8)
project(infra_msg)

# 默认使用 Release 编译模式（-O3 -DNDEBUG）
# 可通过命令行覆盖：colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointMitControl.msg"
  "msg/MotorStatus.msg"
  "msg/TeleOpArmStatus.msg"
  "msg/CartesianImpedance.msg"
  "msg/ArmControlState.msg"
  "msg/EndPoseData.msg"
  "msg/HumanDataInterface.msg"
  "msg/EndEffectorConfig.msg"
  "msg/EndPoseCmd.msg"
  "msg/RobotEECmd.msg"
  "msg/TeleopPedalState.msg"
  "msg/GripperKeyState.msg"
  "msg/TeleopGripperKeyState.msg"
  "msg/PeripheralCommand.msg"
  "msg/TeleopManagerState.msg"
  "msg/CollectorState.msg"
  "srv/ControllerConfig.srv"
  "srv/CollectorService.srv"
  "srv/InferenceConfig.srv"
  "srv/SetOperatorStatus.srv"
  DEPENDENCIES 
  std_msgs
  builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()



