cmake_minimum_required(VERSION 3.8)
project(infra_msg)

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
  DEPENDENCIES 
  std_msgs
  builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()



