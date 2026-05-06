cmake_minimum_required(VERSION 3.8)
project(human_data)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(infra_msg REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Detect target architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(NEXUS_ARCH "aarch64")
else()
  set(NEXUS_ARCH "x86_64")
endif()

# Install prebuilt libraries for the target architecture
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/${NEXUS_ARCH}")
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/${NEXUS_ARCH}/
    DESTINATION lib
    USE_SOURCE_PERMISSIONS
    PATTERN "human_data" EXCLUDE
  )
endif()

# Install executables for the target architecture
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/${NEXUS_ARCH}/human_data")
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/${NEXUS_ARCH}/human_data/
    DESTINATION lib/human_data
    USE_SOURCE_PERMISSIONS
  )
endif()

# Install headers
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
  )
endif()

# Export for downstream packages
ament_export_include_directories(include)
ament_export_libraries(common_fsm kinematics_solver gripper_solver human_data_solver_node)
ament_export_dependencies(rclcpp std_msgs sensor_msgs infra_msg ament_index_cpp)

ament_package()
