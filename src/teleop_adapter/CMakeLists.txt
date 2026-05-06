cmake_minimum_required(VERSION 3.8)
project(teleop_adapter)

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
    PATTERN "teleop_adapter" EXCLUDE
  )
endif()

# Install executables for the target architecture
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/${NEXUS_ARCH}/teleop_adapter")
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/${NEXUS_ARCH}/teleop_adapter/
    DESTINATION lib/teleop_adapter
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
ament_export_libraries(serial_port device_adapter teleop_arm_adapter y1_arm_adapter ar5_arm_adapter teleop_arm_driver_node)
ament_export_dependencies(rclcpp std_msgs sensor_msgs infra_msg ament_index_cpp)

ament_package()
