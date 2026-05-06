cmake_minimum_required(VERSION 3.8)
project(nexus_manage)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(infra_msg REQUIRED)
find_package(geometry_msgs REQUIRED)

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
    PATTERN "nexus_manage" EXCLUDE
  )
endif()

# Install executables for the target architecture
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/${NEXUS_ARCH}/nexus_manage")
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/${NEXUS_ARCH}/nexus_manage/
    DESTINATION lib/nexus_manage
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

# Export for downstream packages
ament_export_include_directories(include)
ament_export_libraries(state_machine_engine config_parser_component key_detector_component controller_config_component inference_config_component rotation_orthogonalizer_component rotation_mapping_component command_calculator_component teleop_manager_node)
ament_export_dependencies(rclcpp std_msgs infra_msg geometry_msgs)

ament_package()
