cmake_minimum_required(VERSION 3.8)
project(rpd_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rpd_robot_hardware_interface.cpp
  src/servo/SCS.cpp
  src/servo/SCSCL.cpp
  src/servo/SCSerial.cpp
  src/servo/SMSBL.cpp
  src/servo/SMSCL.cpp
  src/servo/SMS_STS.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/rpd_robot_hardware/servo>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface rpd_robot_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_package()