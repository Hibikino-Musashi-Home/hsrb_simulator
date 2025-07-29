cmake_minimum_required(VERSION 3.5)
project(hsrb_gz_ros2_control)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tmc_exxx_servo_motor_protocol REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

include_directories(include)

add_library(hsrb_hardware_plugins SHARED
  src/hsrb_system.cpp
)
ament_target_dependencies(hsrb_hardware_plugins
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  rclcpp
  gz_ros2_control
  tmc_exxx_servo_motor_protocol
)
target_link_libraries(hsrb_hardware_plugins
  ignition-gazebo6::core
  ignition-plugin1::register
)

install(DIRECTORY
  include/
  DESTINATION include
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(hsrb_hardware_plugins)

pluginlib_export_plugin_description_file(gz_ros2_control gz_hardware_plugins.xml)

## Install
install(
  TARGETS hsrb_hardware_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Setup the project
ament_package()
