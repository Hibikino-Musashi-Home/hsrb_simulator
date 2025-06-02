cmake_minimum_required(VERSION 3.5)
project(hsrb_gripper_fake_interface)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}/hsrb_gripper_fake_interface.cpp)
ament_target_dependencies(${PROJECT_NAME}
                          hardware_interface
                          rclcpp)

pluginlib_export_plugin_description_file(hardware_interface hsrb_gripper_fake_interface.xml)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies()

ament_package()
