cmake_minimum_required(VERSION 3.8)
project(gripper_control)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(gripper_msgs REQUIRED)
find_package(gripper_servo REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(gripper_controller SHARED src/gripper_controller.cpp)
target_compile_features(gripper_controller PUBLIC cxx_std_17)
target_include_directories(gripper_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gripper_controller
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
target_link_libraries(gripper_controller
  gripper_servo::gripper_servo
  ${gripper_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface gripper_control.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gripper_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(gripper_controller)
ament_package()
