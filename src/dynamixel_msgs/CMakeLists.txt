cmake_minimum_required(VERSION 3.8)
project(dynamixel_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTorque.srv"
  "srv/Reboot.srv"
  "srv/SetZero.srv"
  "srv/ReadRegister.srv"
  "srv/WriteRegister.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
