cmake_minimum_required(VERSION 3.8)
project(dynamixel_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dynamixel_sdk
  dynamixel_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(dynamixel_bus SHARED src/dynamixel_bus.cpp)
add_library(dynamixel_hardware::dynamixel_bus ALIAS dynamixel_bus)
target_compile_features(dynamixel_bus PUBLIC cxx_std_17)
target_include_directories(dynamixel_bus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(dynamixel_bus PUBLIC dynamixel_sdk)

add_library(dynamixel_system SHARED src/dynamixel_system.cpp)
add_library(dynamixel_hardware::dynamixel_system ALIAS dynamixel_system)
target_compile_features(dynamixel_system PUBLIC cxx_std_17)
target_include_directories(dynamixel_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(dynamixel_system PUBLIC dynamixel_bus)
ament_target_dependencies(dynamixel_system PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface dynamixel_hardware.xml)

# Offline rescue tool: links dynamixel_bus only, no ROS runtime needed.
add_executable(dxl_offline src/dxl_offline.cpp)
target_link_libraries(dxl_offline PRIVATE dynamixel_bus)
install(TARGETS dxl_offline DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS dynamixel_bus dynamixel_system
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(dynamixel_bus dynamixel_system)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()