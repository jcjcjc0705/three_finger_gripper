cmake_minimum_required(VERSION 3.8)
project(gripper_servo)

# Deliberately no ROS dependencies. The same sources build for the firmware.
find_package(ament_cmake REQUIRED)

add_library(gripper_servo STATIC
  src/collision.cpp
  src/servo.cpp
)
target_compile_features(gripper_servo PUBLIC cxx_std_17)
# The controller is a shared object and links this in. Harmless for the
# firmware build, which compiles the sources on its own terms anyway.
set_target_properties(gripper_servo PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(gripper_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# The firmware build has neither, and relying on them here would let them creep in.
target_compile_options(gripper_servo PRIVATE -fno-exceptions -fno-rtti)

add_executable(gripper_servo_check test/check.cpp)
target_link_libraries(gripper_servo_check gripper_servo)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gripper_servo EXPORT export_gripper_servo
        ARCHIVE DESTINATION lib)
install(TARGETS gripper_servo_check DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_gripper_servo HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
