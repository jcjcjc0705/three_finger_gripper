# Everything that runs on the board. Use ./build.sh, which has the compiler:
#   ./build.sh           build
#   ./build.sh check     run the offline tests
#   ./build.sh deploy    stripped copies in deploy/, ready to copy to the board

CROSS_COMPILE ?= arm-linux-gnueabihf-
# Static, so that the board's own libraries do not matter.
CXX := $(CROSS_COMPILE)g++ -static
STRIP := $(CROSS_COMPILE)strip
HOSTCXX ?= g++

BUILD_DIR := build
DEPLOY_DIR := deploy

DXL_SDK_DIR := third_party/dynamixel_sdk
ULD_DIR := third_party/vl53l1x_uld
UROS_DIR := third_party/micro_ros

CXXFLAGS := -std=c++17 -O2 -Wall -Wextra \
	-I src \
	-I $(DXL_SDK_DIR)/include \
	-I $(DXL_SDK_DIR)/include/dynamixel_sdk \
	-I $(ULD_DIR)

SDK_SRCS := $(DXL_SDK_DIR)/src/port_handler.cpp \
	$(DXL_SDK_DIR)/src/port_handler_linux.cpp \
	$(DXL_SDK_DIR)/src/packet_handler.cpp \
	$(DXL_SDK_DIR)/src/protocol1_packet_handler.cpp \
	$(DXL_SDK_DIR)/src/protocol2_packet_handler.cpp \
	$(DXL_SDK_DIR)/src/group_handler.cpp \
	$(DXL_SDK_DIR)/src/group_sync_read.cpp \
	$(DXL_SDK_DIR)/src/group_sync_write.cpp
ULD_SRCS := $(ULD_DIR)/core/VL53L1X_api.cpp \
	$(ULD_DIR)/core/VL53L1X_calibration.cpp \
	$(ULD_DIR)/platform/vl53l1_platform_linux.cpp

BUS_SRCS := src/dynamixel/dynamixel_bus.cpp
SERVO_SRCS := src/servo/servo.cpp src/servo/collision.cpp
CAMERA_SRCS := src/camera/v4l2_capture.cpp
TOF_SRCS := src/tof/tof_i2c.cpp
SEQUENCER_SRCS := src/grasp_sequencer.cpp src/grasp_store.cpp
MOTOR_DEBUG_SRCS := src/motor_debug.cpp
STREAM_SRCS := src/stream_server.cpp

to_obj = $(addprefix $(BUILD_DIR)/,$(1:.cpp=.o))

SDK_OBJS := $(call to_obj,$(SDK_SRCS))
ULD_OBJS := $(call to_obj,$(ULD_SRCS))
BUS_OBJS := $(call to_obj,$(BUS_SRCS))
SERVO_OBJS := $(call to_obj,$(SERVO_SRCS))
CAMERA_OBJS := $(call to_obj,$(CAMERA_SRCS))
TOF_OBJS := $(call to_obj,$(TOF_SRCS))
SEQUENCER_OBJS := $(call to_obj,$(SEQUENCER_SRCS))
MOTOR_DEBUG_OBJS := $(call to_obj,$(MOTOR_DEBUG_SRCS))
STREAM_OBJS := $(call to_obj,$(STREAM_SRCS))

TARGETS := dxl_offline camera_capture tof_read uros_hello uros_node

# Rebuilds what a changed header reaches. gripper_joints.hpp in particular is
# written to the motors' EEPROM at startup, so a stale object writes stale values.
CXXFLAGS += -MMD -MP
-include $(shell find $(BUILD_DIR) -name '*.d' 2>/dev/null)

.DEFAULT_GOAL := all
.PHONY: all clean deploy check

all: $(TARGETS)

deploy: $(TARGETS)
	@mkdir -p $(DEPLOY_DIR)
	@cp $(TARGETS) $(DEPLOY_DIR)/
	@$(STRIP) $(addprefix $(DEPLOY_DIR)/,$(TARGETS))
	@ls -la $(DEPLOY_DIR)

dxl_offline: $(SDK_OBJS) $(BUS_OBJS) $(call to_obj,src/dxl_offline.cpp)
	$(CXX) $^ -o $@ -pthread

camera_capture: $(CAMERA_OBJS) $(call to_obj,src/camera_capture.cpp)
	$(CXX) $^ -o $@

tof_read: $(TOF_OBJS) $(ULD_OBJS) $(call to_obj,src/tof_read.cpp)
	$(CXX) $^ -o $@

# -lmicroros comes last: it is an archive, so the linker only pulls the objects
# something ahead of it actually referenced.
uros_hello: $(call to_obj,src/uros_hello.cpp)
	$(CXX) $^ -o $@ -L $(UROS_DIR) -lmicroros -pthread

uros_node: $(SDK_OBJS) $(BUS_OBJS) $(SERVO_OBJS) $(CAMERA_OBJS) $(TOF_OBJS) \
		$(ULD_OBJS) $(SEQUENCER_OBJS) $(MOTOR_DEBUG_OBJS) $(STREAM_OBJS) \
		$(call to_obj,src/uros_node.cpp)
	$(CXX) $^ -o $@ -L $(UROS_DIR) -lmicroros -pthread

$(BUILD_DIR)/src/uros_%.o: src/uros_%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -I $(UROS_DIR)/include -c $< -o $@

# Third party code, whose warnings are not ours to act on.
$(BUILD_DIR)/$(DXL_SDK_DIR)/%.o: $(DXL_SDK_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -w -c $< -o $@

$(BUILD_DIR)/$(ULD_DIR)/%.o: $(ULD_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -w -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Built for the host, not the board: this is logic with no hardware behind it.
check:
	@mkdir -p $(BUILD_DIR)
	$(HOSTCXX) -std=c++17 -O1 -Wall -Wextra -I src test/servo_check.cpp $(SERVO_SRCS) \
		-o $(BUILD_DIR)/servo_check
	$(BUILD_DIR)/servo_check
	$(HOSTCXX) -std=c++17 -O1 -Wall -Wextra -I src test/grasp_check.cpp $(SEQUENCER_SRCS) \
		$(SERVO_SRCS) -o $(BUILD_DIR)/grasp_check
	$(BUILD_DIR)/grasp_check
	$(HOSTCXX) -std=c++17 -O1 -Wall -Wextra -I src test/motor_debug_check.cpp \
		$(MOTOR_DEBUG_SRCS) -o $(BUILD_DIR)/motor_debug_check
	$(BUILD_DIR)/motor_debug_check

clean:
	rm -rf $(BUILD_DIR) $(DEPLOY_DIR)
	rm -f $(TARGETS)
