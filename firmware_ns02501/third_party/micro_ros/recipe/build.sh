#!/bin/bash
# Runs inside microros/micro_ros_static_library_builder:jazzy.
# Not the image's own entrypoint: that expects the project to supply a
# library_generation.sh at a fixed path, which buys nothing here and hides
# what actually runs.
set -e

source /opt/ros/jazzy/setup.bash
source /uros_ws/install/local_setup.bash

cd /work
# Fetches ~30 repos into mcu_ws/ and marks the packages that cannot cross
# compile with COLCON_IGNORE.
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

# The meta file REPLACES the default rather than merging with it, so
# recipe/colcon.meta is derived from upstream's own client-colcon.meta.
ros2 run micro_ros_setup build_firmware.sh /recipe/toolchain.cmake /recipe/colcon.meta

cp /work/firmware/build/libmicroros.a /out/
rm -rf /out/include
cp -r /work/firmware/build/include /out/

# The container is root, so everything it wrote on a bind mount belongs to
# root on the host -- including the work tree, which the host user then cannot
# even delete to retry. Hand both back.
chown -R "${HOST_UID}:${HOST_GID}" /out/libmicroros.a /out/include
chown -R "${HOST_UID}:${HOST_GID}" /work

echo
echo "=== done ==="
ls -la /out/libmicroros.a
