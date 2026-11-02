# Cross-compile micro-ROS for the ns02501/NT98532 board.
#
# Follows upstream's own demo_toolchain.cmake closely, and deliberately does
# NOT set CMAKE_SYSROOT or any CMAKE_FIND_ROOT_PATH_MODE_*. A first attempt
# set FIND_ROOT_PATH_MODE_PACKAGE to ONLY, which looks right for a cross build
# and is wrong here: this build needs to find ament_cmake and the rosidl
# generators, and those are x86 host tools in /opt/ros/jazzy, not ARM packages
# in a sysroot. Restricting the search to the sysroot fails the very first
# package with "Add the installation prefix of ament_cmake to
# CMAKE_PREFIX_PATH". The compiler knows its own sysroot for target headers.
#
# No -march/-mfpu/-mfloat-abi either. This compiler already defaults to
# armv8-a+simd, hard float, neon-fp-armv8 and thumb, which is what every other
# binary in this project is built with; naming them again only creates a
# chance to disagree, and an ABI mismatch between libmicroros.a and the rest
# is a link error at best.
#
# The toolchain must be visible inside the build container at this same path.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Cross builds cannot run the test executable CMake normally compiles to probe
# the compiler; building a static library instead is upstream's own fix.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TC_PREFIX /opt/ivot/arm-ca32-linux-gnueabihf-10.4.0/usr/bin/arm-ca32-linux-gnueabihf-)

set(CMAKE_C_COMPILER   ${TC_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TC_PREFIX}g++)
set(CMAKE_AR           ${TC_PREFIX}ar     CACHE FILEPATH "archiver")
set(CMAKE_RANLIB       ${TC_PREFIX}ranlib CACHE FILEPATH "ranlib")

set(CMAKE_C_FLAGS_INIT   "-O2")
set(CMAKE_CXX_FLAGS_INIT "-O2")
