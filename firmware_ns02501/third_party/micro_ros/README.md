# micro-ROS

`libmicroros.a` is the micro-ROS client built for this board: `rcl`, `rclc`, `rmw_microxrcedds`
and the type support for the messages `uros_node` uses, for ROS 2 Jazzy over UDP.

Its limits are fixed at build time, in `recipe/colcon.meta`: 6 publishers, 4 subscriptions,
2 services, and only the message packages it was built with.

## Rebuilding it

Only needed to change those limits or the message packages. `recipe/` holds what decides
the result:

| File | |
|---|---|
| `toolchain.cmake` | The compiler: the ns02501 SDK's `arm-ca32-linux-gnueabihf` at `/opt/ivot` |
| `colcon.meta` | The limits. It replaces the default meta file rather than merging with it, so it has to keep everything else it sets |
| `build.sh` | What runs inside the container |

It runs in `microros/micro_ros_static_library_builder:jazzy`, with these mounted, and takes
a few minutes:

| Mount | |
|---|---|
| `/recipe` | `recipe/` |
| `/work` | An empty work directory |
| `/out` | This directory |
| `/opt/ivot` | The toolchain, at this same path |

and the environment variables `HOST_UID` and `HOST_GID`.
