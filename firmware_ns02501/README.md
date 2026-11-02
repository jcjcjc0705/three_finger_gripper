# firmware_ns02501

Everything that runs on the ns02501/NT98532 board. For what it does and how to use it, see
the [top-level README](../README.md). For wiring and board configuration, see
[HARDWARE.md](HARDWARE.md).

## Build

Needs Docker, nothing else. The compiler comes from the `Dockerfile` here.

```bash
./build.sh           # every program
./build.sh check     # offline tests, no hardware
./build.sh deploy    # stripped copies in deploy/
./build.sh clean
```

## Deploy

Copy `deploy/` to `/usr/bin/gripper/` on the board, with `uros_node` stopped. The programs
are static, so nothing else has to go with them. Over the Ethernet link, with a terminal
on the board's serial console at 115200:

```bash
cd deploy && python3 -m http.server 8000 --bind 192.168.0.1     # PC
```

```bash
mkdir -p /usr/bin/gripper && cd /usr/bin/gripper                # board
for f in uros_node dxl_offline tof_read camera_capture uros_hello; do
  wget -O $f.new http://192.168.0.1:8000/$f && chmod +x $f.new && mv $f.new $f
done
sync
```

`/tmp` on the board is mounted noexec and is lost at power off. `/usr/bin` is on NAND.

## Programs on the board

### uros_node

The gripper. `docker/scripts/agent.sh` starts and stops it, passing `UROS_ARGS` on to it.

```
./uros_node --agent 192.168.0.1
./uros_node --agent 192.168.0.1 --dry-run     torque never enabled
./uros_node --help                            every option
```

| Option | Default | |
|---|---|---|
| `--dry-run` | | Torque never enabled. Commands run with nothing moving, and contact can never be detected |
| `--touch-mm` | 50 | What the TOF reads where the fingers meet an uncalibrated object |
| `--band-mm` | 40 | Start closing this far before that |
| `--fastest-close-s` | 3 | A full closure takes at least this long |
| `--close-seconds` | 4 | Time to close fully on `grasp` |
| `--contacts` | 3 | Flexion joints in contact that count as holding |
| `--stall-current` | 0.5 | Fraction of the current ceiling that counts as contact |
| `--stall-release` | 0.3 | Fraction below which contact lets go |
| `--grasp` | `tripod` | Built in grasp chosen at start. Moves nothing until told |
| `--store` | `/usr/bin/gripper/gripper_store.txt` | What the hand has been taught |
| `--video-size` | 640x480 | MJPEG frame size. 1280x720 needs `--video-buffers 2` |
| `--no-camera` | | Run without the camera |

### Testing one subsystem

With `uros_node` stopped:

```bash
./dxl_offline scan                  # expect 8 motors, model 1240
./dxl_offline dump 1
./dxl_offline read 1 64 1           # address and size
./dxl_offline write 1 65 1 1        # address, size and value
./dxl_offline torque all off
./dxl_offline reboot all
./dxl_offline --help

./tof_read --short                  # the mode uros_node runs
./tof_read --short --seconds 0      # until Ctrl-C

./camera_capture --list
./camera_capture --frames 5 --output /tmp

./uros_hello --agent 192.168.0.1    # the link to the Agent, touching no hardware
```

## What the hand has been taught

`/usr/bin/gripper/gripper_store.txt`, plain text, written on every change. A board without
one starts with `kObjects` from `src/grasp_sequencer.hpp`.

```
gripper_store 1
grasp <name> <index> <middle> <6 open> <6 closed> <current> <seconds> <contacts>
blind <grasp> <closure>
object <name> <grasp> <touch_mm> <touch closure>
```

Angles are degrees, in the order thumb, index, middle, each proximal then distal. Closures
are 0 to 1. A distance or touch closure of 0 means not calibrated yet.

The same changes on `/gripper/command`, answered on `/gripper/status` with `store ok saved`
once on NAND, or `store err <why>`:

```
object_save <name> <grasp>      also moves the hand to its preshape
object_delete <name>
grasp_save <name> <index> <middle> <6 open> <6 closed> <current> <seconds> <contacts>
grasp_delete <name>
catalog                         lists every grasp and object, row by row
```

`store=failed` on the status line means a change did not reach NAND and will be lost at
power off.

To build a grasp into the firmware, add a row to `kGrasps` in `src/grasp_sequencer.hpp`.
`./build.sh check` then tests it: inside the joint limits, closes without self-collision,
and reachable from every other grasp.

## Source

| Path | |
|---|---|
| `src/uros_node.cpp` | The node: threads, topics, control loop |
| `src/grasp_sequencer.cpp` | Grasps, following the object in, contact |
| `src/grasp_store.cpp` | Objects and added grasps, kept on NAND |
| `src/motor_debug.cpp` | What `dynamixel_tools` asks of the board. The text protocol is in `motor_debug.hpp` |
| `src/gripper_joints.hpp` | The eight motors' IDs, directions, limits and gains |
| `src/servo/` | Servo loop and collision check. `gripper_model.hpp` is the gripper's kinematics and collision model, generated from the CAD |
| `src/reach_table.hpp` | How far each spread can close, generated from the same model |
| `src/dynamixel/` | The motor bus |
| `src/camera/`, `src/stream_server.cpp` | V4L2 capture, and the TCP stream to `palm_stream` |
| `src/tof/` | The VL53L1X over I2C |
| `src/dxl_offline.cpp`, `src/tof_read.cpp`, `src/camera_capture.cpp`, `src/uros_hello.cpp` | The test programs |
| `test/` | Offline tests, run by `./build.sh check` |

| Third party | |
|---|---|
| `third_party/dynamixel_sdk/` | [ROBOTIS DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK), Apache-2.0 |
| `third_party/vl53l1x_uld/` | ST's VL53L1X Ultra Lite Driver, BSD-3-Clause, see its [README](third_party/vl53l1x_uld/README.md) |
| `third_party/micro_ros/` | Prebuilt `libmicroros.a`, see its [README](third_party/micro_ros/README.md) |

`libmicroros.a` allows 6 publishers, all in use, and 4 subscriptions, of which 3 are in use.
More takes a rebuild of it.
