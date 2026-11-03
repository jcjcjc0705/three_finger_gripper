# three_finger_gripper

A three finger gripper with 8 XC330-M288-T motors, a palm camera and a palm TOF sensor.
Everything on the gripper runs on its ns02501/NT98532 board under micro-ROS. A PC
connected over Ethernet starts it, displays it and sends it commands.

- [System architecture](#system-architecture)
- [Setup](#setup)
- [Starting and stopping](#starting-and-stopping)
- Functions: [Grasping](#grasping), [Teaching objects and grasps](#teaching-objects-and-grasps),
  [Viewing in Foxglove](#viewing-in-foxglove), [Motor debugging](#motor-debugging),
  [Safety behaviour](#safety-behaviour), [3D model](#3d-model)
- [ROS 2 interface](#ros-2-interface)
- [Known limitations](#known-limitations)

---

## System architecture

```
 board (ns02501/NT98532, 192.168.0.3)            PC (192.168.0.1, Docker, ROS 2 Jazzy)

 uros_node                                       gripper_node    starts and stops uros_node
 ├ grasp sequencer, stored objects and grasps ◄─ serial console ─┘
 ├ servo loop, self-collision check
 ├ motor debugging                               gripper_agent   micro-ROS Agent
 │                                                    │ ROS 2 topics
 ├ motors    /dev/ttyS1  ┐                            ├─ gripper_state    /joint_states, /tf,
 ├ TOF       /dev/i2c-2  ├──── UDP 8888 ──────────────┘                   board_bridge
 │                       ┘                            ├─ grasp_tui        grasp control
 └ camera    /dev/video0 ───── TCP 9000 ─► gripper_camera  /palm_camera   dxl_cli, dxl_debug
                                                      └─ gripper_foxglove  :8765 ─► Foxglove
```

| Runs on | Part | What it does |
|---|---|---|
| Board | `uros_node` | Control loop, grasps, self-collision protection, motor debugging, what the hand has been taught |
| Board | `dxl_offline`, `tof_read`, `camera_capture`, `uros_hello` | Test one subsystem on its own, with `uros_node` stopped |
| PC | `gripper_agent` | The micro-ROS Agent, which connects `uros_node` to ROS 2 |
| PC | `gripper_node` | Starts and stops `uros_node` over the board's serial console |
| PC | `gripper_state` | `robot_state_publisher` and `board_bridge`: `/joint_states`, `/tf`, motor services |
| PC | `gripper_camera` | `palm_stream`: camera frames from the board to ROS 2 |
| PC | `gripper_foxglove` | The Foxglove bridge on port 8765 |
| PC | `grasp_tui` | Terminal interface to watch and drive the grasps |
| PC | `dxl_cli`, `dxl_debug` | Motor registers, torque, reboot, jogging |

Rates: control loop and joint states 25 Hz, TOF 30 Hz, camera 640x480 MJPEG at 30 fps,
about 5 Mbit/s.

### Repository

```
docker/              PC image, one compose file and one script per service
firmware_ns02501/    Everything that runs on the board, see its README
  HARDWARE.md        Wiring and board configuration
src/                 ROS 2 packages on the PC
  dynamixel_msgs       motor service definitions
  dynamixel_tools      dxl_cli, dxl_debug, board_bridge
  gripper_bringup      launch files, Foxglove layout
  gripper_grasp        grasp_tui
  gripper_sensors      palm_stream
  three_finger_gripper_description   URDF and meshes
cad/                 The gripper's CAD
```

---

## Setup

**Hardware:** wire the board as in [firmware_ns02501/HARDWARE.md](firmware_ns02501/HARDWARE.md).

**Board:** build and copy the firmware as in
[firmware_ns02501/README.md](firmware_ns02501/README.md). It lives in `/usr/bin/gripper/`
and the board's address is 192.168.0.3.

**PC:** Linux with Docker and Docker Compose. Then:

1. Set the Ethernet port connected to the board to the static address `192.168.0.1/24`.
2. Plug in the USB serial adapter on the board's console. To start with another adapter
   than the Prolific one in `docker/compose/docker-compose-agent.yaml`:
   `export BOARD_CONSOLE=/dev/serial/by-id/<your adapter>`.
3. Build the image and the ROS 2 workspace, once:

   ```bash
   docker/scripts/build.sh
   ```

   The image runs as UID 1000, which has to be able to write to this directory.

---

## Starting and stopping

```bash
docker/scripts/start-all.sh            # everything, torque on
docker/scripts/start-all.sh --dry-run  # everything, torque stays off and nothing moves
```

Torque coming on does not move the fingers. `start-all.sh` leaves you in a shell in one
of the containers. Open others with:

```bash
docker/scripts/start-all.sh exec
```

**Leaving the first shell stops everything.** `uros_node` turns the torque off, so
whatever is in the hand falls. Every motor is then read back to confirm.

Each service also has its own script, working on the same containers, so one can be
restarted while the others keep running:

| Script | Starts |
|---|---|
| `docker/scripts/agent.sh [--dry-run]` | Agent and `uros_node` |
| `docker/scripts/gripper.sh` | `/joint_states`, `/tf`, `board_bridge` |
| `docker/scripts/camera.sh` | Camera from the board |
| `docker/scripts/foxglovebridge.sh` | Foxglove bridge |

Any other argument goes to `docker compose`:

| Command | |
|---|---|
| `docker/scripts/agent.sh restart agent` | Restart the link only. The hand is left alone |
| `docker/scripts/agent.sh restart node` | Restart `uros_node`. **Drops the torque** |
| `docker/scripts/agent.sh logs node` | Starting and stopping `uros_node`. Its own log is `/tmp/uros.log` on the board |
| `docker/scripts/camera.sh logs -f` | A service's log |
| `docker/scripts/gripper.sh debug` | An idle container of that service, to build or run tools in |

`UROS_ARGS` passes options to `uros_node`, such as
`UROS_ARGS="--band-mm 60" docker/scripts/start-all.sh`. `--dry-run` is added to them. See
the [firmware README](firmware_ns02501/README.md#uros_node) for the options.

---

## Grasping

```bash
docker/scripts/start-all.sh exec
ros2 run gripper_grasp grasp_tui
```

⚠ With torque on, `Enter`, `g`, `a`, `b` and `r` move the fingers.

| Key | |
|---|---|
| Arrows, `Enter` | Pick an object or a grasp and move to its preshape |
| `a` | Start or stop following the object in, see below |
| `g` | Close now |
| `r` | Release: open to the preshape. Anything held falls |
| `s` | Stop and hold where it is |
| `c` | Save the current range as where the fingers meet this object |
| `b` | Measure where this grasp's fingers block the TOF. The empty hand closes |
| `t` | Torque on or off. While holding something it asks first |
| `n`, `e`, `d` | New, edit, delete the object or grasp under the cursor |
| `q` | Quit. The hand carries on with what it was doing |

The screen shows the phase, the grasp and object in hand, the closure, the TOF range and,
for each joint, the goal, the position, the difference, the current and its ceiling,
contact, torque, temperature, voltage and hardware error. Warnings are shown in reverse
video: no data from the board, a self-collision pose, a motor hardware error, a failed
save.

| Phase | |
|---|---|
| `idle` | No goal: holding where it is |
| `preshape` | Open, in this grasp's preshape |
| `armed` | Waiting for the object to come in range |
| `following` | Closing as the object comes in |
| `closing` | Closing until the contacts stop it |
| `holding` | Keeping its grip |
| `measuring` | Measuring where the fingers block the TOF |

**Following the object in (`a`).** Once the object is within 40 mm of its calibrated
distance, the nearer it comes, the further the hand closes, reaching the object as the
object reaches the calibrated distance. If it stops, the hand stops. If it backs off or
disappears, the hand opens. Once the fingers block the TOF the hand carries on at the rate
it was closing until enough joints report contact. A full closure takes at least 3 s, so
an object coming faster than about 12 mm/s arrives before the fingers do.

**Closing now (`g`)** closes over 4 s, whatever the TOF reads.

### Grasps

| Grasp | Spread (index / middle) | Opening | For |
|---|---|---|---|
| `tripod` | +52° / -52° | about 80 mm | Round objects, three tips evenly spaced |
| `pinch` | +90° / -90° | about 70 mm | Index and middle together against the thumb |
| `wide` | 0° / 0° | about 80 mm to the thumb | Fingers apart |
| `envelop` | +52° / -52° | about 190 mm | Large soft objects: proximal joints lean out, then wrap |
| `pinch_wide` | +90° / -90° | about 190 mm | A pinch for something thicker |
| `two_finger` | 0° / 0° | about 130 mm | Small objects: index against middle, the thumb leans out |

### Objects

A new board knows `cup` (`pinch`), `foam` (`tripod`) and `bear` (`envelop`), uncalibrated.
Up to 32 objects and 16 added grasps are kept on the board, and survive a power cut.

`/gripper/object` names what is in front of the hand. Object recognition is not included:
whatever recognises the object only has to publish its name there.

---

## Teaching objects and grasps

The object list shows each object's grasp and calibration: `cup -> pinch  85mm 84%` closes
with `pinch`, meets the object at a TOF reading of 85 mm and touches it at 84% closure.
The grasp list shows `blind`, the closure from which the fingers block the TOF. `grasp_tui`
says what is missing for the object and grasp in hand.

**Once per grasp,** with the hand empty and nothing within 200 mm of the TOF, select it
and press `b`, `y`.

**A new object:**

1. Press `n` on the object list and name it.
2. Pick a grasp, or `+ new grasp` to add one. The hand moves to its preshape.
3. Hold the object where the fingers should meet it and press `c`.
4. Keep it there and press `g`. The hand closes and saves how far it had closed when it
   touched. Press `r` to open.

From then on, pick the object, press `a` and bring it in.

**A new grasp,** from `+ new grasp` or `n` on the grasp list, after naming it:

1. Press `t` for torque off. The fingers go limp.
2. Bend them to the open pose and press `o`.
3. Bend them to the closed pose and press `k`. The spread stays as it was at `o`.
4. Press `t` for torque on, which moves nothing, and `Enter` to save.

Arrows and digits edit a number in the form. `current`, `seconds` and `contacts` set this
grasp's Goal Current, closing time and joints in contact, with 0 keeping the default.
A grasp outside the joint limits, closing into self-collision or unable to open again is
refused, with the reason.

---

## Viewing in Foxglove

Connect [Foxglove](https://foxglove.dev) to `ws://localhost:8765` and import
`src/gripper_bringup/foxglove/board_layout.json` for the 3D model, camera, TOF, status and
motor temperature, voltage and error.

Without that layout, set the 3D panel's **Scene > Mesh up-axis** to `Z-up`, and use
`/palm_camera/image_raw/compressed` for the camera. `/palm_camera/image_raw` is the same
camera decoded, at about 215 Mbit/s.

---

## Motor debugging

With everything running, from `docker/scripts/start-all.sh exec`:

```bash
ros2 run dynamixel_tools dxl_cli dump 1              # every register of motor 1
ros2 run dynamixel_tools dxl_cli read 1 146 1        # address and size
ros2 run dynamixel_tools dxl_cli write 1 led 1       # address or name, then value
ros2 run dynamixel_tools dxl_cli torque all off      # a motor, or all
ros2 run dynamixel_tools dxl_cli reboot 3            # clears a hardware error
ros2 run dynamixel_tools dxl_cli zero 3              # the present position becomes zero
ros2 run dynamixel_tools dxl_debug                   # table of all motors, with jogging
ros2 run dynamixel_tools dxl_debug --check           # print the table once
```

`dxl_debug` keys: up/down select a joint, left/right jog it, `[` `]` change the step, `h`
back to the start pose, `t` torque, `q` quit.

- A reboot leaves that motor's torque off, and its RAM registers restored.
- Enabling torque on a motor does not move it.
- Grasp commands are ignored while any motor has no torque.
- Jogging goes through the same self-collision and joint limit checks as everything else,
  and is ignored while closing or holding.
- Refused: writing registers 100-119, the ID and the baud rate. EEPROM writes need that
  motor's torque off.

With `uros_node` stopped, `dxl_offline` on the board does the same over the bus directly.
See the [firmware README](firmware_ns02501/README.md#programs-on-the-board).

### Motors

Zero is fully open, and negative closes. The bus runs at 1 Mbaud.

| ID | Joint | Function | Limits |
|---|---|---|---|
| 1 | `C_2_Joint` | Thumb proximal | -12.7° to 72.2° |
| 2 | `C_3_Joint` | Thumb distal | -120° to 36.9° |
| 3 | `A_2_Joint` | Index proximal | -28.6° to 70.9° |
| 4 | `A_3_Joint` | Index distal | -120° to 36.9° |
| 5 | `B_2_Joint` | Middle proximal | -28.6° to 70.9° |
| 6 | `B_3_Joint` | Middle distal | -120° to 36.9° |
| 7 | `A_1_Joint` | Index spread | ±90° |
| 8 | `B_1_Joint` | Middle spread | ±90° |

---

## Safety behaviour

| Situation | What happens |
|---|---|
| A goal would make the fingers collide | The fingers stop short, `servo=blocked` |
| Fingers pushed into a pose they cannot move out of | After 0.3 s every motor loses torque, `fault=self_collision`. Torque is refused until the fingers are moved apart by hand, then `t` turns it on. It does not come back by itself |
| Starting in such a pose | Torque stays off until then |
| A motor shuts itself down on overload, overheating or electrical shock | `hw_err` counts it and grasp commands are ignored. `dxl_cli reboot <id>`, then torque on |
| The PC or the link goes away | The hand keeps its grip and finishes what it was doing, and reconnects by itself |
| A finger pushing at its current ceiling | It stops advancing, and keeps its grip |
| Stopping through the scripts | Torque off, then every motor is read back and turned off if still on |

Each motor also has its position limits in EEPROM, and Goal Current limits how hard it can
push: 350 on flexion joints and 250 on spread joints, out of 800.

---

## 3D model

The model in RViz with sliders for the joints, no hardware needed:

```bash
xhost +local:
docker/scripts/gripper.sh debug
ros2 launch three_finger_gripper_description view_robot.launch.py
```

---

## ROS 2 interface

| Topic | Type | Rate | |
|---|---|---|---|
| `/gripper/command` | `std_msgs/String` | in | Commands, below |
| `/gripper/object` | `std_msgs/String` | in | Name of the object in front of the hand |
| `/gripper/status` | `std_msgs/String` | 5 Hz | `phase= object= grasp= closure= servo= contacts= ... fault= note=` |
| `/gripper/goal` | `std_msgs/Float32MultiArray` | 10 Hz | 8 positions (rad), 8 current ceilings, then 1 if a goal is active |
| `/gripper/joint_states` | `sensor_msgs/JointState` | 25 Hz | From the board. `effort` is Present Current in register units |
| `/gripper/motor_diag` | `std_msgs/Float32MultiArray` | 2 Hz | Per joint: temperature, voltage, hardware error, torque |
| `/gripper/motor_cmd` | `std_msgs/String` | in | Motor requests from `board_bridge`, as text. The format is in [motor_debug.hpp](firmware_ns02501/src/motor_debug.hpp) |
| `/gripper/motor_reply` | `std_msgs/String` | on request | The replies |
| `/tof/range` | `sensor_msgs/Range` | 30 Hz | `+inf` when the sensor could not measure |
| `/joint_states`, `/tf` | | 25 Hz | From `gripper_state` |
| `/dynamic_joint_states` | `control_msgs/DynamicJointState` | 2 Hz | `/gripper/motor_diag` with names |
| `/palm_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 30 Hz | JPEG |
| `/palm_camera/image_raw` | `sensor_msgs/Image` | 30 Hz | Decoded |

Joint order on the array topics is motor 1 to 8.

| Command | |
|---|---|
| a grasp name | Choose it and move to its preshape |
| `arm`, `disarm` | Start or stop following the object in |
| `grasp` | Close now |
| `release`, `open` | Back to the preshape |
| `stop` | Hold where it is, issuing no goals |
| `calibrate` | The current range is where the fingers meet the object in hand, saved. The next `grasp` saves how far it had closed by then |
| `measure` | Close an empty hand slowly and save where the fingers block the TOF |
| `photo` | Save a JPEG on the board and report its path on `/gripper/status` |
| `catalog`, `object_save`, `object_delete`, `grasp_save`, `grasp_delete` | What `grasp_tui` uses to list and teach, see the [firmware README](firmware_ns02501/README.md#what-the-hand-has-been-taught) |

```bash
ros2 topic pub --once /gripper/object std_msgs/msg/String "{data: cup}"
ros2 topic pub --once /gripper/command std_msgs/msg/String "{data: arm}"
```

`board_bridge` offers `~/set_torque`, `~/reboot`, `~/set_zero`, `~/read_register` and
`~/write_register` under `/gripper_board`, with the types in `dynamixel_msgs`.

---

## Known limitations

- **`uros_node` is started over the serial console.** The adapter has to be plugged in,
  and no other terminal may hold the port while it starts or stops. It does not start by
  itself when the board boots.
- **If `gripper_node` dies without stopping cleanly,** through a PC crash or `docker kill`,
  `uros_node` keeps running with torque on until the next start stops it.
- **The last part of a followed grasp is a prediction,** because the fingers block the TOF.
  An object changing speed there is gripped further from its calibrated distance.
- **An added grasp is not checked against every other grasp.** If the hand cannot reach its
  preshape from where it is, it stops with `servo=blocked`: go through another grasp.
- **The longest hold tested at the current ceiling is 20 s.** Longer holds have not been
  tested for heat.
- The palm mesh shows the sensor board mirrored, and `palm_tof`'s pose in the URDF is
  approximate. Neither affects control.
