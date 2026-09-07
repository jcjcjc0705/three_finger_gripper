# three_finger_gripper

ros2_control driver for a three finger gripper: 8 XC330-M288-T motors, 8
independently actuated joints, palm camera and TOF.

ROS 2 Jazzy, everything in Docker.

---

## Where the control loop lives

Today the PC runs the servo loop over an OpenRB-150. Later it moves onto a
Novatek NT98532 running micro-ROS, and the PC keeps only perception and the
grasp decision.

```
Phase 1 (now)                      Phase 2 (NT98532)
ros2_control controller  ←swap→    micro-ROS thread
        ↓                                  ↓
   gripper_servo         ←same→      gripper_servo
        ↓                                  ↓
   DynamixelBus          ←same→      DynamixelBus
        ↓                                  ↓
 PortHandler (Linux)     ←swap→   port_handler_novatek
```

**No grasp strategy lives below the decision layer.** The servo layer only
knows how to reach a joint pose smoothly and stop pushing when the current
saturates. Swapping a hand-written policy for a learned one never touches
firmware.

---

## Quick start

One script per thing that can run, and one that runs them all. Each starts the
service and drops you into a shell in it; Ctrl-C tears it down again.

```bash
docker/scripts/gripper.sh          # ros2_control and the eight motors
docker/scripts/camera.sh           # palm camera
docker/scripts/tof.sh              # palm TOF
docker/scripts/foxglovebridge.sh   # bridge on :8765
docker/scripts/start-all.sh        # all four at once
```

Anything else is passed through to `docker compose`:

```bash
docker/scripts/gripper.sh down
docker/scripts/camera.sh logs -f
```

To build or run tools without the service's own process in the way, use
`debug`: it stops the service and gives you an idle container with the same
mounts and devices.

```bash
docker/scripts/gripper.sh debug

cd /ws
colcon build --symlink-install
source install/setup.bash
ros2 launch three_finger_gripper_description view_robot.launch.py
```

The source tree is bind mounted at `/ws`, so edits on the host apply
immediately, and interactive shells source ROS and the workspace for you.

`start-all` and the single-service scripts cannot run at the same time: both
want the same container names.

---

## Hardware

| Device | Container path | Source |
|---|---|---|
| USB to DYNAMIXEL (FTDI, 8 motors) | `/dev/gripper` | `usb-FTDI_USB__-__Serial_Converter_FTBEQOTJ…` |
| TOF (Seeed XIAO M0) | `/dev/tof` | `usb-Seeed_Seeed_XIAO_M0_69B31688…` |
| Palm camera (Innodisk) | `/dev/palm_cam` | `usb-Innodesk_Innodisk_USB_Camera_0x0001-video-index0` |

Mapped by serial number in `docker/compose/`, so which USB port they land in
does not matter. `docker-compose-start-all.yaml` includes the single-service
files rather than repeating them, so each device is declared once.

The motors need their own supply. Running them off USB bus power sags the rail
under a grip and trips the motors' input-voltage shutdown: 4.5 V at idle was
already too little, 4.9 V is fine.

### Motors

Names are the CAD's, so the URDF, the USD and the mechanical drawings agree.

| Motor | Function | Joint | Limits |
|---|---|---|---|
| 1 | thumb proximal flexion | `C_2_Joint` | -12.7° ~ 72.2° |
| 2 | thumb distal flexion | `C_3_Joint` | -120° ~ 36.9° |
| 3 | index proximal flexion | `A_2_Joint` | -28.6° ~ 70.9° |
| 4 | index distal flexion | `A_3_Joint` | -120° ~ 36.9° |
| 5 | middle proximal flexion | `B_2_Joint` | -28.6° ~ 70.9° |
| 6 | middle distal flexion | `B_3_Joint` | -120° ~ 36.9° |
| 7 | index spread | `A_1_Joint` | ±90° |
| 8 | middle spread | `B_1_Joint` | ±90° |

The thumb base is fixed, so the thumb has no spread joint.

### Sign convention

**Zero is fully open. Negative closes.** All six flexion joints curl toward the
palm on negative angles; the positive range hyperextends the finger further out.
That is why the ranges look lopsided — the proximal joints only need 13–29° of
negative travel because the fingers are ~100 mm long, so a small rotation at the
base sweeps the tip a long way.

| Pose | Index tip radius from the palm axis |
|---|---|
| `A_2_Joint` 0° (open) | 62 mm |
| `A_2_Joint` -28.6° (closed) | 16 mm |
| `A_3_Joint` -120° (curled) | 27 mm, and 80 mm lower |

Fingertip triangle, spread at 0°: **124 / 78 / 67 mm** fully open, **44 / 30 /
21 mm** fully closed. The joint limits alone keep the fingertips 21 mm apart at
worst, so they cannot pinch each other (link bodies are not checked yet).

Two spread configurations worth knowing:

| | `A_1_Joint` | `B_1_Joint` | Result |
|---|---|---|---|
| Tripod | 52° | -52° | tips 111° / 117° / 132° apart, confirmed on hardware |
| Pinch | 80° | -80° | index and middle side by side, thumb opposite |

Tripod is the evenest symmetric spread; exactly 120° apart is not reachable
while keeping the fingers mirrored, because the thumb's base sits closer in. At
52° the index and middle tips are 54 mm from the palm axis and the thumb only
33 mm — that 21 mm is fixed by where the bases are, and a grasp has to take it
up with the flexion joints, not the spread.

**Nothing outside the decision layer should know these numbers.**

All eight run `current_based_position`: position control with a torque ceiling.
On the flexion joints that makes contact safe; on the spread joints it protects
the mechanism when two fingers meet.

Model number 1240 = XC330-M288-T. The bus runs at **1 Mbaud**; the motors ship
at 57600, which caps the loop at about 16 Hz once you account for a 25 byte
SyncRead across eight motors plus the SyncWrite. At 1 Mbaud the same cycle is
3.5 ms.

    ros2 run dynamixel_hardware dxl_offline --port /dev/gripper scan

### Self-collision

Fourteen link pairs can touch inside the joint limits — run
`tools/selfcollision.py` for the list. The dominant ones are index/thumb and
middle/thumb distal links, then fingers curling into the palm.

Do not build a lookup table off a one-dimensional "closure" path. Along a naive
curl-everything-proportionally trajectory the index and thumb fingertips
interpenetrate by up to 8.6 mm around 55% closure and separate again by 80% —
**collision is not monotonic in closure**, so a table built by walking that path
describes the path, not the mechanism. The servo layer has to check the pairs at
run time; the plan is a few spheres per link, which is a couple of hundred
distance tests per cycle and fits on the MCU.

---

## Sensors

| | Topic | Notes |
|---|---|---|
| Palm camera | `/palm_camera/image_raw` | `v4l2_camera`, no driver of our own |
| Palm TOF | `/tof/range`, `/tof/raw` | `gripper_sensors/tof_node` |

Both have frames on the palm, `palm_camera` and `palm_tof`, defined in
`urdf/palm_sensors.xacro`. The TOF sits at the board centre, the camera 7.2 mm
away from it on the side opposite the thumb, both 81 mm above the palm base and
looking along +Z.

## Grasping

You pick how the fingers should be arranged; the TOF picks the moment.

```bash
ros2 run gripper_grasp grasp_pilot
```

`1/2/3` choose a grasp, `space` arms, `r` releases, `a` toggles the alignment
gate, `c` calibrates the trigger, `q` quits.

| Grasp | Spread | Closes to | For |
|---|---|---|---|
| tripod | +52° / -52° | 28% | round objects, three tips evenly spaced |
| pinch | +90° / -90° | 30% | rims: index and middle together against the thumb |
| wide | 0° / 0° | 37% | widest opening |

**Calibrate the trigger before the first run.** Put an object where you want the
hand to close on it -- surrounded by the fingers, not yet touched -- and press
`c`. The current range becomes the middle of the trigger window. This cannot be
derived: `palm_tof`'s pose in the URDF is a guess, and the measured range
disagrees with it by more than a factor of two.

Contact comes from the per-joint stall flags, never from the range. Past about
11% closure the TOF is looking at the fingers, so the two signals never overlap
and there is no ambiguity to resolve.

### Closure limits are a table, not a search

Self-collision splits the closure space in two, and the far half is reachable
only by moving the spread joints along the way -- which during a grasp sweeps
the object out of the hand. It is not worth reaching either: curling that far
carries the fingertips past each other rather than together.

So `tools/gen_reach.py` measures where the edge is and writes it into
`gripper_grasp/reach_table.py`, and every grasp takes its ceiling from there.
The decision layer never asks for a pose the hand cannot reach, so
`gripper_servo` never has to refuse one. **A `BLOCKED` status now means
something bypassed the table**, not business as usual.

The limit has a cliff in it: 96% at -25° spread, 51% at -20°. `max_closure()`
takes the smaller of the two bracketing samples rather than interpolating,
because interpolating across that cliff hands back a number the hand cannot
reach.

### The gripper sees its own fingers

Measured with `ros2 run gripper_sensors tof_occlusion`, which closes the hand
slowly and logs the range against closure:

| Closure | What the TOF is measuring |
|---|---|
| 0 – 11% | the scene |
| 11 – 31% | its own fingers |
| 31% | self-collision limiting stops the closure |

A single-zone TOF has no way to pick a direction: it integrates its whole cone
and reports one distance dominated by the strongest return, and a finger a few
centimetres away beats anything further out. So **trust the range only while the
hand is nearly open**, and use the per-joint `stalled` flags for contact once it
starts closing. That is what they are for.

The measurement also says the sensor is not where this repo assumes. At 11%
closure the nearest fingertip is 91 mm from a sensor at (0, 0, 0.081) looking
along +Z, but the reading settles at 40 mm. One scalar cannot solve for three
unknowns, so the pose in `urdf/palm_sensors.xacro` stays as measured on the
board and the frame's orientation is still a guess. Get it from the CAD.

**Known CAD discrepancy:** the palm mesh has that sensor board mirrored in Y, so
in RViz the camera appears on the thumb side while the real one is opposite. The
frames carry the measured values, not the mesh's. Nothing functional depends on
it — the kinematics and the self-collision model never touch the palm mesh — but
it should be fixed upstream rather than patched here, since `tools/usd2urdf.py`
regenerates the mesh from the CAD.

The TOF's MCU streams ASCII over USB CDC at 30 Hz:

```
D,<distance_mm>,<range_status>\r\n
```

`tof_node` publishes `sensor_msgs/Range`, reporting an invalid status as `+inf`
per the message's convention, and the raw distance separately on `tof/raw` —
`Range` alone is useless for watching the sensor come alive when every reading
is invalid.

```bash
ros2 launch gripper_bringup sensors.launch.py
```

In Foxglove, set the 3D panel's **Scene > Mesh up-axis to `Z-up`**. STL carries
no up-axis, so the viewer has to be told, and the default rotates every link by
90 degrees about X -- the model comes out with each part facing the wrong way
and the palm upside down. RViz has no such setting and always looked right,
which is the tell: same `/robot_description`, two viewers, different picture
means the viewer is at fault, not the data.

That also starts `foxglove_bridge` on port 8765. Connect Foxglove to
`ws://localhost:8765` for the camera, the TOF, `/joint_states` and
`/dynamic_joint_states` (motor temperature, voltage, current, error) in one
place.

---

## Regenerating the description

The URDF and meshes are generated from the CAD, not hand-edited.

```bash
pip install usd-core trimesh scipy numpy
python3 tools/usd2urdf.py      # cad/*.usdz -> meshes/ + urdf/
python3 tools/verify_urdf.py   # every joint must reproduce the USD pose
```

`verify_urdf.py` solves each joint angle back out of the USD's own link poses.
A non-zero rotation residual means a joint frame was mis-composed — the failure
mode that silently mirrors one finger onto the wrong side.

Two things about the CAD worth knowing:

- The stage declares `metersPerUnit = 0.01` but the geometry is authored in
  metres. Read as centimetres the whole gripper is 1.7 mm across.
- `A_2_Joint` and `A_3_Joint` carry a 180° `localRot1`; the other six do not. A
  USD joint pins one frame on each body, URDF has only one, so the child frame
  has to be folded into the origin and the axis rotated with it.

---

## Packages

| Package | Gripper-specific | Contents |
|---|---|---|
| `dynamixel_msgs` | no | service definitions |
| `dynamixel_hardware` | no | serial bus, ros2_control plugin, offline CLI |
| `dynamixel_tools` | no | `dxl_cli`, `dxl_debug` |
| `three_finger_gripper_description` | yes | URDF, meshes, ros2_control tags |

The three `dynamixel_*` packages are shared unchanged with `omx_arm`.
