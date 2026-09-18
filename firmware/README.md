# Palm TOF firmware

The VL53L1X hangs off a Seeed XIAO SAMD21, which streams ASCII over USB CDC to
`gripper_sensors/tof_node`. Nothing in this directory is built by colcon — it is
flashed with the Arduino toolchain and kept here so the board is reproducible.

| File | What it is |
|---|---|
| `tof_original.ino` | What runs on the board today |
| `tof_vl53l1x.ino` | Diagnostic build: reports the I2C line state and scans the bus |
| `tof_original.ino.bin` | Compiled image of `tof_original.ino`, 39064 bytes |
| `tof_vl53l1x_CURRENT.uf2` | Flash dump taken 2026-08-25, before the app row was erased |

---

## Wire protocol

```
D,<distance_mm>,<range_status>\r\n     30 Hz
ERR,VL53L1X init failed (check wiring/power)
```

`range_status` 0 means the reading is valid. `tof_node` publishes anything else
as `+inf` on `/tof/range`, and the raw millimetres on `/tof/raw` regardless.

The diagnostic build adds `I,`-prefixed lines. `tof_node` counts those as
unparsed frames and warns every hundredth, which is noisy but harmless.

## Sensor settings

| | |
|---|---|
| Library | SparkFun VL53L1X (wraps the ST ULD API) |
| Distance mode | **Short** — about 1.3 m, which is why anything further reads status 4 |
| Timing budget | 33 ms |
| Inter-measurement | 33 ms |
| I2C | 400 kHz, address 0x29 |

`tof_node` declares `max_range: 4.0`, which does not match Short mode. It
affects nothing, but the numbers disagree.

---

## Flashing

```bash
arduino-cli core install Seeeduino:samd@1.8.6 \
    --additional-urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
arduino-cli lib install "SparkFun VL53L1X 4m Laser Distance Sensor"
arduino-cli lib install VL53L1X                       # Pololu, for the diagnostic build
arduino-cli compile --fqbn Seeeduino:samd:seeed_XIAO_m0 tof_original
arduino-cli upload -p /dev/ttyACM0 --fqbn Seeeduino:samd:seeed_XIAO_m0 tof_original
```

Or drop a `.uf2` on the bootloader drive: open the port at 1200 baud and drop
DTR, and the board reappears as a mass storage device named `Arduino`.

> **Entering the bootloader destroys the sketch.** The Arduino SAMD core's
> `banzai()` erases the first flash row — the vector table and the start of
> `Reset_Handler` — before resetting, so the bootloader will not start the app
> again. That is by design: the IDE flashes something new immediately after.
> A power cycle does not undo it. Once you touch 1200 baud you are committed to
> flashing something.
>
> This is how the original firmware was lost on 2026-08-25.

## When the TOF goes quiet

`Wire`'s write path spins on `INTFLAG.MB` with no timeout
(`SERCOM.cpp:564`), so an I2C line held low hangs the sketch before it can
report anything — not even the `ERR,` line, because it never returns from
`sensor.begin()`. Total silence is the symptom.

Flash `tof_vl53l1x.ino` to tell the cases apart. It reads the bus as plain
GPIO before touching `Wire`, so it cannot hang:

```
I,lines,SDA=1,SCL=1     both idle high, bus is healthy
I,lines,SDA=0,SCL=0     held low: short, or the sensor has no power
I,pullup,SDA=0,SCL=0    the ~40k internal pull-up cannot lift it either
```

On 2026-08-26 that read `0,0` because the XIAO's GND and 3V3 wires had broken
off at the solder joints. The palm wiring flexes with every finger movement;
strain relief matters more than joint strength there.

---

## Recovery note

The sources here were reconstructed twice. The first time from a disassembly of
the flash dump, after `banzai()` erased the app; the second from the Arduino
build cache under `~/.cache/arduino/sketches/`, after the working copy was
deleted. That cache is pruned automatically. **Keep this directory.**
