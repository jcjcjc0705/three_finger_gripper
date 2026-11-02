# Hardware

Wiring and configuration of the ns02501/NT98532 EVB.

## Parts

| Part | Connects to | Device on the board |
|---|---|---|
| 8 XC330-M288-T motors, daisy chained | Waveshare Bus Servo Adapter, on UART2 | `/dev/ttyS1`, 1 Mbaud |
| VL53L1X TOF, address 0x29 | I2C, with external pull-ups | `/dev/i2c-2` |
| USB camera, UVC, MJPEG | The board's only USB port | `/dev/video0` |
| PC | Ethernet, directly | PC 192.168.0.1, board 192.168.0.3 |
| PC | USB serial adapter on the console, 115200 | `/dev/ttyS0` |

## Wiring

### Motors

```
J31 pin 7  (TX)   adapter signal pin
J31 pin 9  (RX)   adapter signal pin
J31 pin 8  (GND)  adapter GND
3.3 V             adapter VCC, on its logic header
5 V               adapter power terminal
adapter servo connector  to the XC330 daisy chain
```

⚠ **The adapter's power terminal goes straight to the motors. XC330-M288-T is a 5 V motor
(3.7 to 6.0 V), and these adapters are aimed at 6-12 V servos: 12 V destroys all eight.**
Measure VCC to GND on the servo connector before a motor goes on, and use a meter for its
pin order rather than wire colour.

- The motors need their own supply. 4.5 V at idle is too little for a grip, 4.9 V is fine.
- If `dxl_offline scan` finds nothing and reports no error, swap the TX and RX wires first.
- With only the power terminal connected, the adapter's logic `VCC` reads 0 V. It is an input.
- The same UART is on the 30-pin expansion header, pins 8 (TX) and 10 (RX).
- These pins are also JTAG: an attached ICE pod fights the UART.

### TOF

On the 30-pin expansion header:

| Signal | Pin |
|---|---|
| `VCC3.3V` | 1, 17 |
| `EXT_SDA` | 3 |
| `EXT_SCL` | 5 |
| `GND` | 6, 9, 14, 20, 25 |

**External pull-ups on SDA and SCL are required.** The board's internal ones are off.

`5V_IN` on pins 2 and 4 is the board's own input, not a supply for the motors.

## Board configuration

### Pinmux

Set in the board image, in `configs/Linux/cfg_<project>/nvt-top.dtsi` of the SDK:

```
uart {pinmux = <0x101>;};     // 0x1 console | 0x100 UART2_5, on D_GPIO3/4
i2c  {pinmux = <0x10000>;};   // I2C3_1, on P_GPIO21/22
```

The `uart` value must keep the `0x1`: without it the console stops working.

To check it, read the table this prints on the serial console: `D_GPIO` 3 and 4 and `P_GPIO`
21 and 22 should say `FUNCTION`. It goes to the console directly, so `grep` cannot filter it.

```bash
cat /proc/nvt_info/nvt_pinmux/gpio_summary
```

To set it until the next boot, on an image without it:

```bash
echo uart 0x101 > /proc/nvt_info/nvt_pinmux/pinmux_set
echo i2c 0x10000 > /proc/nvt_info/nvt_pinmux/pinmux_set
```

### Kernel

USB and V4L2 are off in every stock ns02501 defconfig. For the camera, in
`ns02501_ipc_a32_evb_defconfig_release`:

```
CONFIG_USB_SUPPORT=y
CONFIG_USB=y
CONFIG_USB_EHCI_HCD=y
CONFIG_USB_EHCI_HCD_NVTIVOT=y
CONFIG_MEDIA_SUPPORT=y
CONFIG_MEDIA_CAMERA_SUPPORT=y
CONFIG_MEDIA_USB_SUPPORT=y
CONFIG_USB_VIDEO_CLASS=y
```

Leave `CONFIG_USB_GADGET` off: host and gadget share the controller.

## Checking it

With `uros_node` stopped, from `/usr/bin/gripper` on the board:

```bash
./dxl_offline scan               # 8 motors, model 1240
i2cdetect -y -r 2 0x28 0x2a      # 29
./tof_read --short               # about 30 Hz
./camera_capture --list          # MJPG sizes
```

| Symptom | Check, in this order |
|---|---|
| `scan` finds nothing, no error | TX/RX swapped, pinmux, motor power, then the motors' baud: `./dxl_offline --baud 57600 scan` |
| `i2cdetect` blank | Pull-ups, pinmux, wiring. `/dev/i2c-2` is the only right bus |
| `uros_node` will not start | `/tmp/uros.log` on the board, or `docker/scripts/agent.sh logs node` |
| No data on the PC | PC address 192.168.0.1, `docker/scripts/agent.sh logs agent` |
