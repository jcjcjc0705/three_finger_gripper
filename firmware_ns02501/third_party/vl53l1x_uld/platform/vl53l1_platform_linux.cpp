// Linux /dev/i2c-N implementation of the ULD's platform layer. Written for
// this port; everything under core/ is ST's and untouched.
//
// Uses I2C_RDWR rather than plain read()/write() because a VL53L1X register
// read is address-write then data-read with a *repeated start*, not a stop in
// between. Splitting it into two ioctls would put a stop between them and the
// sensor would return the wrong register.

#include "vl53l1_platform.h"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <cstring>

namespace
{

/// The ULD's signatures carry no context, so the descriptor lives here.
int g_fd = -1;

/// Comfortably above the 17 bytes the core's largest ReadMulti asks for.
constexpr uint32_t kMaxPayload = 256;

/// Registers are 16-bit and big-endian on this device.
inline void put_reg(uint8_t * buf, uint16_t reg)
{
  buf[0] = static_cast<uint8_t>(reg >> 8);
  buf[1] = static_cast<uint8_t>(reg & 0xFF);
}

/// The ULD works in 8-bit addresses (0x52); Linux wants 7-bit (0x29).
inline uint16_t addr7(uint16_t device_address) { return device_address >> 1; }

int8_t do_write(uint16_t device_address, uint16_t reg, const uint8_t * data, uint32_t count)
{
  if (g_fd < 0 || count > kMaxPayload) { return -1; }

  uint8_t buf[kMaxPayload + 2];
  put_reg(buf, reg);
  if (count > 0) { std::memcpy(buf + 2, data, count); }

  i2c_msg msg{};
  msg.addr = addr7(device_address);
  msg.flags = 0;
  msg.len = static_cast<uint16_t>(count + 2);
  msg.buf = buf;

  i2c_rdwr_ioctl_data xfer{};
  xfer.msgs = &msg;
  xfer.nmsgs = 1;
  return ::ioctl(g_fd, I2C_RDWR, &xfer) == 1 ? 0 : -1;
}

int8_t do_read(uint16_t device_address, uint16_t reg, uint8_t * data, uint32_t count)
{
  if (g_fd < 0 || count == 0 || count > kMaxPayload) { return -1; }

  uint8_t reg_buf[2];
  put_reg(reg_buf, reg);

  i2c_msg msgs[2]{};
  msgs[0].addr = addr7(device_address);
  msgs[0].flags = 0;
  msgs[0].len = 2;
  msgs[0].buf = reg_buf;
  msgs[1].addr = addr7(device_address);
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = static_cast<uint16_t>(count);
  msgs[1].buf = data;

  i2c_rdwr_ioctl_data xfer{};
  xfer.msgs = msgs;
  xfer.nmsgs = 2;
  return ::ioctl(g_fd, I2C_RDWR, &xfer) == 2 ? 0 : -1;
}

}  // namespace

extern "C" {

void VL53L1_SetI2CFd(int fd) { g_fd = fd; }

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t reg, uint8_t * pdata, uint32_t count)
{
  return do_write(dev, reg, pdata, count);
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t reg, uint8_t * pdata, uint32_t count)
{
  return do_read(dev, reg, pdata, count);
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t reg, uint8_t data)
{
  return do_write(dev, reg, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t reg, uint16_t data)
{
  const uint8_t buf[2] = {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data)};
  return do_write(dev, reg, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t reg, uint32_t data)
{
  const uint8_t buf[4] = {
    static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16),
    static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data)};
  return do_write(dev, reg, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t reg, uint8_t * pdata)
{
  return do_read(dev, reg, pdata, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t reg, uint16_t * pdata)
{
  uint8_t buf[2];
  const int8_t rc = do_read(dev, reg, buf, 2);
  if (rc == 0) { *pdata = static_cast<uint16_t>((buf[0] << 8) | buf[1]); }
  return rc;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t reg, uint32_t * pdata)
{
  uint8_t buf[4];
  const int8_t rc = do_read(dev, reg, buf, 4);
  if (rc == 0)
  {
    *pdata = (static_cast<uint32_t>(buf[0]) << 24) | (static_cast<uint32_t>(buf[1]) << 16) |
             (static_cast<uint32_t>(buf[2]) << 8) | static_cast<uint32_t>(buf[3]);
  }
  return rc;
}

}  // extern "C"
