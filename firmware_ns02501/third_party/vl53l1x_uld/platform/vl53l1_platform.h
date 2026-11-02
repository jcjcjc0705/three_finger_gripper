// Platform layer for the VL53L1X ULD -- the porting seam ST leaves to the
// integrator. The upstream Arduino distribution fills this in with Wire.h;
// this port implements the same eight entry points over Linux /dev/i2c-N.
//
// Everything under core/ is ST's code, unmodified. This file and
// vl53l1_platform_linux.cpp are not.
//
// `deviceAddress` throughout is the 8-bit form the ULD uses internally
// (0x52 for a VL53L1X at its default address). Linux wants the 7-bit form,
// so the implementation shifts.
#ifndef VL53L1_PLATFORM_H_
#define VL53L1_PLATFORM_H_

#include "vl53l1_types.h"
#include "../core/vl53l1_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Binds every subsequent call to this already-open /dev/i2c-N descriptor.
/// The ULD's signatures carry no context pointer, so the fd has to live at
/// file scope -- which also means one sensor at a time per process.
void VL53L1_SetI2CFd(int fd);

int8_t VL53L1_WriteMulti(uint16_t deviceAddress, uint16_t registerAddress,
                         uint8_t * pdata, uint32_t count);
int8_t VL53L1_ReadMulti(uint16_t deviceAddress, uint16_t registerAddress,
                        uint8_t * pdata, uint32_t count);
int8_t VL53L1_WrByte(uint16_t deviceAddress, uint16_t registerAddress, uint8_t data);
int8_t VL53L1_WrWord(uint16_t deviceAddress, uint16_t registerAddress, uint16_t data);
int8_t VL53L1_WrDWord(uint16_t deviceAddress, uint16_t registerAddress, uint32_t data);
int8_t VL53L1_RdByte(uint16_t deviceAddress, uint16_t registerAddress, uint8_t * pdata);
int8_t VL53L1_RdWord(uint16_t deviceAddress, uint16_t registerAddress, uint16_t * pdata);
int8_t VL53L1_RdDWord(uint16_t deviceAddress, uint16_t registerAddress, uint32_t * pdata);

#ifdef __cplusplus
}
#endif

#endif  // VL53L1_PLATFORM_H_
