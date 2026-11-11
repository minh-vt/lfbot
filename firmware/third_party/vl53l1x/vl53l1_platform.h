#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L1X_AUTO_INCR_ADDR 0
typedef struct {
  struct { uint8_t slave_addr; } i2c_trans;
  void *device;
  int failed;
} VL53L1_Device;
typedef VL53L1_Device *VL53L1_DEV;

int8_t VL53L1_WriteMulti(VL53L1_DEV dev, uint16_t index, uint8_t *data, uint32_t count);
int8_t VL53L1_ReadMulti(VL53L1_DEV dev, uint16_t index, uint8_t *data, uint32_t count);
int8_t VL53L1_WrByte(VL53L1_DEV dev, uint16_t index, uint8_t data);
int8_t VL53L1_WrWord(VL53L1_DEV dev, uint16_t index, uint16_t data);
int8_t VL53L1_WrDWord(VL53L1_DEV dev, uint16_t index, uint32_t data);
int8_t VL53L1_RdByte(VL53L1_DEV dev, uint16_t index, uint8_t *data);
int8_t VL53L1_RdWord(VL53L1_DEV dev, uint16_t index, uint16_t *data);
int8_t VL53L1_RdDWord(VL53L1_DEV dev, uint16_t index, uint32_t *data);
int8_t VL53L1_WaitReady(VL53L1_DEV dev);
int8_t VL53L1_WaitBoot(VL53L1_DEV dev);

#ifdef __cplusplus
}
#endif
