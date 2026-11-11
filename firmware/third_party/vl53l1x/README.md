# VL53L1X ULD — vendor code

STMicroelectronics ULD 3.3.0, retrieved from the Paparazzi mirror:
https://github.com/paparazzi/paparazzi/tree/9770b4b9e2fe5f9f4acff1bb44a65d28921e89b3/sw/airborne/peripherals

Files: `vl53l1x_api.c`, `vl53l1x_api.h`. The BSD-3-Clause option in their
original notices is selected; notices are retained in both source files.
This is ST driver code from a mirror, not a direct ST package download.
The attempted direct STSW-IMG009 download failed.

Original SHA256:
- C: 4248dd4d82977ebc131ef19c3f8ef3e2550f851f90115e316d2687c100b56969
- H: 257685313f95c870c07eed3f95f1496dcb60062d6a20b2f88eb8e5735be101b3

Local changes to C: replace the three unbounded ready/boot polling loops with
adapter helpers with a 500 ms wall-clock timeout and explicit error returns;
remove their now-unused local variables. Header API is unchanged.

`vl53l1_platform.h` is the LF-Bot port, not the mirror's STM32/Paparazzi adapter.
The port latches the first I2C failure because some ULD functions overwrite
intermediate status values. Address auto-increment is disabled. The only
instance uses an IDF 7-bit 0x29 device handle. Polling/init is never a control tick.
Driver/configuration has been cross-compiled, not verified on this physical module.
