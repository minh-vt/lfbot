# ADR-004: ESP32 toolchain — ESP-IDF + FreeRTOS, bỏ Arduino

## Status
Accepted

## Date
2026-10-12

## Context
Firmware cần 500Hz hard real-time trên ESP32-S3 dual-core. Arduino (`delay()` + `micros()`) jitter ~350–400Hz, không đáp ứng.

## Decision
`framework = espidf` (ESP-IDF 6.1), bỏ Arduino:
- **FreeRTOS** task pin core 1, `vTaskDelayUntil` chu kỳ cố định 2ms (500Hz thật, `dt` hằng).
- **ADC-DMA** (continuous) cho 8 kênh IR — hiện dùng `adc_oneshot` trước (OPT-2 ⚠️, DMA có sẵn trong IDF 6.1).
- **PCNT** quadrature (chiều + ×4) cho encoder, thay SW ISR.
- **MCPWM** 2 cầu H 20kHz (complementary + dead-time).
- **I2C master mới** + VL53L0X (XSHUT sequencing 3 cảm biến); finish pin giả định.
- **Static allocation**: `RacePlan` fixed-capacity, không `std::vector` trong vòng 500Hz.

## Alternatives Considered
- **Arduino framework** — jitter, `analogRead()` blocking, SW ISR rớt xung. *Rejected*.
- **Compile DB merge host + xtensa** — một DB không phục vụ 2 ABI (glibc vs newlib); `compiledb` native-only, esp32 lint = cppcheck (clangd chấp nhận blurry). *Superseded*.

## Consequences
- Build cần venv `uv` (`.venv`) + `pio`; env `esp32s3` cần `build_unflags = -std=gnu++11` (framework ép C++11 → thiếu `std::clamp`).
- `src/CMakeLists.txt` riêng: espidf chỉ biên dịch `esp32/*`, không `src/sim`.
