# Phần cứng LF-Bot

Cấu hình hiện hành dùng module mua sẵn: ESP32-S3 N16R8, một thanh IR analog 8 mắt, hai DRV8871, một ToF400C VL53L1X và GY-9250; giữ pin 3S và motor 12 V. Hai MKE-S16 để dự phòng. Các module được gắn lên một PCB base làm khung xe, làm tại nhà một mặt đồng: [base](base/README.md) ([ADR-025](../docs/decisions/ADR-025-pcb-base-carrying-modules.md)). Thiết kế firmware ở [ARCHITECTURE](../docs/ARCHITECTURE.md).

## Bắt đầu

- [PCB base: thiết kế, cách route, review](base/README.md).
- [Board xe: PCB 1 mặt làm tại nhà, cầu nối 0 Ω](base/single/README.md).
- [Pin map, nguồn và cách đấu module lên board](wiring.md).
- [BOM hiện hành](../docs/HARDWARE_BOM.csv).
- [Quyết định chuyển sang module](../docs/decisions/ADR-019-module-based-hardware.md).
