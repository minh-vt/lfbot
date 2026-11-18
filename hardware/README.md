# Phần cứng LF-Bot

Cấu hình hiện hành dùng module mua sẵn: ESP32-S3 N16R8, một thanh IR analog 8 mắt, hai DRV8871, một ToF400C VL53L1X, GY-9250 và module đo áp pin; giữ pin 3S và motor 12 V. Hai MKE-S16 để dự phòng. Các module được gắn lên một PCB base làm khung xe: [base](base/README.md) ([ADR-025](../docs/decisions/ADR-025-pcb-base-carrying-modules.md)). Firmware hiện chỉ đọc sensor; thiết kế ở [ARCHITECTURE](../docs/ARCHITECTURE.md).

## Bắt đầu

- [Pin map, nguồn và cách đấu dây](wiring.md).
- [BOM hiện hành](../docs/HARDWARE_BOM.csv).
- [Quyết định chuyển sang module](../docs/decisions/ADR-019-module-based-hardware.md).

- [PCB base: thiết kế, cách route, review](base/README.md). Bảng GPIO của base khác wiring.md; wiring.md là cách đấu dây rời cho khung in 3D cũ.

## PCB cũ

MAIN và IR-03 đã dừng ([ADR-019](../docs/decisions/ADR-019-module-based-hardware.md)) và đã xoá khỏi cây làm việc ngày 2026-11-18. Nếu cần, lấy lại từ lịch sử git (commit `3a825ba`). Bản vẽ gá motor, mẫu lỗ khay pin và datasheet motor mà base còn dùng đã chuyển vào [base/mechanical](base/mechanical/) và [base/datasheets](base/datasheets/).
