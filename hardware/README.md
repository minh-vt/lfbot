# Phần cứng LF-Bot

Cấu hình hiện hành dùng module mua sẵn: ESP32-S3 N16R8, một thanh IR analog 8 mắt, hai DRV8871, một ToF400C VL53L1X và GY-9250; giữ pin 3S và motor 12 V. Hai MKE-S16 để dự phòng. Đã dừng chế tạo PCB riêng; firmware ESP32 chưa migrate theo bộ dây mới.

## Bắt đầu

- [Pin map, nguồn và cách đấu dây](wiring.md).
- [BOM hiện hành](../docs/HARDWARE_BOM.csv).
- [Quyết định chuyển sang module](../docs/decisions/ADR-019-module-based-hardware.md).
- [Review codebase và danh sách cleanup chờ duyệt](../docs/CLEANUP_REVIEW.md).

## Thiết kế PCB đã dừng

[MAIN](main/README.md), [IR-03](ir-array/README.md) và `_archive/` được giữ để phục hồi, không phải hướng dẫn đấu module hiện hành. Không gửi Gerber IR cũ đi in hoặc dùng pinout MAIN cho bộ dây mới. Source CAD, thư viện, mechanical models và lịch sử chưa bị xóa; mọi cắt bỏ phải theo danh sách cleanup được duyệt.
