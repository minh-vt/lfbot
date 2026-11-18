# MAIN LF-Bot — đã dừng

> **Thiết kế PCB đã dừng theo [ADR-019](../../docs/decisions/ADR-019-module-based-hardware.md).** Dùng [pin map module](../wiring.md) và [BOM hiện hành](../../docs/HARDWARE_BOM.csv); không gia công, route tiếp hoặc đấu dây theo board này.

Board mang DevKit ESP32-S3 và hai IC A4950 cũ. Source KiCad vẫn mở được bằng KiCad 10 qua [main.kicad_pro](main.kicad_pro); [schematic](main.kicad_sch) và [PCB](main.kicad_pcb) là bản lưu lịch sử: hai mặt linh kiện, 14 SMD tín hiệu ở B.Cu, chưa route xong, chưa kiểm chứng Z/lắp ráp/nhiệt.

Tài liệu placement/routing/BOM của board đã được archive và bỏ khỏi cây làm việc. Lý do dừng PCB nằm ở [ADR-019](../../docs/decisions/ADR-019-module-based-hardware.md).

Giữ `Main.pretty/`, `Main.kicad_sym`, library table, [cơ khí](mechanical/) và [datasheet motor](datasheets/GA12-N20.pdf) để phục hồi. `Bracket.step` vẫn là nguồn gá motor được chủ dự án xác nhận. Không gửi Gerber cũ đi in.
