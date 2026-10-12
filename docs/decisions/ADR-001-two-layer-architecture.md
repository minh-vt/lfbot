# ADR-001: Kiến trúc 2 tầng — drive không-thứ-nguyên + imperative-shell adapters

## Status
Accepted

## Date
2026-10-12

## Context
Robot LF-Bot thi F-Maze-Solving: sa bàn **bí mật**, 2 run, 500Hz hard real-time trên ESP32-S3. Kiến trúc cũ có God-class `SimEngine`, 3 struct state trùng lặp chép tay ~15 trường mỗi tick, và vật lý (mm/m/s/μ/g) trộn vào trong trí tuệ — những đại lượng robot thật không quan sát được. Cần một trí tuệ duy nhất chạy nguyên vẹn trên sim lẫn robot thật.

## Decision
Tách 2 tầng (functional core + imperative shell):
- **`lib/drive/`** — trí tuệ thuần (free functions, data-oriented), **không-thứ-nguyên**: chỉ thấy `[-1,1]`, tick thô, `dt`. Một seam ngoài duy nhất `step(state, frame, plan, cfg) → command`.
- **Adapters (`src/sim/`, `src/esp32/`)** — imperative shell, sở hữu tài nguyên (Raylib `Image`, peripheral ESP32), đổi vật lý ↔ dimensionless.
- **`lib/sensing/`** — layer base: chân lý cảm biến + chuẩn hóa (`normalize_ir`, `normalize_tof`); cả 2 adapter tham chiếu, kernel không phụ thuộc.

## Alternatives Considered
- **Kernel mang đơn vị vật lý (mm, m/s, μ)** — robot thật không đo được μ/g/R; muốn dùng chung trên ESP32 sẽ phải giả lập các đại lượng không có. *Rejected*.
- **Module `layout/` riêng** — ban đầu đặt tên theo nội dung (vị trí cảm biến), đổi thành `sensing/` vì thực chất là chuẩn hóa; tên theo *vai trò* (ADR-006). *Superseded*.

## Consequences
- Cùng một `step()` chạy nguyên vẹn trên sim + ESP32; test headless không GUI.
- Mọi hằng vật lý (mm/tick, μ, g) chỉ tồn tại trong adapter/oracle, không rò vào drive.
- `Readings`/`DriveCommand` là hợp đồng duy nhất giữa drive và 2 adapter.
