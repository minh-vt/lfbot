# ROADMAP — Phase sau

Các việc còn dở (deferred phần cứng / chưa hoàn thành) chuyển từ tickets sang đây.

## 1. Calibration on-robot — đo envelope thật (chờ phần cứng)
- `speed_limit(κ)` = bảng slip-boundary: circle-test tăng tốc đến trượt (phát hiện qua gyro / encoder L-R mismatch / line-loss), lưu NVS.
- `brake_limit` = `a_brake·dt/v_max` (phanh gấp, đo Δv/tick).
- IMU MPU6050: driver I2C + calibrate gyro bias; gyro dùng cho **slip detection** (KHÔNG tích ψ — positive feedback, ADR-003).
- Mode calibration trong `esp32`.
- *Nguồn:* tickets 23, 34, 17(wiring).

## 2. Obstacle detour hoàn chỉnh — 0 collision cả 2 run
- State machine `kIdle→kBrake→kCommit→kRejoin` đã có (ticket 36) nhưng `sim-test` vẫn va chạm.
- Cần: chọn bên thông minh theo ToF sườn (`tof_left`/`tof_right`), arc detour off-line rồi rejoin **cùng** line (Q-15).
- *Nguồn:* tickets 03, 36.

## 3. Offset profile `d[t]` tối ưu — argmin ∫ds/v
- Hiện là heuristic `-0.5·line_curve` (placeholder ticket 25).
- Tối ưu thật cần `speed_limit(κ)` đo được (mục 1).

## 4. Run 2 ổn định + sprint nhanh hơn Run 1
- `sim-test`: **Run 1 PASS (40.46s), Run 2 DQ (LINE LOST)**.
- Gốc: plan 1.5-D ghi *steer* (không ghi *vị trí ngang*) → feedforward không tái tạo chính xác đường; và cùng tracking-limit cho 2 run → Run 2 không nhanh hơn an toàn.
- Hướng: ghi thêm lateral position vào plan, hoặc tách envelope Run 2 (feedforward cho phép nhanh hơn).
- *Nguồn:* spec "Issue mở", ticket 35.
