# Tiến độ — continuous-control

Cập nhật: 2026-10-12.

## Done

- **T01 RobotLayout SSoT** — `lib/sensing/layout.hpp` là nguồn chân lý duy nhất cho hình học
  robot/cảm biến; `geometry_l/geometry_s/robot_radius` suy ra; sim + esp32 + test đọc một nguồn.
- **T02 Pipeline pass-manager** — `step()` là pass-manager (`estimate_motion → estimate_line →
  steer → apply_field → choose_speed → drive_motors`); accessor `RacePlan` trả trung tính khi
  chưa ready nên bỏ được các nhánh `plan.ready` thừa.
- **T03 Trường thế liên tục** — bỏ `AvoidPhase`, thay bằng `apply_field`:
  `steer = (1-w)·path_steer + w·(-side·kRepel)`, `speed·(1-w·kAvoidSlow)`, bộ nhớ rò liên tục +
  tie-break khi chính diện.

## Đo được (ticket 04 — một phần)

Chạy `just sim-avoid` (gạch đặt đúng trên đường chạy Run 1) quét góc ToF sườn:

| Góc ToF sườn | Kết quả |
|---|---|
| 26° | VA CHẠM (field không đủ) |
| 45° **chọn** | **né được, `min_clearance=+55mm`, không va chạm** |
| 60° | VA CHẠM |
| 90° | VA CHẠM (ToF vuông góc không thấy gạch chính diện) |

Kết luận: **góc ToF sườn 45°** (không phải 90°) — che vùng chéo-phía-trước, thấy gạch sớm.
Đã áp vào `kRobotLayout` (`tof_side_angle_deg = 45`, `tof_range_mm = 500`).

## Còn dở (blocker cho "0 va chạm cả 2 run")

- **T05 rejoin**: sau khi né, robot mất vạch (`LINE LOST`) vì detour vượt span cảm biến (±42mm
  < ~152mm cần để qua gạch). Cần: detour off-line có chủ đích + rejoin CÙNG line (record/replay).
  `sim-avoid` hiện PASS ở gate "gặp vật cản + không va chạm" nhưng DNF sau đó.
- **T04** (harness quét cấu hình commit được), **T06** (tuning đo được), **T07** (out-in-out),
  **T08** (replay perf), **T09** (renderer UX/pure-view) — ready-for-agent.

## Verify hiện tại

- `just test` ✅ · `just format-check` ✅ · `just lint-native` 0 warning ✅ · `just lint-esp32` ✅
- `just sim-avoid` ✅ (không va chạm) · `just sim-test` ❌ Run2 DQ (nợ cũ, đã ghi ROADMAP)
- build `sim` ✅ · build `esp32s3` ✅
