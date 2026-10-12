# ADR-003: First-principles tuning — zero hand-tuned constants

## Status
Accepted

## Date
2026-10-12

## Context
Mọi tham số điều khiển (`steer_kp`, `steer_kd`, `corner_limit`, `base_throttle`, `avoid_strength`, …) ban đầu là "magic number" chọn tay. Không muốn tune thủ công, không muốn đoán số.

## Decision
Mỗi tham số `Tuning` chỉ thuộc 1 trong 3 loại: **SUY** (hình học + dt) · **ĐO** (calibration once) · **XÓA** (thay bằng luật vật lý). Nền tảng là plant line-following `ë = (v²/(L·s))·u` (double integrator):

- **kp/kd qua pole-placement, schedule theo v** (encoder): `kp = ωn²·L·s/v²`, `k_psi = 2ζωn·L/v` (ζ=0.7). Không còn `steer_kp/steer_kd`.
- **Hai trần tốc độ độc lập:** tracking `v ≤ ωn·√(s·budget/|κ|)` + friction `v ≤ √(μg/|κ|)`, chạy `min`. Mất vạch ở cua = vượt trần tracking, không phải trượt lốp.
- **`speed_limit(κ)` = bảng đo slip-boundary** (circle-test đến trượt), `brake_limit = a_brake·dt/v_max` (phanh gấp).
- **Xóa:** `avoid_slowdown` → luật phanh `v²≤2·a_brake·d`; `avoid_strength` → detour side-commit; `offset_gain` → `argmin ∫ds/v`; `sprint_scale` → 1.0; `base_throttle` → envelope + error-budget; `obstacle_window = ceil((ToF_range+half_len)/tick_len)`.

Còn đúng 3 lựa chọn hợp pháp: ζ=0.7, margin 0.9, ωn (chặn bởi motor bandwidth).

## Alternatives Considered
- **Auto-tune 3 tầng (derive/measure/trim) với `tuning.hpp`** — speculative, bị revert (AS-13); viết lại thành first-principles ở đây. *Superseded*.
- **Grip theo steer** — index theo control signal, không theo geometry. *Rejected*.

## Consequences
- Giảm ~10 tham số giả định → ~2 tham số đo được.
- **Phát hiện:** tích gyro để có ψ là **positive feedback** (ω là yaw tuyệt đối, không quy chiếu về tiếp tuyến line khi chưa biết κ) → ψ lấy từ `de/ds`; gyro dành slip detection/calibration.
- Calibration on-robot (circle-test, MPU6050) còn **deferred** — xem `docs/ROADMAP.md`.
