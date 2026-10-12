# ADR-005: Sim observability — TrackRules SSoT + renderer phân rã + HUD rem + curvature viz

## Status
Accepted

## Date
2026-10-12

## Context
Để verify hành vi robot mà không cần mắt người, sim phải: (a) có trọng tài ground-truth độc lập với niềm tin robot, (b) HUD co giãn theo viewport, (c) trực quan hóa được độ cong/điểm phanh để *thấy* bug điều khiển trước khi sửa.

## Decision
- **`TrackRules` (SSoT)** — thay `RaceArbiter`: phán quyết vi phạm (`kObstacleCollision`, `kOutOfBounds`, `kLineLostTimeout`, `kTimeLimitExceeded`) và chốt `completed` khi xe *dừng hẳn* trong đích; sim dừng sớm có lý do khi DQ.
- **Renderer phân rã** — `draw_hud` → `draw_top_dock` + `draw_scrubber_timeline` + `draw_telemetry_dock`; tách `draw_chassis`/`draw_ghost_chassis`; tên `poll_*`.
- **HUD rem-scale** — `root_rem(screen_h)` (neo 1080p=18px, clamp [14,38]); dock top=3rem, bottom=3.8rem, padding=1.3rem; xóa font pixel cứng.
- **Curvature viz** — Predictive Arc (tích phân heading theo `κ=2·steer/L`, amber khi phanh) + chấm Apex trên Run 1 trail + nhãn `R=X.XXm [THẲNG/CUA NHẸ/CUA GẮT]`.

## Alternatives Considered
- **Font pixel cứng (`std::max(32, …)`)** — phình to ở mọi độ phân giải. *Rejected*.
- **God renderer một hàm 140 dòng** — khó bảo trì. *Rejected* (phân rã).

## Consequences
- Người dùng nhìn là hiểu ngay (zero-friction): phân biệt "phanh vì cua hay vì vật cản", thấy apex/braking zone.
- Đây là feedback-loop (kính hiển vi) trước khi sửa điều khiển — theo nguyên tắc diagnose trước khi mổ.
