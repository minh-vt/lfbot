# ADR-002: Chiến lược 2-pass + profile 1.5-D

## Status
Accepted

## Date
2026-10-12

## Context
Sa bàn bí mật → robot phải tự dò (Run 1) rồi sprint (Run 2) on-chip trong 3 phút. Cần vừa *sống sót* khi chưa biết gì, vừa *chạy đúng giới hạn bám* khi đã có map — mà không dùng quy hoạch 2D (vi phạm hợp đồng dimensionless + rủi ro).

## Decision
Profile liên tục 1-D theo tick (arc-length), không segment map:
- **Run 1 ghi, Run 2 phát lại** — cùng một controller `step()`, không biết mode: plan rỗng = Run 1 (feedback-dominant), plan đầy = Run 2 (feedforward-dominant). `steer = feedforward + PD(e)`.
- **Backward-pass tick-space** (`plan_speed`): quét ngược `speed_limit[t] = min(grip(t), speed_limit[t+1] + brake_limit)` — ràng buộc phanh là *phi-cục bộ* (giảm ga *trước* cua).
- **`line_curve[t]`** (geometry thuần, low-pass) tách khỏi `planned_steer[t]` (control signal lẫn nhiễu PD) — feedforward sạch + speed_limit theo độ cong.
- **`obstacle_side[t]`** ghi ở Run 1, giãn cửa sổ ±W tick (chịu trôi odometry) → Run 2 né sớm; ToF phản ứng là safety net.
- **`offset_setpoint d[t]`** (racing-line 1.5-D): lệch ngang ≤ sensor span. Luật Q-15: offset trên CÙNG đoạn line hợp lệ; cắt line (đi tắt qua đoạn KHÁC) = DNF.

## Alternatives Considered
- **Segment map rời rạc (3 bin, debounce)** — mất thời gian lookup/debounce khi vào cua. *Rejected*.
- **Quy hoạch quỹ đạo 2D / QP** — cần pose 2D + đơn vị vật lý, vi phạm dimensionless; rủi ro cao. *Rejected* (Out of Scope).
- **Grip theo steer (`corner_limit·|steer|`)** — steer là control signal, không phải geometry. *Superseded* bởi `speed_limit(line_curve)` (ADR-003).

## Consequences
- Robot tự map + dựng profile on-chip; Run 2 chạy đúng envelope.
- **Nợ:** plan 1.5-D ghi *steer*, không ghi *vị trí ngang* → feedforward Run 2 không tái tạo chính xác đường (Run 2 DQ, xem `docs/ROADMAP.md`).
