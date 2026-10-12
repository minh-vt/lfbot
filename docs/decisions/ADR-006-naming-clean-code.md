# ADR-006: Naming Linux-style + clean-code conventions

## Status
Accepted

## Date
2026-10-12

## Context
Tên cũ trộn lý thuyết điều khiển (PD, feedforward, backward-pass, curvature, μ) vào public API; làm AI agent + người đọc khó hiểu *việc robot làm*.

## Decision
Tên = **việc robot làm**, không phải lý thuyết; lý thuyết nằm trong comment/implementation (`docs/NAMING.md`):
- `SensorFrame`→`Readings`, `TrackPlan`→`RacePlan`, `MotorCommand`→`DriveCommand`, `TuneConfig`→`Tuning`.
- `steer_pd`→`steer_to_line`, `throttle_profile`→`choose_speed`, `mix_differential`→`drive_motors`, `resolve_line_error`→`resolve_line_offset`→`fuse_line_offset`, `bake`→`plan_speed`.
- Bỏ `docs/CONTEXT.md` + 8 thuật ngữ cũ (RawTrace, TrackCorridor, …).
- Comment: câu nối tiếng Việt, thuật ngữ SSoT giữ tiếng Anh; cấm decoration (`// ---`, `// namespace`) và restate-name; giữ why/đơn vị/range.
- Guard-clauses + early return thay nested if (cognitive complexity thấp).

## Alternatives Considered
- **Giữ tên lý thuyết** — "đúng" về mặt hàn lâm nhưng không nói việc robot làm. *Rejected*.

## Consequences
- Đọc code theo đường `main → adapter → drive` không lạc; tên tự giải thích.
- Quy ước nhất quán cho cả AI agent lẫn kỹ sư.
