# 07: Out-in-out `d[t]` — racing line

**What to build:** `offset_setpoint d[t] = -k·line_curve[t]` (cắt trong cua) thay placeholder 0. Run 2 chạy setpoint `d[t]` để theo đường racing line, nhanh hơn Run 1.

**Blocked by:** 05 — Hợp đồng record/replay (Run 2 phát lại đúng) · 06 — Tuning đo được (`speed_limit(κ)` đúng để suy offset).

**Status:** ready-for-agent

- [ ] `d[t]` khác 0, tỷ lệ với độ cong `line_curve[t]` (cắt trong cua).
- [ ] Run 2 chạy setpoint `d[t]` (line hút về `d[t]` thay vì 0).
- [ ] `just sim-test`: Run 2 hoàn thành và nhanh hơn Run 1.
