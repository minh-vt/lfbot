# 03: Trường thế liên tục né vật cản — bỏ state machine

**What to build:** Thay state machine `AvoidPhase` + phép cộng `path_steer + avoid.bias` bằng một trường thế liên tục không-thứ-nguyên. Quyền điều khiển chuyển liên tục theo `obstacle_danger`: `steer = (1-w)·line_steer + w·(-side·k_repel)`, tốc độ `·(1-w·k_slow)`. Không snap ±1, không default-right; điểm chết đối diện (side ≈ 0) giải bằng bias liên tục (độ cong hoặc bộ nhớ rò), không phải state.

**Blocked by:** 02 — Pipeline pass-manager (trường thế là một stage sạch trong pipeline).

**Status:** ready-for-agent

- [ ] Steer là hàm liên tục của `obstacle_danger` (không bước nhảy); không còn enum `AvoidPhase`.
- [ ] Khi danger tăng dần, quyền điều khiển chuyển liên tục từ bám line sang né; tốc độ giảm liên tục.
- [ ] Không còn snap ±1 (`copysign`) và không còn lựa chọn bên mặc định.
- [ ] Run 1 trong sim sống sót qua vật cản (không va chạm).
- [ ] `just test` xanh với test mới chứng minh tính liên tục (steer theo danger là hàm liên tục).
