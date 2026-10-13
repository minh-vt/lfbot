# 03: Trường thế liên tục né vật cản — bỏ state machine

**What to build:** Thay state machine `AvoidPhase` + phép cộng `path_steer + avoid.bias` bằng một trường thế liên tục không-thứ-nguyên. Quyền điều khiển chuyển liên tục theo `obstacle_danger`: `steer = (1-w)·line_steer + w·(-side·k_repel)`, tốc độ `·(1-w·k_slow)`. Không snap ±1, không default-right; điểm chết đối diện (side ≈ 0) giải bằng bias liên tục (độ cong hoặc bộ nhớ rò), không phải state.

**Blocked by:** 02 — Pipeline pass-manager (trường thế là một stage sạch trong pipeline).

**Status:** done (2026-10-12)

- [x] Steer là hàm liên tục của `obstacle_danger` (không bước nhảy); không còn enum `AvoidPhase`.
- [x] Khi danger tăng dần, quyền điều khiển chuyển liên tục từ bám line sang né; tốc độ giảm liên tục.
- [x] Không còn snap ±1 (`copysign`) và không còn lựa chọn bên mặc định (thay bằng bộ nhớ rò liên tục + tie-break khi chính diện).
- [x] Run 1 trong sim sống sót qua vật cản (không va chạm) — chứng minh bởi `just sim-avoid` (gạch trên đường chạy): `min_clearance=+55mm, no collision`.
- [x] `just test` xanh với `test_field_continuous` (steer liên tục theo danger, không nhảy bậc).

*Ghi chú:* sau khi né, robot mất vạch (`LINE LOST`) vì detour off-line vượt span cảm biến — phần rejoin thuộc ticket 05.
