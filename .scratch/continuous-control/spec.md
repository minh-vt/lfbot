# Spec: Continuous dimensionless control — trường thế + multi-pass pipeline

> Trạng thái: ready-for-agent · Sinh từ hội thoại to-spec/to-tickets.

## Problem Statement

Robot LF-Bot (F-Maze-Solving) hiện không né được vật cản (chỉ giảm tốc rồi va chạm), Run 2 bị DQ (LINE LOST) vì feedforward không tái tạo đúng đường đã chạy, và codebase hard-code không scale được. Nguyên nhân gốc:

1. Né vật cản bị mô hình thành state machine rời rạc (`AvoidPhase`) + **cộng** hai tín hiệu đối lập (`path_steer + avoid.bias`) thay vì một trường thế liên tục.
2. Hợp đồng record/replay ghi sai tín hiệu: Run 1 ghi `path_steer` (thiếu thành phần né), Run 2 phát lại kênh `obstacle_side` ±1 với dấu ngược.
3. ToF sườn đặt sai góc (±26°) gây điểm mù 65° mỗi bên, khiến tín hiệu `obstacle_side` thoái hóa khi đối diện vật cản.
4. Gain bị giả: `bandwidth = 12.0` vượt spec motor (3–6 rad/s).
5. Hằng số hình học rải rác nhiều file, không có một nguồn chân lý duy nhất.

## Solution

Một hệ điều khiển **liên tục, không thứ nguyên, multi-pass**:

- **Trường thế liên tục** `U(s,y) = ½·k_line·y² + repulsive-bump` — line hút, vật cản đẩy; tín hiệu lái là gradient của thế; quyền điều khiển chuyển liên tục theo `obstacle_danger` (không state machine, không snap ±1, không default-right).
- **Kiến trúc pipeline**: per-tick streaming (motion → line → steer → avoid → speed → mix) + batch plan giữa 2 run (record → dilate → backward-pass → offset).
- **Hợp đồng record/replay đúng**: Run 1 ghi tín hiệu steer cuối cùng (gồm detour) + tốc độ thực; Run 2 phát lại nguyên vẹn.
- **SSoT `RobotLayout`**: một struct duy nhất cho mọi hằng hình học; mô hình robot trên sim luôn đồng bộ với phần cứng thật (IR cong, ToF bao quát).
- **Tuning đo được (Tier 2)**: ωn từ spec motor, `speed_limit(κ)` công thức đóng, kp/kd qua pole-placement schedule theo v.

Sim đóng vai "kính hiển vi": mọi thay đổi phần cứng được thử nghiệm trên sim trước, bằng số liệu, rồi mới áp lên robot thật.

## User Stories

1. As thí sinh, tôi muốn robot né vật cản bằng một cung mượt mà liên tục (không giật, không state machine), để không va chạm ở cả 2 run.
2. As thí sinh, tôi muốn được phép rời line để né vật cản (không phạm quy), để detour quanh gạch trên tim line hợp lệ theo Điều 5.5.
3. As thí sinh, tôi muốn Run 1 ghi đúng tín hiệu đã thực thi (gồm detour + tốc độ thực), để Run 2 phát lại chính xác đường đã chạy.
4. As thí sinh, tôi muốn Run 2 chạy nhanh hơn Run 1 nhờ out-in-out racing line, để sprint đúng giới hạn bám.
5. As kỹ sư firmware, tôi muốn mọi thông số điều khiển được suy ra/đo một lần (không gain tay), để không phải tune lại khi đổi sa bàn.
6. As kỹ sư firmware, tôi muốn kp/kd được schedule liên tục theo v bằng pole-placement, để đáp ứng không đổi ở mọi tốc độ.
7. As kỹ sư phần cứng, tôi muốn thử nghiệm bố trí cảm biến (góc ToF, IR cong/thẳng) trên sim trước khi sửa robot thật, để chọn thiết kế bằng số liệu.
8. As kỹ sư phần cứng, tôi muốn mô hình robot trên sim luôn đồng bộ với robot thật (IR cong, ToF bao quát), để kết quả sim đáng tin.
9. As nhà phát triển, tôi muốn `step()` là một pipeline các stage thuần (không phải một hàm với bool), để đọc và test từng khâu riêng.
10. As nhà phát triển, tôi muốn mọi hằng số hình học có MỘT nguồn chân lý duy nhất, để đổi robot không phải sửa nhiều file.
11. As người dùng sim, tôi muốn tua lại replay mượt (không lag/giật), để soi lỗi từng frame.
12. As người dùng sim, tôi muốn HUD dùng một ngôn ngữ thống nhất, để không rối.
13. As người dùng sim, tôi muốn chỉ một chỉ báo hướng (predictive arc) thay vì hai cái trùng lặp, để nhìn rõ hơn.
14. As kỹ sư CI, tôi muốn `just test` + `just sim-test` xanh với đúng config như GUI, để không có drift giữa test và thực chạy.

## Implementation Decisions

- **D1 — Trường thế liên tục (không thứ nguyên):** `U(s,y) = ½·k_line·y² + repulsive-bump`. Tín hiệu lái = gradient theo y. Quyền chuyển liên tục `w = obstacle_danger`: `steer = (1-w)·line_steer + w·(-side·k_repel)`, `speed·(1-w·k_slow)`. Không `AvoidPhase`, không snap ±1, không default-right. Điểm chết đối diện (side ≈ 0) giải bằng bias liên tục từ độ cong hoặc bộ nhớ rò (leaky) — không phải state.
- **D2 — Multi-pass pipeline:** per-tick streaming (motion → line → steer → avoid → speed → mix, mỗi stage là hàm thuần với IR struct) + batch plan giữa 2 run (record → dilate → backward-pass → offset). `step()` là pass-manager. Backward-pass là optimization pass (ràng buộc phi-cục-bộ, quét ngược).
- **D3 — Hợp đồng record/replay:** Run 1 ghi tín hiệu steer cuối cùng (gồm detour) + tốc độ thực; Run 2 phát lại nguyên vẹn làm feedforward, feedback chỉ là lưới an toàn. Xóa kênh `obstacle_side` ±1 riêng và dấu ngược.
- **D4 — `RobotLayout` SSoT:** một struct duy nhất cho wheelbase/track/half-length/half-width/IR span+offsets/lookahead/ToF range+FoV+góc+vị trí/tick-length. `geometry_l`, `geometry_s`, bán kính bao robot được **suy ra** (constexpr). Sim (physics + renderer) và esp32 đọc cùng một nguồn → đồng bộ phần cứng (IR cong, ToF bao quát).
- **D5 — Tuning Tier 2 đo được:** ωn từ spec motor (3–6 rad/s, không 12.0); `speed_limit(κ)` công thức đóng (xóa bảng 16 điểm); `brake_limit`, `obstacle_window` suy ra; một loader chung cho GUI và test + validate range ωn.
- **D6 — kp/kd pole-placement schedule theo v:** `kp ∝ 1/v²`, `k_psi ∝ 1/v` (damping qua heading ψ thay vì ė). Không gain tay.
- **D7 — Out-in-out:** `offset_setpoint d[t] = -k·line_curve[t]` (cắt trong cua), thay placeholder 0; Run 2 chạy setpoint `d[t]`.
- **D8 — Renderer thuần view + UX:** bỏ heading needle (giữ predictive arc), thống nhất HUD một ngôn ngữ, renderer không tính lại κ/ToF (nhận từ view-model). Bake trail để replay mượt.
- **D9 — Thí nghiệm cảm biến trên sim trước:** quét góc ToF và hình dạng IR với metric khách quan; áp kết quả vào mô hình sim rồi mới lên robot thật.

## Testing Decisions

- **Seam ngoài cao nhất:** `step(state, frame, plan, cfg) → StepResult` — giữ nguyên chữ ký, test hành vi qua nó.
- **Seam trong:** từng stage của pipeline + `field_steer(frame, plan) → steer` + `normalize_tof` — test trực tiếp.
- **Seam dữ liệu:** `RobotLayout` — test tính nhất quán của các hằng suy ra.
- **Seam thí nghiệm:** `WorldPhysics::sample()` — test headless quét cấu hình cảm biến, in bảng metric (khoảng cách phân biệt bên, sai số κ).
- Chỉ test hành vi ngoài, không test chi tiết cài đặt. Không mock, không ODE tổng hợp.
- Giữ nguyên các bất biến hiện có: line centering, deflection, anti-saturation, finish latch, backward-pass.
- `just sim-test`: Run 2 phải hoàn thành detour (không va chạm) và nhanh hơn Run 1.

## Out of Scope

- Quy hoạch quỹ đạo 2D / QP.
- μ·g·R trong drive (chỉ oracle trong sim).
- Adaptive control online.
- Calibration trên phần cứng thật (chờ phần cứng) — nhưng sim phải dùng đúng config như GUI.
- Sửa PCB / thêm cảm biến thật.

## Further Notes

- Manual Điều 5.5 cho phép rời line (chỉ cấm cắt line) — nền tảng pháp lý cho detour.
- Vô địch All Japan (APOLLON/Aegis) xác nhận: 2-pass (dò → sprint) + out-in-out là kỹ thuật thắng cuộc.
- Điểm mù ToF: 3 ToF (1 trước + 2 vuông góc, FoV 25°) để hở 65° mỗi bên; cần đo để chọn góc (~45°).
- IR cong cho κ trực tiếp + không bão hòa ở biên; thẳng chỉ cho offset.
