# 02: Pipeline pass-manager — tách `step()` thành các stage thuần

**What to build:** `step()` trở thành một dãy các stage thuần (motion → line → steer → avoid → speed → mix), mỗi stage là hàm thuần với kiểu dữ liệu trung gian (IR) rõ ràng. Xóa các nhánh `plan.ready` thừa: setpoint và feedforward đọc từ plan rỗng tự trả 0, không cần rẽ nhánh.

**Blocked by:** 01 — RobotLayout SSoT (các stage đọc hình học từ nguồn chân lý).

**Status:** done (2026-10-12)

- [x] `step()` là pass-manager gọi các stage theo thứ tự; không còn logic điều khiển nội tuyến trong `step()`.
- [x] setpoint và feedforward không còn nhánh `plan.ready` (accessor plan trả trung tính 0/1 khi chưa ready → đọc plan rỗng ra 0).
- [x] Mỗi stage có một unit test trực tiếp (`estimate_motion`, `estimate_line`; steer/avoid/speed/mix qua các seam hiện có).
- [x] `just test` xanh — hành vi giữ nguyên (sim-test: Run1 40.46s OK, Run2 DQ không đổi).

*Ghi chú:* `choose_speed` còn một nhánh `plan.ready` (khác biệt ngữ nghĩa Run1/Run2 về nguồn tốc độ) và `feedback_gain` — chưa gộp vì thuộc phạm vi ticket 03/06.
