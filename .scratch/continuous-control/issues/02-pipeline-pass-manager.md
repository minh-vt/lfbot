# 02: Pipeline pass-manager — tách `step()` thành các stage thuần

**What to build:** `step()` trở thành một dãy các stage thuần (motion → line → steer → avoid → speed → mix), mỗi stage là hàm thuần với kiểu dữ liệu trung gian (IR) rõ ràng. Xóa các nhánh `plan.ready` thừa: setpoint và feedforward đọc từ plan rỗng tự trả 0, không cần rẽ nhánh.

**Blocked by:** 01 — RobotLayout SSoT (các stage đọc hình học từ nguồn chân lý).

**Status:** ready-for-agent

- [ ] `step()` là pass-manager gọi các stage theo thứ tự; không còn logic điều khiển nội tuyến trong `step()`.
- [ ] setpoint và feedforward không còn nhánh `plan.ready` (đọc plan rỗng → 0).
- [ ] Mỗi stage có một unit test trực tiếp (hành vi ngoài, không chi tiết cài đặt).
- [ ] `just test` xanh — hành vi bên ngoài giữ nguyên so với trước khi tách.
