# 06: Tuning đo được (Tier 2) + công thức đóng

**What to build:** ωn lấy từ spec motor (3–6 rad/s) thay giá trị giả `12.0`; `speed_limit(κ)` thành công thức đóng (xóa bảng tra 16 điểm); `brake_limit` và `obstacle_window` được suy ra từ công thức; một loader cấu hình chung cho cả GUI và test, kèm validate dải ωn.

**Blocked by:** 01 — RobotLayout SSoT (các hằng suy ra đọc từ nguồn chân lý).

**Status:** ready-for-agent

- [ ] Không còn giá trị ωn = 12.0; ωn nằm trong dải spec motor (3–6 rad/s).
- [ ] `speed_limit(κ)` là hàm đóng (không bảng tra), liên tục theo độ cong.
- [ ] `brake_limit`, `obstacle_window` được suy từ công thức (không số tay).
- [ ] Loader cấu hình chung dùng cho cả GUI và test; ωn ngoài dải bị từ chối.
- [ ] `just test` và `just sim-test` xanh, không còn gain tay.
