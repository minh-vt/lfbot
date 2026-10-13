# 05: Chốt layout + sửa hợp đồng record/replay

**What to build:** Run 1 ghi đúng tín hiệu steer cuối cùng (gồm thành phần né detour) + tốc độ thực; Run 2 phát lại nguyên vẹn làm feedforward. Xóa kênh `obstacle_side` ±1 riêng và lỗi dấu ngược. Áp layout đo được từ ticket 04, đồng bộ mô hình robot sim (IR cong, ToF bao quát).

**Blocked by:** 03 — Trường thế liên tục (tạo ra tín hiệu steer cuối cùng để ghi) · 04 — Harness thí nghiệm cảm biến (cung cấp layout đo được).

**Status:** ready-for-agent

- [ ] Run 1 ghi tín hiệu steer cuối cùng (bao gồm thành phần né), không phải `path_steer` chưa né.
- [ ] Không còn kênh `obstacle_side` ±1 riêng; dấu né không thể bị đảo ngược.
- [ ] Run 2 phát lại tín hiệu đã ghi và hoàn thành detour (không va chạm).
- [ ] Layout ToF/IR từ ticket 04 được áp vào mô hình sim, đồng bộ với phần cứng.
- [ ] `just sim-test`: Run 2 không DQ vì LINE LOST.
