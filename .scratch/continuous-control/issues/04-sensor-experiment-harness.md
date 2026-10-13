# 04: Harness thí nghiệm cảm biến — quét ToF + IR trên sim

**What to build:** Một test headless quét cấu hình cảm biến: góc ToF sườn {0°, 26°, 45°, 90°} và mảng IR thẳng vs cong. Đo các metric khách quan: khoảng cách phân biệt bên (ToF) và sai số ước lượng κ so với ground truth (IR). In bảng so sánh. Mô hình robot sim phải phản ánh đúng từng cấu hình (IR cong, ToF bao quát) để quét.

**Blocked by:** 01 — RobotLayout SSoT (cấu hình cảm biến là dữ liệu có thể quét).

**Status:** ready-for-agent

- [ ] Một test headless chạy được, in bảng metric theo từng cấu hình cảm biến.
- [ ] Metric ToF: khoảng cách (trước vật cản) mà tín hiệu phân biệt bên vượt ngưỡng.
- [ ] Metric IR: sai số ước lượng κ so với ground truth (đường cong biết trước).
- [ ] Mảng IR cong và bố trí ToF bao quát được mô hình đúng trên sim để quét.
- [ ] Kết quả đủ để chọn góc ToF + hình dạng IR bằng số liệu, không cảm tính.
