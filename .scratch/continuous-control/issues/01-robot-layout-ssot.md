# 01: RobotLayout SSoT — một nguồn chân lý hình học

**What to build:** Một struct hình học duy nhất là nguồn chân lý cho mọi kích thước robot/cảm biến (mm). Các hằng không-thứ-nguyên (`geometry_l`, `geometry_s`, bán kính bao robot) được **suy ra** từ struct này thay vì tay tính. Mô hình robot trên sim (vật lý + renderer) và adapter esp32 đọc cùng một nguồn, để mọi thay đổi phần cứng (IR cong, ToF bao quát) tự động đồng bộ.

**Blocked by:** None (có thể bắt đầu ngay).

**Status:** done (2026-10-12)

- [x] Một struct hình học duy nhất chứa: wheelbase, track, half-length, half-width, IR span + offsets, lookahead, ToF range/FoV/góc/vị trí, tick length.
- [x] `geometry_l`, `geometry_s`, bán kính bao robot được suy ra từ struct — không còn số tay ở bất kỳ nơi nào khác.
- [x] Mọi nơi từng hard-code các hằng này (drive tuning, world physics, renderer, esp32) đọc từ một nguồn duy nhất.
- [x] Mảng IR cong và bố trí ToF bao quát được mô hình đúng trên sim từ cùng struct.
- [x] `just test` và build sim xanh; đổi một giá trị trong struct làm toàn bộ hệ thống cập nhật theo.
