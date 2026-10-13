# 08: Replay mượt — bake trail + snap camera

**What to build:** Vết quỹ đạo (trail) được bake vào texture tĩnh, chỉ vẽ lại khi thay đổi, thay vì duyệt lại toàn bộ điểm mỗi frame. Camera **snap** (nhảy thẳng) đến frame khi đang tua, bỏ bộ lọc làm mượt gây cảm giác lag.

**Blocked by:** None (chỉ đụng lớp renderer).

**Status:** ready-for-agent

- [ ] Trail không còn được duyệt lại từng điểm mỗi frame (bake thành texture tĩnh).
- [ ] Khi scrub, camera nhảy thẳng đến frame đang xem (không lag theo bộ lọc).
- [ ] Tua frame mượt, không giật (có thể đối chiếu FPS trước/sau).
- [ ] Build sim xanh, hành vi hiển thị giữ nguyên khi không tua.
