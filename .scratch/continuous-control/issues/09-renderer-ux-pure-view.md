# 09: Renderer UX + pure-view

**What to build:** Xóa heading needle (giữ predictive arc) để không còn hai chỉ báo hướng trùng lặp. Thống nhất HUD về một ngôn ngữ. Renderer trở thành view thuần: không tính lại κ hay vị trí ToF nội bộ, nhận các giá trị đã tính từ view-model.

**Blocked by:** 01 — RobotLayout SSoT (renderer đọc hình học từ nguồn chân lý).

**Status:** ready-for-agent

- [ ] Chỉ còn một chỉ báo hướng (predictive arc); heading needle bị xóa.
- [ ] HUD dùng một ngôn ngữ thống nhất (không còn trộn hai ngôn ngữ).
- [ ] Renderer không còn công thức vật lý (κ, vị trí ToF) nội bộ — nhận từ view-model.
- [ ] Build sim xanh; hiển thị vẫn đầy đủ (arc, HUD, vật cản, vạch đích).
