# Kế hoạch Tinh chỉnh Ý tưởng: Headless Simulation Smoke-Testing & Automated CI Runner

## 1. Problem Statement
**How Might We:** Làm thế nào để giải phóng kỹ sư khỏi việc làm "máy bấm phím test dạo" bằng cách xây dựng một cơ chế kiểm thử mô phỏng hoàn toàn tự động, phi đồ họa (Headless Virtual Clock), hoàn tất 2 lượt chạy đua (Quét map + Đua bứt tốc né vật cản) trong dưới 500ms trực tiếp từ dòng lệnh `just sim-test` với các ràng buộc bất biến định lượng khắt khe?

---

## 2. Recommended Direction: "Surgical Headless Engine & Invariant Test Harness"

### 2.1. Bản chất giải pháp
Thay vì tiếp tục để vòng lặp vật lý mô phỏng bị "nhốt" bên trong vòng lặp cửa sổ Raylib của `main.cpp`, chúng ta thực hiện một phẫu thuật kiến trúc tinh gọn (Surgical Seam Extraction):
1. **Trích xuất `SimulationEngine` (`firmware/src/sim/sim_engine.hpp/.cpp`):**
   - Chứa toàn bộ logic tích phân $3 \times 3$ cảm biến quang trên ảnh sa bàn, Raycast ToF trên OBB vật cản, và bước nhảy vi sai kinematics.
   - Hoàn toàn độc lập với GUI: Không phụ thuộc vào `InitWindow()`, không chứa `toast_msg`, không có vòng lặp FPS.
   - Nhận vào cấu hình sa bàn, vị trí vật cản ban đầu, cung cấp hàm `step(dt)` và truy vấn trạng thái `get_state()`.
2. **Tái cấu trúc `main.cpp` (Desktop GUI):**
   - Biến `main.cpp` thành một "Observer / View Layer" đơn thuần: Khởi tạo Raylib window, gọi `SimulationEngine::step()` mỗi khung hình và vẽ lên màn hình.
3. **Xây dựng `test_simulation.cpp` & Target `just sim-test`:**
   - Một chương trình kiểm thử headless native chạy qua PlatformIO (`pio run -e sim_test -t exec`).
   - Nạp trực tiếp `assets/track.png` vào CPU RAM bằng Raylib `LoadImage` (không cần cửa sổ X11).
   - Chạy Run 1 ở tốc độ tính toán tối đa của CPU (bước tích phân $dt = 0.002\text{s}$, 7,500 bước xong trong ~30ms).
   - Kiểm tra các bất biến của Run 1:
     + Robot không bao giờ mất line ($\text{line\_lost} == \text{false}$ trên suốt lộ trình trừ vạch đích).
     + Quỹ đạo ghi nhận đủ số lượng waypoints ($N > 500$).
     + Tự động kích hoạt bộ tối ưu quỹ đạo $C^2$ QP khi chạm ô đích.
   - Tự động reset về vạch xuất phát, nạp quỹ đạo đua $C^2$, chuyển sang Run 2 (Sprint).
   - Chạy Run 2 ở tốc độ tối đa qua vật cản:
     + Kiểm tra bất biến cự ly vật cản: $d_{\text{obs\_actual}} \ge 150\text{ mm}$ (không bao giờ chạm vùng nguy hiểm).
     + Kiểm tra bất biến thời gian: $T_{\text{run2}} < T_{\text{run1}}$ (thời gian Lượt 2 bắt buộc phải nhanh hơn Lượt 1 ít nhất $15\%$).
     + Kiểm tra sai số lệch vạch: $\max |e_{\text{cte}}| \le 15\text{ mm}$ (nằm gọn trong hành lang $30\text{ mm}$).
     + Dừng gọn trong ô đích, kích hoạt phanh chủ động.
   - Xuất bảng báo cáo định lượng (Simulation Scorecard) và trả về exit code `0` nếu PASSED, `1` nếu FAILED.

---

## 3. Key Assumptions to Validate
- [x] **Giả định 1:** Raylib `LoadImage`, `ImageBlurGaussian`, `GetImageColor` hoạt động 100% headless trên Linux mà không cần `InitWindow()` hay DISPLAY. *(Đã kiểm chứng thực nghiệm bằng GCC: Thành công 100%)*
- [ ] **Giả định 2:** `TrajectoryPlanner::generate_racing_line()` tạo ra quỹ đạo đua từ trail của Run 1 trong môi trường headless đồng nhất tuyệt đối với kết quả trên GUI.
- [ ] **Giả định 3:** Vạch đích ở ô $x=400, y=1300$ kích hoạt điều kiện dừng chính xác sau 1 vòng chạy trọn vẹn mà không bị dừng nhầm ở thời điểm xuất phát ($x=500, y=1300$).

---

## 4. MVP Scope (Phạm vi tối thiểu giải quyết triệt để vấn đề)

### Trong phạm vi (IN):
1. **Tách module `SimEngine`:** Di chuyển `update_simulation` và `raycast_all_tof` vào một lớp sạch `SimEngine` trong `firmware/src/sim/`.
2. **Kịch bản kiểm thử headless `test_simulation`:**
   - Chạy 1 vòng hoàn chỉnh Lượt 1 (Map) và 1 vòng hoàn chỉnh Lượt 2 (Sprint có 1 vật cản tại `(1100, 300)`).
   - Kiểm tra 5 bất biến:
     1. Zero derailment (không văng vạch).
     2. Zero collision (khoảng cách vật cản luôn $> R_{\text{clearance}}$).
     3. Successful finish (dừng đúng ô đích, mode chuyển về `kIdle`).
     4. Lap time improvement ($T_2 < T_1$).
     5. Corridor bound ($|e_{\text{cte}}| \le 15\text{ mm}$).
3. **Tích hợp lệnh Just:** Bổ sung `just sim-test` vào `justfile` và môi trường `[env:sim_test]` trong `platformio.ini`.
4. **Context Engineering Rules (`AGENTS.md`):** Thiết lập file chỉ dẫn cấp cao để AI tự giác chạy `just sim-test` trước khi báo cáo kết quả cho người dùng.

### Không làm (NOT DOING - and Why):
1. **Không viết lại bộ đọc ảnh PNG tùy biến (No custom PNG decoder):**
   - *Lý do:* Raylib đã có sẵn `stb_image` bên trong, chạy được headless cực nhanh. Viết thêm thư viện đọc PNG là over-engineering vi phạm nguyên tắc Simplicity First.
2. **Không làm đồ họa ảo qua Web / WebSocket trong giai đoạn này:**
   - *Lý do:* Người dùng đã có sẵn GUI Raylib cục bộ (`just sim`) rất trực quan và mượt mà cho việc quan sát bằng mắt khi cần. Headless test sinh ra để phục vụ CI và chạy tự động, không cần streaming pixel.
3. **Không tạo trình giả lập vật lý 3D phức tạp (Không dùng Gazebo/Box2D):**
   - *Lý do:* Mô hình động học vi sai $500\text{ Hz}$ hiện tại đã ánh xạ hoàn hảo $1:1$ với phần cứng thực tế và thể lệ cuộc thi F-Maze. Thêm physics engine ngoài chỉ làm cồng kềnh và giảm tốc độ tính toán.

---

## 5. Open Questions & Technical Trade-offs
1. **Tích hợp vào `just test` hay tách riêng `just sim-test`?**
   - *Lựa chọn A (Khuyến nghị):* Giữ `just test` cho unit test toán học siêu nhẹ (< 0.3s) và bổ sung `just sim-test` cho full-lap simulation smoke-test (~0.5s). Tạo alias `just check` để chạy cả hai (`just format-check && just lint && just test && just sim-test`).
   - *Lựa chọn B:* Gộp trực tiếp `test_simulation` vào bên trong `test_core.cpp` để `just test` chạy tất cả mọi thứ trong 1 lệnh duy nhất.
2. **Kịch bản vật cản:** Nên test với 1 vật cản mặc định của sa bàn thi đấu chuẩn, hay chạy vòng lặp ngẫu nhiên vị trí vật cản?
   - *Khuyến nghị:* MVP dùng 1 vật cản chuẩn tại `(1100, 300)` với góc xoay $0^\circ$ và $45^\circ$ để đảm bảo tính tất định (determinism).
