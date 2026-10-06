# ADR-0003: Headless Raylib Simulation Engine for Sub-Second Continuous Integration

## Status
Accepted

## Date
2026-10-06

## Context
Trước đây, toàn bộ logic tích phân vật lý mô phỏng và đọc cảm biến từ sa bàn ảnh bitmap bị khóa cứng bên trong vòng lặp đồ họa của tệp [`firmware/src/sim/main.cpp`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/main.cpp).
Vòng lặp này phụ thuộc trực tiếp vào các hàm hiển thị cửa sổ của Raylib (`InitWindow`, `BeginDrawing`, `DrawTexture`) và các biến trạng thái giao diện (`toast_msg`, `toast_timer`).

Hậu quả:
1. **Bế tắc kiểm thử tự động:** Không có cách nào kiểm tra tính đúng đắn của toàn bộ chu trình 2 lượt chạy trong CI nếu không mở cửa sổ đồ họa X11/OpenGL.
2. **Lãng phí thời gian khủng khiếp:** Một vòng chạy 15-20 giây trong cửa sổ GUI bị giới hạn bởi tốc độ đồng hồ thực 60 FPS (tốn đúng 15-20 giây của con người). Cả hai lượt chạy tốn tới 30-50 giây.
3. **Mất liên kết tự động của AI Agent:** Agent không thể tự kiểm chứng sau khi sửa mã nguồn, biến con người thành một "máy bấm nút khói" (Biological smoke-tester).

## Decision
1. **Trích xuất Seam kiến trúc [`SimEngine`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/sim_engine.hpp#L18):**
   - Đóng gói toàn bộ trạng thái và các bước cập nhật vật lý, đọc cảm biến ảnh bitmap, và raycast ToF vào class `SimEngine`.
   - `SimEngine` hoàn toàn không phụ thuộc vào cửa sổ đồ họa hay thao tác chuột/phím. Nó nhận bước thời gian cố định $dt = 0.002\text{ s}$ ($500\text{ Hz}$) và thực hiện tính toán thuần túy trên CPU.
2. **Khai thác năng lực Headless CPU của Raylib:**
   - Chứng minh thực nghiệm: Các hàm `LoadImage()` (sử dụng stb_image), `ImageBlurGaussian()`, và `GetImageColor()` của Raylib hoạt động trực tiếp trên mảng byte bộ nhớ heap của CPU mà không cần khởi tạo context OpenGL hay màn hình X11.
   - Cho phép nạp sa bàn thực tế [`firmware/assets/track.png`](file:///home/minhvt/Projects/lfbot/firmware/assets/track.png) và thực hiện tích phân vùng $3 \times 3$ điểm ảnh mô phỏng vết quang học $\varnothing 5\text{ mm}$ hoàn toàn trong chế độ không đồ họa (Headless).
3. **Xây dựng bộ kiểm thử tích hợp tự động [`firmware/test/test_simulation.cpp`](file:///home/minhvt/Projects/lfbot/firmware/test/test_simulation.cpp):**
   - Chạy toàn bộ kịch bản 2 lượt chạy (Run 1: Thăm dò, Run 2: Đua tốc độ cao) cùng 6 kịch bản thử thách khắc nghiệt (adversarial tests): lệch xuất phát, né vật cản động, dừng khẩn cấp, chạy ngược chiều, và kiểm tra giới hạn thời gian.
4. **Cung cấp lệnh kiểm thử tốc độ cao:**
   - Bổ sung lệnh `just sim-test` (môi trường `sim_test` trong PlatformIO).
   - Tốc độ xử lý: Hoàn tất 30+ giây vật lý của 2 lượt chạy trong **dưới 0.5 giây CPU** (~500x nhanh hơn đồng hồ thời gian thực).
   - Tích hợp vào lệnh kiểm tra toàn diện `just check` (`format-check` + `test` + `sim-test`) hoàn tất trong ~1.1 giây.

## Alternatives Considered

### 1. Dùng Xvfb (X Virtual Framebuffer) để chạy GUI ở chế độ ẩn
- **Ưu điểm:** Không cần refactor tách mã `main.cpp`.
- **Nhược điểm:** Phụ thuộc vào công cụ ngoài hệ thống (Xvfb); vẫn bị giới hạn tốc độ bởi FPS rendering ảo; tốn CPU render pixel vô ích; dễ phát sinh lỗi driver đồ họa trong container CI.
- **Lý do loại bỏ:** Cồng kềnh, chậm chạp và không triệt để.

### 2. Kiểm thử giả lập mô hình toán rời rạc (Toy Analytical Mocks)
- **Ưu điểm:** Rất nhanh (micro-giây).
- **Nhược điểm:** Không kiểm tra được hiện tượng nhiễu quang học thực tế trên sa bàn ảnh PNG thật; bỏ qua tương tác phức tạp giữa cụm mắt đọc và mép vạch cong $90^\circ$.
- **Lý do loại bỏ:** Không đảm bảo tính trung thực (fidelity) với môi trường thực tế.

## Consequences
- Chu kỳ phản hồi phát triển rút ngắn từ 45 giây xuống còn 0.5 giây.
- Đảm bảo 100% các thay đổi thuật toán điều khiển, lập quỹ đạo, hay hệ số PID đều được kiểm chứng tự động trước khi commit.
- Giao diện Raylib GUI (`just sim`) nay trở thành một tầng quan sát mỏng (thin visualization layer), tái sử dụng trực tiếp `SimEngine` mà không chứa logic vật lý trùng lặp.
