# ADR-0002: Independent RaceArbiter Ground-Truth Verification

## Status
Accepted

## Date
2026-10-06

## Context
Trong môi trường kiểm thử mô phỏng, trước đây hệ thống phụ thuộc hoàn toàn vào cờ logic nội bộ của robot (`RobotCoordinator::check_finish_condition`) để xác định robot đã về đích hay chưa (dựa trên việc cả 8 mắt quang học nhìn thấy vạch đen).

Phương thức tự đánh giá này bộc lộ lỗ hổng chí mạng (Confirmation Bias / Blind Spot):
1. **Ảo tưởng về sự hoàn hảo:** Nếu robot bị trượt văng khỏi cua, quay đầu 180 độ và chạy lùi lại vạch xuất phát, hoặc cắt ngang một đoạn đường khác của sa bàn, robot vẫn có thể kích hoạt cờ về đích và tự báo cáo thành công.
2. **Không giám sát luật thi đấu (The Rules of Racing):** Thể lệ cuộc thi F-Maze-Solving (Điều 5.5) quy định hủy lượt chạy nếu rời vạch quá 5.0 giây, chạm vào vật cản, hoặc không dừng gọn trong ô đích $160 \times 160\text{ mm}$. Hệ thống điều khiển bên trong robot không có góc nhìn toàn cục (God's eye view) để tự phạt chính mình.
3. **Phụ thuộc vào mắt người ("Biological Smoke-Tester"):** Không có bộ tiêu chí tự động dẫn đến việc lập trình viên phải mở giao diện đồ họa, căng mắt nhìn robot chạy 30-50 giây và phán đoán cảm tính.

## Decision
1. **Tách biệt tuyệt đối giữa Robot và Trọng tài:**
   - Xây dựng module độc lập [`RaceArbiter`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/race_arbiter.hpp#L74) đại diện cho tổ trọng tài điện tử đặt bên ngoài sa bàn.
   - Trọng tài chỉ nhận dữ liệu sự thật vật lý thuần túy qua cấu trúc [`ArbiterGroundTruth`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/race_arbiter.hpp#L50) (tọa độ $x, y$, hướng xoay, vận tốc thực, cự ly thực tới vật cản, cảm biến quang có chạm vạch hay không). Trọng tài hoàn toàn không truy cập hay phụ thuộc vào biến nội bộ của vi điều khiển robot.
2. **Cơ chế Cổng giám sát cung đường có hướng (Directed Sector Gates):**
   - Thiết lập các cổng kiểm soát thứ tự [`SectorGate`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/race_arbiter.hpp#L30) dọc theo sa bàn.
   - Xe bắt buộc phải vượt qua các cổng theo đúng thứ tự và đúng chiều tiến của vector pháp tuyến $(\vec{v} \cdot \mathbf{n} > 0)$. Nếu xe đi lùi hoặc đi tắt, trọng tài lập tức truất quyền thi đấu với mã lỗi `kWrongWay` hoặc `kGateSkipped`.
3. **Giám sát thời gian rời vạch nghiêm ngặt:**
   - Bộ đếm thời gian thực `continuous_line_lost_timer_s_` tích lũy liên tục khi xe không đè lên vạch. Nếu vượt quá $5.0\text{ giây}$, trọng tài xử thua ngay lập tức với mã `kLineLostTimeout`.
4. **Ô dừng đích chuẩn thể lệ ([`FinishBoxGeometry`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/race_arbiter.hpp#L39)):**
   - Vòng đua chỉ được công nhận hợp lệ (`kCompleted`) khi thân xe nằm trọn trong ô đích $160 \times 160\text{ mm}$ (tại tọa độ $x \in [320, 480]$, $y \in [1220, 1380]$) và vận tốc xe đã phanh dừng hoàn toàn ($v \le 0.05\text{ m/s}$).
5. **Chứng chỉ kết quả bất biến ([`LapVerdict`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/race_arbiter.hpp#L62)):**
   - Trọng tài xuất ra kết quả đóng gói không thể sửa đổi gồm trạng thái (`RaceStatus`), thời gian chính thức, số cổng đã qua, và chi tiết vi phạm nếu có.

## Alternatives Considered

### 1. Để `RobotCoordinator` tự báo cáo trạng thái hoàn thành
- **Ưu điểm:** Tiết kiệm một lớp kiến trúc.
- **Nhược điểm:** Xung đột lợi ích (conflict of interest); robot không thể nhận biết các lỗi vi phạm sa bàn toàn cục; tạo kẽ hở cho các lỗi logic đánh lừa bộ test.
- **Lý do loại bỏ:** Vi phạm nguyên tắc kiểm thử khách quan và che giấu lỗi điều khiển nghiêm trọng.

### 2. Phân tích log vết di chuyển sau khi chạy (Post-run Log Analyzer)
- **Ưu điểm:** Đơn giản khi viết script ngoài.
- **Nhược điểm:** Không thể dừng mô phỏng ngay khi xảy ra va chạm hoặc phạm quy; lãng phí chu kỳ tính toán trong CI; khó tích hợp sâu vào test suite C++ native.
- **Lý do loại bỏ:** Chậm hơn và kém trực quan hơn so với kiểm soát real-time qua Arbiter.

## Consequences
- Hệ thống có một "nguồn chân lý duy nhất" (SSoT) khách quan để xác định robot chạy đúng hay sai.
- Cho phép xây dựng bộ kiểm thử tự động Headless bắt lỗi 100% các trường hợp trượt line, chạy lùi, đâm vật cản.
- Cung cấp dữ liệu định lượng chính xác (thời gian vòng đua, cự ly né tối thiểu, thời gian rời vạch) thay vì cảm tính.
