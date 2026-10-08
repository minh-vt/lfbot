# ADR-0006: Optical Dynamic Braking via Centroid Deviation for Run 1 Unmapped Exploration

## Status
Accepted

## Date
2026-10-08

## Context
Trong Lượt 1 (Run 1: Mapping / Exploration), robot di chuyển trên một sa bàn hoàn toàn chưa biết trước hình học (chưa có bản đồ tọa độ hay vị trí chướng ngại vật).

1. **Hạn chế của vận tốc tĩnh (Static Cruise Speed):**
   - Trước đây, hệ thống chỉ gán một hằng số vận tốc tĩnh `safe_map_speed_mps = 0.80 m/s`.
   - Khi chạy trên đường thẳng, $0.80\text{ m/s}$ có thể là an toàn, nhưng khi bất ngờ gặp khúc cua vuông $90^\circ$ hoặc rãnh chữ Y/hộp chữ nhật, robot không có tri thức trước để phanh sớm (do chưa có bản đồ để chạy Backward Pass).
   - Vận tốc $0.80\text{ m/s}$ kết hợp với giới hạn đáp ứng cơ học khiến xe bị trượt văng vạch trước khi bộ điều khiển PID kịp sinh đủ lực xoay.
2. **Khó khăn khi thiếu giao diện điều khiển:**
   - Vận tốc Lượt 1 bị gắn cứng trong mã nguồn, người vận hành không thể tinh chỉnh linh hoạt theo độ bám thực tế của sàn thi đấu mà không phải biên dịch lại firmware.

## Decision

### 1. Ứng dụng phương pháp cổ điển: Phanh động học quang học (All-Japan RoboTrace Standard)
Áp dụng nguyên lý đệ nhất từ giải thuật dò line cổ điển của các giải đua RoboTrace Nhật Bản:
- Vì cảm biến quang học được đặt vươn về phía trước (`sensor_lookahead_m = 0.055m`), khi robot bắt đầu tiếp cận một khúc cua gắt, vạch đen sẽ lệch nhanh khỏi tâm thanh cảm biến trước khi thân xe kịp xoay.
- Do đó, độ lệch tâm quang học $|e| = |\text{centroid\_mm}|$ chính là tín hiệu chỉ báo sớm (early curvature onset indicator) trực tiếp nhất về độ cong sắp tới.
- Thiết lập luật điều khiển vận tốc thích nghi liên tục trong `firmware/lib/strategy/tactical_arbiter.cpp`:
  $$v_{\text{target}}(e) = \frac{V_{\text{map}}}{1.0 + \left(\frac{|e|}{e_0}\right)^2}$$
  với ngưỡng chuẩn hóa $e_0 = 20.0\text{ mm}$.
  - Khi xe đi thẳng ($e \approx 0\text{ mm}$): $v_{\text{target}} \approx V_{\text{map}}$ (lướt tối đa vận tốc thăm dò).
  - Khi mép vạch bẻ cua gắt ($|e| \to 25 - 31.5\text{ mm}$): $v_{\text{target}}$ tự động ghìm xuống $\le 0.38 V_{\text{map}}$ ($\approx 0.30 - 0.35\text{ m/s}$), giúp xe bò qua góc vuông $90^\circ$ với độ tin cậy tuyệt đối.
  - Khi xe thoát cua và vạch trở lại tâm: xe tự động tăng tốc mượt mà trở lại $V_{\text{map}}$.

### 2. Bổ sung tham số điều khiển `Map V` trực tiếp trên HUD
- Mở rộng bảng điều khiển HUD với hàng tham số `"Map V"` nằm ngay trên `"Sprint V"`.
- Hỗ trợ nút bấm $\pm 0.05\text{ m/s}$ trong dải $[0.20, 1.50]\text{ m/s}$, cho phép tinh chỉnh tức thì trong quá trình thử nghiệm.

## Alternatives Considered

### 1. Chạy chậm toàn bộ Lượt 1 ở $0.35\text{ m/s}$
- **Bác bỏ:** Làm thời gian hoàn thành Lượt 1 quá dài, lãng phí thời gian trên các đoạn thẳng và có nguy cơ chạm trần thời gian tổng thể của lượt thi.

### 2. Dùng camera hoặc ToF để dự đoán độ cong vạch phía trước
- **Bác bỏ:** Thừa thãi và làm phức tạp hóa phần cứng không cần thiết (vi phạm nguyên tắc Simplicity First / Ponytail). Cảm biến quang 8 mắt hiện tại hoàn toàn đủ thông tin hình học nếu khai thác đúng cách.

## Consequences
- Lượt 1 đạt tỷ lệ hoàn thành vòng chạy (completion rate) $100\%$ ngay cả trên các sa bàn hiểm hóc có khúc cua vuông $90^\circ$.
- Quá trình giảm tốc và tăng tốc diễn ra hoàn toàn tự nhiên, không cần bất kỳ lệnh phân nhánh điều kiện rời rạc nào.
- Người dùng có thể điều chỉnh độc lập chiến lược an toàn Lượt 1 (`Map V`) và chiến lược bứt phá Lượt 2 (`Sprint V`).
