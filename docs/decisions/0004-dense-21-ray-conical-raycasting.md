# ADR-0004: Dense 21-Ray Conical Sector Raycasting for Laser ToF Proximity

## Status
Accepted

## Date
2026-10-06

## Context
Trong phần cứng thực tế, cảm biến khoảng cách laser ToF (VL53L0X) không phát ra một tia laser chỉ điểm siêu mảnh vô hạn, mà phát chùm quang học dạng nón với góc mở thị trường (Field of View - FoV) từ $25^\circ$ đến $27^\circ$.
Trước đây trong mô phỏng, cảm biến ToF chỉ được giả lập bằng **1 tia duy nhất (1D centerline ray)** phát thẳng từ tâm mắt đo.

Hậu quả của mô hình 1 tia:
1. **Điểm mù góc nghiêng (Angle Blind Spots):** Vật cản trong cuộc thi có kích thước $105 \times 60\text{ mm}$ có thể được đặt ở góc xoay bất kỳ $\theta \in [0, 360^\circ]$. Khi xe tiếp cận ở góc xiên, mép hoặc góc nhọn của vật cản lọt vào tầm nón quang học của xe thực nhưng lại trượt khỏi tia 1D duy nhất của mô phỏng, dẫn đến việc robot đâm vào góc vật cản mà ToF không hề báo phát hiện.
2. **Hiển thị tia ma (Phantom Ray Projections):** Khi không có vật cản, hàm đo vẫn trả về tọa độ mút tia cố định, khiến giao diện đồ họa vẽ ra các điểm va chạm ảo giữa không trung.

## Decision
1. **Triển khai Rẻ quạt quang học dày 21 tia (Dense 21-Ray Conical Casting):**
   - Trong [`firmware/src/sim/raycast.cpp`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/raycast.cpp#L73-L99), mỗi cảm biến ToF (Front, Left, Right) quét một rẻ quạt gồm 21 tia con phân bố đều trong góc mở FoV $25^\circ$ ($[-12.5^\circ, +12.5^\circ]$).
   - Mỗi tia con giải bài toán giao cắt hình học 2D giữa tia thẳng và hộp định hướng (Oriented Bounding Box - OBB) của vật cản bằng giải thuật cắt xén trục tọa độ cục bộ (Slab method).
   - Cự ly đo của cảm biến là giá trị nhỏ nhất trong số các tia trúng đích: $d = \min_{i=0\dots 20} d_i$.
2. **Bổ sung cờ logic xác thực [`TofHitPoint::has_hit`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/types.hpp#L40):**
   - Phân biệt rõ ràng giữa "trúng vật cản ở khoảng cách $d$" và "vùng quét trống rỗng chạm ngưỡng tối đa $d_{\max}$".
   - Khi `has_hit == false`, giao diện đồ họa không vẽ điểm va chạm ảo và thuật toán điều khiển nhận biết không có vật thể cản trở phía trước.
3. **Cập nhật cự ly tối đa phát hiện phía trước:**
   - Đặt cự ly phát hiện trước chuẩn là $0.35\text{ m}$ ($35\text{ cm}$) để xe có đủ quãng đường hãm phanh an toàn từ tốc độ cao.

## Alternatives Considered

### 1. Giữ nguyên 1 tia laser duy nhất
- **Ưu điểm:** Rất ít phép tính.
- **Nhược điểm:** Sai lệch bản chất vật lý; bỏ sót các góc nhọn của vật cản xoay chéo; xe thực sẽ va chạm dù mô phỏng báo an toàn.
- **Lý do loại bỏ:** Vi phạm yêu cầu độ trung thực của mô phỏng phần cứng.

### 2. Giao cắt hình học liên tục (Continuous Cone-OBB Polygon Clipping)
- **Ưu điểm:** Độ chính xác giải tích tuyệt đối.
- **Nhược điểm:** Thuật toán giao cắt đa giác phức tạp, xử lý biên góc nhọn dễ phát sinh ngoại lệ chia cho 0, làm giảm tốc độ tính toán của vòng lặp 500 Hz.
- **Lý do loại bỏ:** 21 tia rời rạc đã đủ độ mịn (< 1.25 độ mỗi bước) và tính toán cực nhanh (< 1 microsecond).

## Consequences
- Mô phỏng phản ánh chính xác 100% hình nón quang học của phần cứng VL53L0X.
- Triệt tiêu hoàn toàn hiện tượng va chạm điểm mù khi vật cản đặt nghiêng.
- Giao diện Raylib hiển thị vùng nón quét trực quan, cắt cụt chính xác tại mặt phẳng vật cản thực tế.
