# ADR-0005: First-Principles Continuous Kinodynamic Velocity Profiling & Differential Drive Counter-Rotation

## Status
Accepted

## Date
2026-10-08

## Context
Trong các bài toán đua robot vi sai tốc độ cao (differential drive), việc vào các khúc cua gắt (như góc vuông $90^\circ$ hoặc rãnh chữ U/chữ Y có bán kính cong $R \approx 40 - 60\text{ mm}$, độ cong $|\kappa| \approx 16 - 25\text{ rad/m}$) đặt ra hai thách thức động học nghiêm trọng:

1. **Vá víu ngưỡng vận tốc rời rạc (Piecewise Velocity Patching):**
   - Trước đây, hệ thống sử dụng một ngưỡng ngắt đoạn tùy tiện: `if (abs_k > 10.0f) v_limit = 0.40f;`.
   - Cách tiếp cận này tạo ra bước nhảy gián đoạn (discontinuity) trong hồ sơ vận tốc, không phản ánh quy luật vật lý liên tục của cơ cấu truyền động vi sai, và thất bại trong việc điều tiết vận tốc mượt mà theo độ cong biến thiên liên tục của sa bàn.
2. **Kẹt bán kính quay do cấm đảo chiều bánh (Differential Understeer & Spinout):**
   - Trong `StanleyTracker`, vận tốc bánh xe bị kẹp cận dưới bằng 0: `vl = std::max(0.0f, vl); vr = std::max(0.0f, vr);`.
   - Với khoảng cách cơ sở hai bánh $W = 115\text{ mm}$, khi bánh trong dừng hẳn ($v_{\text{inner}} = 0$), bán kính quay tức thời tối thiểu bị khóa cứng ở $R_{\min} = W / 2 = 57.5\text{ mm}$.
   - Khi gặp góc cua nhọn $R < 50\text{ mm}$, xe không thể đánh lái đủ gắt, dẫn đến hiện tượng thiếu lái (understeer) nghiêm trọng, bắn văng ra khỏi vạch sang làn đối diện và bị loại do phạm quy "cắt line" (Điều 5.5 / Điều 8 trong `docs/manual.md`).

## Decision

### 1. Giới hạn động học vi sai liên tục (Continuous Differential Kinodynamic Envelope)
Thay thế hoàn toàn câu lệnh `if-else` ngắt đoạn bằng phương trình giới hạn bão hòa liên tục dẫn xuất từ hình học động học vi sai:
- Khi xe di chuyển với vận tốc dài $v$ qua độ cong $\kappa$, vận tốc bánh ngoài là:
  $$v_{\text{outer}} = v \left(1 + \frac{1}{2} |\kappa| W\right)$$
- Để đảm bảo bánh ngoài không vượt ngưỡng bão hòa động cơ và mô-men xoay $\omega = v \kappa$ không vượt quá khả năng ổn định động học, giới hạn vận tốc vi sai liên tục được chuẩn hóa thành:
  $$v_{\text{diff\_limit}} = \frac{v_{\text{top}}}{1.0 + \left(\frac{v_{\text{top}}}{v_{\text{safe}}}\right) |\kappa| W}$$
- Vận tốc trần tại mỗi điểm quỹ đạo trong `firmware/lib/track/trajectory_planner.cpp` là giao của 3 giới hạn vật lý:
  $$v_{\text{limit}} = \min\left(v_{\text{top}},\, \sqrt{\frac{\mu g}{\max(|\kappa|, \epsilon)}},\, v_{\text{diff\_limit}}\right)$$
  - Trên đoạn thẳng ($\kappa \to 0$): $v_{\text{diff\_limit}} \to v_{\text{top}}$.
  - Tại đỉnh cua gắt ($|\kappa| \approx 16\text{ rad/m}$): $v_{\text{diff\_limit}}$ giảm mượt mà xuống $\le 0.37\text{ m/s}$ mà không có bất kỳ bước nhảy gián đoạn nào.

### 2. Kích hoạt đảo chiều bánh xe trong Stanley (Differential Counter-Rotation)
- Trong `firmware/lib/control/stanley_tracker.cpp`, gỡ bỏ hoàn toàn phép kẹp `std::max(0.0f, v)`.
- Thay thế bằng phép kẹp bão hòa động cơ đối xứng:
  $$v \in [-v_{\text{max\_wheel}}, +v_{\text{max\_wheel}}]$$
- Cho phép bánh trong chủ động quay lùi (counter-rotation) khi góc lái lớn ($\text{steer} > 1.0\text{ rad}$), cho phép robot xoay tại tâm trục (pivot turn) với bán kính $R \to 0$, ôm trọn các khúc rẽ $90^\circ$ mà không bao giờ văng khỏi hành lang đường đua.

## Alternatives Considered

### 1. Giữ nguyên ngưỡng `if (abs_k > 10.0f)` và thêm ngưỡng trung gian 5.0, 15.0
- **Bác bỏ:** Cách tư duy vá víu (patchwork), tạo ra vô số magic numbers và làm tăng độ phức tạp thuật toán mà không giải quyết được tính liên tục trơn tru $C^2$ của hồ sơ gia tốc.

### 2. Giảm vận tốc toàn bộ đường đua $V_{\text{top}}$ xuống thấp
- **Bác bỏ:** Làm mất lợi thế bứt phá tốc độ trên các đoạn thẳng của Lượt 2 Sprint, đi ngược lại mục tiêu tối ưu thời gian vòng chạy.

## Consequences
- Loại bỏ 100% mã phân nhánh `if-else` trong tính toán tốc độ góc cua.
- Quỹ đạo Run 2 tự động hãm tốc từ xa trên đoạn thẳng tiếp cận nhờ thuật toán Backward Pass của Velocity Profiler, vào cua an toàn tuyệt đối.
- Xe có khả năng quay góc vuông tại chỗ nhờ cơ chế counter-rotation đối xứng.
