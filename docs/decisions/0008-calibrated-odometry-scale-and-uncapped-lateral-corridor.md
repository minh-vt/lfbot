# ADR-0008: Calibrated Odometry Scale & Uncapped Lateral Corridor Optimization

## Status
Accepted

## Date
2026-10-08

## Context
Trong quá trình vận hành và kiểm thử thực tế, hai vấn đề kỹ thuật quan trọng liên quan đến tham số hệ thống đã được phát hiện:

1. **Giả định suy đoán về thông số cơ khí của bộ đo xung (Encoder Speculation):**
   - Trước đây trong `KinematicsConfig`, đường kính bánh xe được giả định tĩnh `wheel_diameter_m = 0.034f` và số xung mỗi vòng `ticks_per_revolution = 560.0f`.
   - Trong thực tế chế tạo robot, đường kính bánh xe bị biến dạng nhẹ theo tải trọng và độ mòn cao su, còn tỷ số truyền động cơ N20 thường có sai số cơ khí. Việc tính toán vận tốc dựa trên hai thông số suy đoán rời rạc tạo ra sai số đo đạc tích lũy và gây khó khăn khi hiệu chuẩn thực tế.
2. **Khóa cứng hành lang lệch ngang (Lateral Corridor Choking):**
   - Trong `RobotCoordinator::set_optimizer_params`, tham số `max_lateral_deviation_mm` bị kẹp cứng trong khoảng $[5.0, 30.0]\text{ mm}$.
   - Trên sa bàn kích thước lớn $2400 \times 1600\text{ mm}$, độ lệch ngang tối đa $30\text{ mm}$ chỉ tương đương 2-3 pixel trên màn hình hiển thị.
   - Giới hạn $30\text{ mm}$ khóa chặt bộ giải QP tối ưu hóa quỹ đạo (`GlobalTrajectoryOptimizer`), khiến đường chạy tối ưu gần như trùng khít với đường vạch ban đầu, không tạo ra được đường đua cắt góc (Apex Cutting / Late Apex) rõ rệt để tăng tốc độ thi đấu.
   - Thêm vào đó, nhãn tham số trên HUD bị đặt tên nhầm là `"Corridor"`, gây hiểu lầm với độ rộng băng dính vạch.

## Decision

### 1. Chuẩn hóa Hệ số tỷ lệ Quãng đường trên mỗi Xung (Calibrated `meters_per_tick_scale`)
- Trong `firmware/lib/config/robot_config.hpp`:
  - Thay thế hai trường suy đoán bằng một trường đo lường duy nhất: `meters_per_tick_scale = 0.00019074f` (giá trị danh định $(0.034\pi) / 560$).
  - Bổ sung phương thức hiệu chuẩn thực nghiệm:
    ```cpp
    void calibrate(float distance_m, int32_t ticks) {
      if (distance_m > 0.0f && ticks > 0) {
        meters_per_tick_scale = distance_m / static_cast<float>(ticks);
      }
    }
    ```
  - Giữ lại phương thức `meters_per_tick()` để tương thích ngược 100% với các phép tính động học vi sai trong `Odometry` và `Kinematics`.

### 2. Mở rộng Hành lang Tối ưu hóa Lệch ngang lên $120\text{ mm}$ & Hiển thị `Max Dev`
- Trong `firmware/lib/system/robot_coordinator.cpp`:
  - Mở rộng dải kẹp của `max_lateral_deviation_mm` từ $[5.0, 30.0]\text{ mm}$ lên $[0.0, 120.0]\text{ mm}$, với giá trị mặc định được nâng lên $80.0\text{ mm}$.
  - Cho phép bộ giải QP tự do nắn quỹ đạo tối ưu, ôm cua góc trễ (Late Apex) rõ nét và lướt mượt mà qua các đoạn cong chữ S.
- Trong giao diện HUD (`firmware/src/sim/hud.cpp` & `main.cpp`):
  - Đổi nhãn hiển thị thành `"Max Dev"` (đơn vị mm).
  - Tăng bước nhảy tinh chỉnh lên $\pm 5.0\text{ mm}$ cho mỗi lần bấm phím $+ / -$.

### 3. Dọn dẹp Tham số Rác (Deadcode & Magic Number Purge - /ponytail-review)
- Loại bỏ hoàn toàn các trường tham số không sử dụng trong `firmware/lib/config/robot_config.hpp`:
  - `cross_marker_threshold`, `cross_marker_min_eyes`, `optical_confidence_k` (tàn dư từ logic nhận diện ngã tư cũ).
  - `repulsion_gain`, `side_tof_repulsion_gain` (tàn dư từ trường thế đẩy cũ).
  - `inertial_decay_lambda` (tàn dư từ bộ lọc trễ cũ).

## Alternatives Considered

### 1. Giữ nguyên công thức tính vận tốc theo đường kính bánh và số xung
- **Bác bỏ:** Yêu cầu người dùng phải tự đo đường kính lốp với độ chính xác đến $0.1\text{ mm}$ thay vì cho xe chạy thử 1 mét thực tế để lấy tỷ lệ chuẩn.

### 2. Tiếp tục giới hạn độ lệch ngang ở $30\text{ mm}$
- **Bác bỏ:** Bộ tối ưu hóa QP trở nên vô nghĩa vì không có không gian hình học để giảm độ cong Menger $\kappa$.

## Consequences
- Hệ thống đo đạc động học độc lập với giả định phần cứng, có thể hiệu chuẩn dễ dàng trên robot thực tế thông qua một lần chạy mẫu.
- Quỹ đạo Run 2 trên giao diện mô phỏng thể hiện rõ rệt đường đua tối ưu (racing line) cắt góc dứt khoát.
- Mã nguồn cấu hình sạch sẽ, không còn biến rác và magic numbers.
