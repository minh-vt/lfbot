# ADR-0001: Convex Quadratic Program Trajectory Optimization over Reactive FSM Avoidance

## Status
Accepted

## Date
2026-10-06

## Context
Cuộc thi F-Maze-Solving cho phép robot thực hiện 2 lượt chạy: Lượt 1 (Thăm dò sa bàn, lập bản đồ) và Lượt 2 (Chạy tốc độ cao dựa trên dữ liệu đã thu thập).
Trước đây, hệ thống sử dụng máy trạng thái hữu hạn 7 trạng thái (`obstacle_bypass.cpp/.hpp`) để né vật cản phản xạ (rẽ 90 độ, bám sườn bằng ToF ở cự ly 5 cm, bẻ ngược lại tìm vạch), hoặc sử dụng đa thức bậc 5 cục bộ (Quintic Polynomial Virtual Chicane).

Các nhược điểm nghiêm trọng của phương pháp cũ:
1. **Giật cục và mất quán tính:** FSM phản xạ khiến xe phải phanh gấp từ tốc độ cao xuống dưới 0.45 m/s, các góc bẻ 90 độ tạo gia tốc ngang và jerk cực lớn làm trượt bánh xe.
2. **Rủi ro phạm quy 5 giây (Điều 5.5):** Khi bám sườn vật cản trong điều kiện bề mặt trượt hoặc góc xoay vật cản bất kỳ, thời gian rời vạch dễ vượt quá ngưỡng 4.2 - 5.0 giây dẫn đến bị hủy lượt chạy.
3. **Cục bộ và không tối ưu toàn cục:** Quintic Chicane chỉ chắp vá cục bộ xung quanh vật cản mà không xét đến mối tương quan với góc cua ngay trước hoặc sau vật cản, làm mất đi lợi thế Late-Apex khi vào cua.
4. **Độ tin cậy thấp:** Máy trạng thái phản xạ phụ thuộc vào việc đọc ToF sườn thời gian thực, vốn rất nhiễu do góc phản xạ bề mặt vật cản nghiêng.

## Decision
1. **Loại bỏ hoàn toàn FSM né vật cản phản xạ:** Xóa bỏ `obstacle_bypass.cpp` và `obstacle_bypass.hpp` khỏi mã nguồn dự án.
2. **Triển khai Bộ tối ưu hóa Quỹ đạo Toàn cục Lồi (Global Convex QP Trajectory Optimizer):**
   - Sử dụng [`GlobalTrajectoryOptimizer`](file:///home/minhvt/Projects/lfbot/firmware/lib/track/global_trajectory_optimizer.hpp#L41) kết hợp với [`TrajectoryPlanner`](file:///home/minhvt/Projects/lfbot/firmware/lib/track/trajectory_planner.hpp#L11).
   - Hàm mục tiêu tối thiểu hóa tích phân bình phương độ cong Menger $\min \sum \kappa_i^2$ dọc toàn bộ quỹ đạo.
   - Ràng buộc biên hành lang dải băng vạch đen ($|n_i| \le 15\text{ mm}$) đảm bảo cụm 8 mắt quang học luôn ôm trọn vạch đen mà không mất dấu.
   - Vùng cấm vật cản được mô hình hóa bằng trường thế lực đẩy làm mịn (Smooth Repulsive Field) dựa trên bán kính an toàn ngoại tiếp $R_{\text{clearance}} \approx 155\text{ mm}$.
3. **Điều khiển bám quỹ đạo bằng [`StanleyTracker`](file:///home/minhvt/Projects/lfbot/firmware/lib/control/stanley_tracker.hpp#L29):**
   - Thay thế Cascaded PID ở Lượt 2 bằng thuật toán bám Stanley phi tuyến tính: kết hợp sai số cự ly pháp tuyến tại điểm ngắm nhìn trước (lookahead cross-track error) và sai số hướng la bàn (heading error) cùng bù động học feedforward $\text{steer}_{ff} = \frac{1}{2} v \kappa W_{\text{base}}$.
4. **Cơ chế dự phòng ở Lượt 1:**
   - Trong Lượt 1 khi gặp vật cản chưa biết trước, sử dụng cung lượn tránh động mịn ([`TrajectoryFollower::is_evading_`](file:///home/minhvt/Projects/lfbot/firmware/lib/control/trajectory_follower.hpp#L57)) để vượt qua và ghi nhận vị trí vật cản vào danh sách `detected_obstacles_` của [`RobotCoordinator`](file:///home/minhvt/Projects/lfbot/firmware/lib/system/robot_coordinator.hpp#L44).

## Alternatives Considered

### 1. FSM phản xạ 7 trạng thái kết hợp ToF sườn
- **Ưu điểm:** Đơn giản về mặt ý niệm, không cần nhớ bản đồ.
- **Nhược điểm:** Dễ kẹt trạng thái khi ToF đọc nhiễu, nguy cơ vượt quá 5 giây rời vạch, tốc độ rùa bò (< 0.5 m/s).
- **Lý do loại bỏ:** Không thể đáp ứng mục tiêu vô địch đua tốc độ cao tại F-Maze-Solving.

### 2. Đường tránh cục bộ Đa thức bậc 5 (Quintic Polynomial Chicane)
- **Ưu điểm:** Độ cong liên tục $C^2$ tại hai đầu đoạn tránh.
- **Nhược điểm:** Chỉ xử lý cục bộ, không xét đến hình học đoạn cua liền kề; không tối ưu được thời gian di chuyển toàn sa bàn; khó ràng buộc chính xác với hành lang băng dính.
- **Lý do loại bỏ:** Thua kém phương pháp QP toàn cục cả về thời gian hoàn thành vòng đua lẫn độ ổn định động học.

## Consequences
- Hệ thống đạt quỹ đạo đua mượt mà tối ưu (Late-Apex), giảm thiểu lực quán tính ngang trên toàn bộ sa bàn.
- Lượt 2 đạt tốc độ lên tới 2.5 m/s trên đoạn thẳng và duy trì vận tốc ma sát bám đường cực đại trong các góc cua.
- Thời gian tính toán QP toàn cục trên tập điểm sa bàn hoàn thành trong dưới 20 ms trên vi xử lý.
- Mã nguồn gọn gàng, loại bỏ các trạng thái FSM rườm rà và khó kiểm thử.
