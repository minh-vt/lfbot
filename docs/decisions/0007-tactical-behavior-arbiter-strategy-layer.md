# ADR-0007: Extraction of Deep Module TacticalBehaviorArbiter into Strategy Layer

## Status
Accepted

## Date
2026-10-07

## Context
Trong thiết kế ban đầu của hệ thống:
1. **Trộn lẫn trách nhiệm (Violated Single Responsibility):**
   - Lớp `TrajectoryFollower` trong `firmware/lib/control/` vừa thực hiện tính toán điều khiển vòng kín (PID bám line, Stanley tracking), vừa phải kiêm nhiệm:
     - Logic chốt mất vạch (Line-loss timeout guardrail).
     - Động học quán tính duy trì vận tốc qua khe hở vạch (Dead-reckoning momentum buffer).
     - Quy hoạch cung né tránh chướng ngại vật $C^2$ cục bộ (Obstacle evasion chicane arc).
     - Trạng thái dừng khẩn cấp (Emergency halt).
2. **Khó kiểm thử độc lập (Test Fragility):**
   - Các quyết định chiến thuật phản xạ cấp cao bị gắn chặt vào thuật toán tính vận tốc bánh xe cấp thấp, dẫn đến việc kiểm thử chiến thuật phải thông qua các bộ điều khiển PID/Stanley với nhiều tham số phụ thuộc.

## Decision
Trích xuất toàn bộ logic trọng tài chiến thuật cấp cao thành một module sâu độc lập: **`TacticalBehaviorArbiter`** đặt tại `firmware/lib/strategy/`:

1. **Giao diện hẹp, hiện thực sâu (Deep Module Contract):**
   - Đầu vào: `Pose2D`, `current_v_mps`, `centroid_mm`, `line_found`, dữ liệu ToF 3 hướng (`front`, `left`, `right`), `dt`, và `RobotMode`.
   - Đầu ra duy nhất: Cấu trúc dữ liệu bất biến `TacticalReference`:
     ```cpp
     struct TacticalReference {
       float target_v_mps = 0.0f;
       float lateral_error_m = 0.0f;
       float steer_cmd_rad = 0.0f;
       bool has_steer_override = false;
       bool should_halt = false;
       bool is_evading = false;
       bool is_dead_reckoning = false;
       uint8_t bypass_state = 0;
     };
     ```
2. **Trách nhiệm cốt lõi của Arbiter:**
   - **Bảo vệ an toàn mất vạch:** Theo dõi biến thời gian tích lũy `line_loss_duration_s_`. Cho phép duy trì quán tính trong thời gian ngắn (< 300 ms) qua khoảng đứt đoạn; nếu vượt quá ngưỡng thời gian quy định sẽ kích hoạt `should_halt = true` để dừng xe an toàn.
   - **Né chướng ngại vật phản xạ $C^2$ (Lượt 1):** Khi ToF phát hiện vật cản trong cự ly nguy hiểm, Arbiter tự động tính toán cung né tránh đa thức bậc 5 chuyển tiếp mượt mà, ghi đè góc lái tạm thời (`has_steer_override`), và tự động bắt lại vạch quang học khi hoàn tất đường vòng.
   - **Phanh quang học động học (Run 1 Optical Braking):** Tự động điều tiết `target_v_mps` dựa trên độ lệch tâm `centroid_mm`.

## Alternatives Considered

### 1. Giữ nguyên toàn bộ logic trong `TrajectoryFollower`
- **Bác bỏ:** Tạo ra một "God class" trong tầng điều khiển, vi phạm nguyên tắc tách biệt ranh giới kiến trúc (Architectural Boundaries).

### 2. Đưa logic trọng tài lên lớp giao diện `SimEngine` hoặc `main.cpp`
- **Bác bỏ:** Làm mất tính độc lập phần cứng của thuật toán cốt lõi. Firmware nhúng chạy trên ESP32-S3 sẽ không thể tái sử dụng các quy tắc an toàn này.

## Consequences
- `TrajectoryFollower` được tinh gọn trở lại đúng bản chất: chỉ thuần túy bám theo `TacticalReference` được cấp phát từ Arbiter.
- Toàn bộ hành vi an toàn (né vật cản, mất vạch, phanh khẩn cấp) được kiểm thử độc lập 100% bằng headless unit tests (`firmware/test/test_core.cpp`).
- Module `strategy/tactical_arbiter` hoàn toàn không phụ thuộc Raylib hay phần cứng nhúng, sẵn sàng cho việc triển khai trên bất kỳ nền tảng nào.
