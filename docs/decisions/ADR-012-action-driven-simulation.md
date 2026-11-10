# ADR-012: Simulator nhận motor output, core phẳng và preprocessing dùng chung

**Status:** Accepted; phần frontend native CMake được thay bởi [ADR-013](ADR-013-platformio-only.md). Accepted ngày 2026-10-19 theo yêu cầu chủ dự án triển khai thiết kế này. Thay một phần [ADR-009](ADR-009-scoped-robot-native-runtime.md)/[ADR-010](ADR-010-preserve-ui-and-name-sensor-contracts.md) về phạm vi nêu dưới đây; các ADR đó vẫn giữ status Accepted và lịch sử. Việc Accepted không có nghĩa mọi mục trong [ARCHITECTURE](../ARCHITECTURE.md) đã được đo trên phần cứng hoặc đã chọn RL thay controller.

## Context

`Simulation::advance(Robot&, RaceRequest)` vừa gọi controller vừa tiến vật lý, giữ controller telemetry/trace và quyết định lifecycle. Runtime link PUBLIC `lf_robot`, nên không thể dùng engine để chạy action trực tiếp mà không kéo controller hiện tại vào. Chuyển callback thành interface ảo không loại bỏ trách nhiệm gọi controller của engine.

Một môi trường RL C++ cần bước theo action, state/next_state độc lập, cùng acquisition/normalization/output semantics với robot. Hiện conversion/validation nằm trong Robot implementation; copy sang trainer sẽ tạo hai định nghĩa của đơn vị, freshness, encoder wrap và deadline. Packaging ba package với manifests/include/src riêng không giúp giải quyết các vấn đề này.

## Decision đề xuất

- Engine `Simulation` chỉ nhận `MotorOutput`, sinh physical sensor packets và chấm thế giới. Không include/call/link Robot, Race, Raylib hoặc RLtools. Data-only header là dependency hợp lệ, không cần engine độc lập cả schema motor/sensor.
- Một package `firmware/lib/robot/` phẳng: `robot_data`, `drive`, `robot`, `race`. Header data không include controller. `Drive` gom conversion/output validation và history cần thiết, được Robot và environment native dùng chung. Không dựng Robot private storage trong trainer hoặc platform.
- Giữ scoped typed ownership `run_robot`/`RobotSession` của đường chạy robot. Race giữ quyết định chuyển động và memory; không thêm controller hierarchy, registry, `void*` context hoặc runtime plugin.
- `RaceTrial` ở native composition ghép Robot + Simulation cho GUI/kiểm thử hai lượt, giữ accounting và tạo frame quan sát; không là manager tổng quát. Engine không giữ RobotObservation hoặc trace dài; renderer chỉ đọc.
- Simulation/Drive có native value state độc lập để adapter RLtools tôn trọng `step(state, action, next_state)`. Assets được borrow const với lifetime rõ; không copy raster/trace trong mỗi transition, không giấu mutable world singleton trong environment.
- Lựa chọn frontend native CMake ban đầu đã bị thay bởi [ADR-013](ADR-013-platformio-only.md); giữ `just`, ESP32/ESP-IDF deferred và [ADR-011](ADR-011-remove-custom-architecture-checker.md).
- Controller hình học hoặc learned policy là implementation cần đánh giá, không buộc hoàn thiện controller cũ trước thử nghiệm RL. Chưa chốt model/algorithm/precision; không đưa truth/reward vào deployed actor. Safety/freshness/deadline/watchdog độc lập với reward.

Interface semantics, folder tree, clock/reset/two-run contracts và migration acceptance nằm duy nhất tại [ARCHITECTURE](../ARCHITECTURE.md); backlog/regressions tại [PLAN](../PLAN.md). Không tạo thêm spec trùng nội dung.

## Alternatives

1. **Giữ Simulation gọi Robot hoặc callback controller:** ít đổi caller trước mắt nhưng engine vẫn điều phối control loop, khó thử action/replay và giữ coupling lifecycle. Không chọn.
2. **`IController`/template framework với nhiều backend:** có thể thay controller nhưng không cần thiết cho engine có input motor cụ thể; tăng interface, cấu hình và ownership complexity. Không chọn.
3. **Data-driven engine + composition ngoài:** chọn; có thêm Drive/RaceTrial tại seams có callers thật, đổi lại engine độc lập và conversion chỉ có một implementation. Chỉ tạo phần training/model khi có việc dùng thực.

## Consequences và phạm vi thay thế

Khi được duyệt, thay quyết định chỉ `simulation.cpp` được gọi Robot, package/module metadata riêng, controller hình học bắt buộc và vị trí schema của ADR-009/010. Giữ scoped Robot ownership, physical/normalized distinction, per-channel evidence, hai lượt, giới hạn thời gian/bộ nhớ, UI preservation và ESP32 deferred. Không khôi phục checker hoặc đổi OS.

Đổi interface đòi chuyển GUI/headless cùng orchestration, xử lý counter rebasing và request prepare/end-run rõ ràng; không chỉ rename/move file. Method lifetime, fault recovery, thresholds freshness và các luật chưa rõ vẫn phải được chốt/test; engine tách sạch không tự chữa DNF hay chứng minh sim-to-real.

Slice triển khai đầu tiên đã làm: core phẳng + `Drive`, engine theo motor output + `Referee`/`RaceTrial`, frontend build ban đầu là CMake và sau đó được ADR-013 thay bằng PlatformIO CLI, regression cho collision/finish/stale/units/lifecycle. Chưa làm: environment training, policy, chẩn đoán DNF, native GUI automation, ESP32/HIL. Research RLtools chỉ xác nhận một số đường C++ training/export ngoài repo, chưa là acceptance LF-Bot/ESP32.
