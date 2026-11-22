# Quyet dinh kien truc

ADRs ghi boi canh, lua chon va danh doi; [ARCHITECTURE.md](../ARCHITECTURE.md) mo ta thiet ke hien hanh. Accepted nghia la chap nhan quyet dinh, khong co nghia da trien khai hay kiem chung tren phan cung.

| ADR | Trang thai | Pham vi |
|---|---|---|
| [004](ADR-004-esp32-toolchain.md) | Accepted | Chọn ESP-IDF; chi tiết triển khai cũ không phải bằng chứng timing |
| [006](ADR-006-naming-clean-code.md) | Accepted | Tên theo việc robot làm; quy ước hiện hành ở NAMING.md |
| [009](ADR-009-scoped-robot-native-runtime.md) | Accepted | Scoped typed owner, contract không thứ nguyên |
| [010](ADR-010-preserve-ui-and-name-sensor-contracts.md) | Accepted | Contract sensor/motor tường minh, dispatch trực tiếp bằng just |
| [011](ADR-011-remove-custom-architecture-checker.md) | Accepted | Bỏ checker kiến trúc tự viết; giữ build/lint/test hành vi |
| [013](ADR-013-platformio-only.md) | Accepted | PlatformIO CLI là frontend build duy nhất cho native và ESP32 |
| [015](ADR-015-build-and-hardware-verification.md) | Accepted | Không hook build riêng; phân biệt CAD / tính toán / đo trên bàn |
| [019](ADR-019-module-based-hardware.md) | Accepted; "dừng PCB" thay bởi 025 cho board base | Dừng MAIN/IR-03; IR analog 8 mắt, hai DRV8871, pin 3S, pin map module |
| [020](ADR-020-esp32-firmware-restart.md) | Accepted | Viết lại firmware ESP32, bỏ simulator và core cũ, hiệu chuẩn tại sân |
| [021](ADR-021-field-adaptive-two-run-design.md) | Accepted một phần; cử chỉ, đích, profile sân thay bởi 023 | Một nút Start/Hủy, đích 8 mắt trắng có xác nhận, tự hiệu chuẩn tại sân |
| [022](ADR-022-control-strategy-and-codebase.md) | Superseded bởi 023 | Điều khiển ba tầng tự suy hệ số, tốc độ tự chọn, UX nút + LED GPIO48 |
| [023](ADR-023-sensor-driven-two-run-firmware.md) | Accepted; phần điều khiển thay bởi 024; chưa triển khai | Dùng đủ cảm biến, sự thật lắp ráp là hằng số, gain chỉnh trên đường thử, một nút, nghiệm thu bằng xe chạy |
| [024](ADR-024-dimensionless-control.md) | Accepted; chưa triển khai | Điều khiển không thứ nguyên; lượt 2 nhìn trước theo công sức lái đã ghi ở lượt 1 |
| [025](ADR-025-pcb-base-carrying-modules.md) | Accepted; đã route, chưa đặt in | PCB làm khung xe mang module; placement/route bằng KiCadRoutingTools; GPIO gán lại theo board |

Số thứ tự đã dùng được giữ nguyên; các số 001, 007, 008, 012, 014, 016, 017, 018 thuộc về thiết kế đã bị xoá (core cũ, simulator, các vòng PCB riêng) và đã bị gỡ khỏi tài liệu theo yêu cầu chủ dự án. Quyết định thay thế chúng nằm trong ADR-020/021/023 và ADR-019.

Giu so thu tu da dung; khong danh so lai de lap cac khoang trong. ADR cu duoc giu lam lich su, danh dau Superseded khi co quyet dinh thay the. Chi tao ADR cho lua chon co danh doi dang ghi lai; khong tao mot ADR cho moi ham hay file.
