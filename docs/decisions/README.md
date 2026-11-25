# Quyết định kiến trúc

ADR ghi bối cảnh, lựa chọn và đánh đổi; [ARCHITECTURE.md](../ARCHITECTURE.md) mô tả thiết kế hiện hành. Accepted nghĩa là chấp nhận quyết định, không có nghĩa đã triển khai hay kiểm chứng trên phần cứng.

| ADR | Trạng thái | Phạm vi |
|---|---|---|
| [013](ADR-013-platformio-only.md) | Accepted | PlatformIO CLI là frontend build duy nhất cho ESP32 và test host |
| [019](ADR-019-module-based-hardware.md) | Accepted | Module mua sẵn: IR analog 8 mắt, hai DRV8871, pin 3S, motor 12 V |
| [020](ADR-020-core-and-esp32.md) | Accepted | Core độc lập phần cứng trong `firmware/core`, driver trong `firmware/esp32` |
| [023](ADR-023-sensor-driven-two-run-firmware.md) | Accepted | Dùng đủ cảm biến, sự thật lắp ráp là hằng số, một nút, nghiệm thu bằng xe chạy |
| [024](ADR-024-dimensionless-control.md) | Accepted; host test, chưa chạy trên xe | Điều khiển không thứ nguyên; lượt 2 nhìn trước theo công sức lái đã ghi ở lượt 1 |
| [025](ADR-025-pcb-base-carrying-modules.md) | Accepted; board 1 mặt chưa làm | PCB làm khung xe mang module; board xe là bản 1 mặt làm tại nhà |
| [026](ADR-026-one-follower-virtual-line.md) | Accepted; host test, chưa chạy trên xe | Một bộ bám line chọn `e` từ line/nhớ/dò/thẳng/line ảo; né gạch bằng line ảo |

Chỉ giữ quyết định còn hiệu lực. Khi một quyết định bị thay hoàn toàn, xoá ADR cũ và sửa các tham chiếu; git giữ lịch sử. Khi chỉ một phần bị thay, sửa ADR đó cho đúng hiện tại. Số đã dùng không đánh lại. Chỉ tạo ADR cho lựa chọn có đánh đổi đáng ghi lại; không tạo một ADR cho mỗi hàm hay file.
