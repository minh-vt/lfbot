# ADR-021: Vận hành một nút, đích bằng 8 mắt trắng, tự hiệu chuẩn tại sân

**Status:** Accepted, chủ dự án duyệt ngày 2026-11-10. Bổ sung [ADR-020](ADR-020-esp32-firmware-restart.md). Thay mô tả “nút Start/Stop” và mạch chia áp 180k/22k trong [ADR-019](ADR-019-module-based-hardware.md)/wiring. Chưa triển khai, chưa kiểm chứng trên board. Phần phản hồi (âm thanh, LED GPIO39) và cử chỉ nút được thay bởi [ADR-022](ADR-022-control-strategy-and-codebase.md); cử chỉ nút, cách nhận đích và profile sân do người giữ nút đo được thay tiếp bởi [ADR-023](ADR-023-sensor-driven-two-run-firmware.md). Phần đo pin bằng module voltage sensor vào GPIO10 được thay bởi [ADR-025](ADR-025-pcb-base-carrying-modules.md): chia áp 47k/10k 1% trên PCB base, vào GPIO1.

## Bối cảnh

Audit sau ADR-020 cho thấy: hiệu chuẩn qua USB cần laptop và thao tác tay, không dùng được ngày thi (Điều 4.7 thu xe, cấm điều chỉnh; sân ngoài trời Q-16). Hiệu chuẩn chỉ tĩnh, không thích nghi khi chạy. Map chỉ trong RAM. Không có phản hồi trạng thái. `Board` gộp mọi thiết bị và chặn I2C trong vòng 2 ms.

Chủ dự án quyết định: nút là Start; 8 mắt toàn trắng là đích; đã mua module voltage sensor.

## Quyết định

- **Một nút:** nhấn ngắn = Start khi nhả tay; nhấn giữ = tự hiệu chuẩn; giữ rất lâu = xóa map; nhấn khi đang chạy = **Hủy** (dừng, không chốt map hoàn chỉnh).
- **Đích:** 8 mắt trắng chỉ là ứng viên khi đang bám line, line mất ở giữa thanh, đi thẳng, không vật cản. Xác nhận bằng đi thêm một đoạn suy từ hình học xe rồi quét có giới hạn. Lượt 2 thêm điều kiện vị trí trên map. Lý do: báo sớm là DNF, báo muộn gần như vô hại vì cổng BTC đã dừng giờ.
- **Ba lớp tham số:** profile robot (đo ở nhà), profile sân (robot tự đo khi nhấn giữ), thích nghi khi chạy có biên.
- **Map lưu flash** khi đứng yên. Lượt kế tiếp do dữ liệu quyết định. Đầu lượt 2 phải khớp map, không khớp thì chạy như lượt 1.
- **Phản hồi:** thử motor phát âm trước; LED GPIO39 là phương án dự phòng.
- **Đo pin:** module voltage sensor vào GPIO10, tỉ lệ đo bằng đồng hồ vạn năng.
- **Nhịp:** Control 500 Hz không chặn; I2C task riêng; supervisor độc lập ép dừng khi lệnh quá hạn.

Chi tiết duy nhất ở [ARCHITECTURE](../ARCHITECTURE.md).

## Phương án bị loại

- **8 mắt trắng = đích ngay lập tức:** mất line ở cua, khi vòng gạch hoặc khi bị nhấc sẽ thành DNF.
- **Nút chỉ là Start:** không có cách dừng tay khi thử hoặc khi xe lao về phía người.
- **Hiệu chuẩn qua laptop ngày thi:** phụ thuộc thiết bị, dễ sai thao tác, có rủi ro với Điều 4.7.

## Hệ quả

- Actuator an toàn phải có trước tự hiệu chuẩn bằng chuyển động.
- Phải thử ngược bộ nhận đích: cua gắt, góc 90°, vòng gạch, nhấc xe, nắng chiếu ngang.
- Code hiện tại (`Board`, `Calibrator`, `service()`) sẽ được tách theo [PLAN](../PLAN.md); lệnh USB chỉ còn cho chẩn đoán.
- Việc tự hiệu chuẩn trên sân thi có được coi là “điều chỉnh” hay không chưa rõ. Thiết kế giảm rủi ro bằng cách chỉ dùng nút và robot, nhưng không khẳng định thay BTC.
