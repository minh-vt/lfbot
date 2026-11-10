# ADR-018: MAIN nhận placement V2 và kiểm chứng đường khó trước

> **Superseded by [ADR-019](ADR-019-module-based-hardware.md)** cho phần cứng hiện hành: dừng PCB riêng, chuyển sang module. Các status và quyết định bên dưới là lịch sử, không phải quyền tiếp tục chế tạo.

**Status:** Accepted theo yêu cầu chủ dự án duyệt candidate V2 và cập nhật MAIN.
Thay baseline placement của [ADR-017](ADR-017-main-connectivity-placement.md);
giữ pinout, BOM, footprint, keepout và envelope/process của ADR-014/016.

## Bối cảnh

V1 giảm MST crossings nhưng gom cụm quá chặt, reference lên xuống và chưa
chứng minh routing dễ hơn. Chủ dự án đã duyệt phương án V2 sau so sánh cùng
tỉ lệ và xem cả chỉ số cải thiện lẫn đánh đổi. Đây không phải phê duyệt gia công.

## Quyết định

- Nhận đúng candidate V2 vào `hardware/main/main.kicad_pcb`; không dùng lại
  auto-placer hoặc dời thêm linh kiện trong ticket routing. Tài liệu placement
  đã archive cùng PCB khi [ADR-019](ADR-019-module-based-hardware.md) dừng thiết kế.
- Chấp nhận MST crossings 66→70, tổng MST 850,149→856,555 mm (+0,75%) để
  tăng khoảng tâm driver 11→15 mm, khoảng courtyard nhỏ nhất giữa hai cụm
  công suất 0,080→2,036 mm và tổ chức hàng thân/chữ nhất quán, font 1 mm.
  Khoảng courtyard không phải bề rộng đường đồng có thể sử dụng.
- Giữ cụm driver–tụ gọn; ghi nhận LSS–shunt tăng từ 3,525 lên
  3,825/3,889 mm. Không coi căn hàng đẹp hoặc MST ngắn là ưu tiên hơn đường
  hồi, width, thermal và tính hợp lệ cơ khí/RF.
- Sau rules/stackup, thêm gate T2a: route thật bốn OUT đồng thời, kiểm tra
  LSS/shunt return và bố trí GND/thermal sơ bộ trước routing tín hiệu toàn
  board. Giữ copper đã đạt trên cùng chuỗi board, không route một bản rồi
  vứt đi làm lại. Tickets routing đã archive cùng PCB khi [ADR-019](ADR-019-module-based-hardware.md) dừng thiết kế.
- Chỉ dùng tools/CLI; không mở hoặc điều khiển desktop. Không tự hạ rule,
  đổi pinout/placement/process hoặc dùng writer CAD tự viết để cứu routing.

## Hệ quả

MAIN đã có baseline V2, chưa có track/via/pour mới. DRC sạch ngoài connectivity
không chứng minh envelope OUT 50 mm, sáu via ngoài EP, 500 mm² GND nối thực
mỗi mặt mỗi driver hoặc đường hồi liên tục. T2a phải phát hiện xung đột sớm;
không đợi route hết mới đo nhiệt/return hoặc âm thầm nới yêu cầu.

Phần bố trí mở rộng về trái còn cần biên antenna DevKit thực. Phê duyệt
placement không biến kích thước DevKit/socket/Z-stack dự kiến thành số đo.
Không tự vẽ một vùng RF từ ảnh để đánh dấu đạt; dữ liệu thiếu và vùng routing
bị ảnh hưởng phải được ghi rõ, chặn nghiệm thu liên quan trước phát hành.

Nguồn kỹ thuật cho thứ tự routing: [Allegro A4950](../../hardware/_archive/libs-datasheets/A4950_Allegro.pdf),
trang 7, yêu cầu shunt return độc lập ngắn về star gần EP, ground plane và
decoupling VBB gần IC. Các width/via/diện tích được chọn cho MAIN nằm trong
[ADR-014](ADR-014-main-a4950-prototype.md), không phải rating suy từ datasheet.
