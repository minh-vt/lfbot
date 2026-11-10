# ADR-016: MAIN routing trên placement baseline

> **Superseded by [ADR-019](ADR-019-module-based-hardware.md)** cho phần cứng hiện hành: dừng PCB riêng, chuyển sang module. Các status và quyết định bên dưới là lịch sử, không phải quyền tiếp tục chế tạo.

**Status:** Accepted; phần baseline/fixed-placement được thay bởi
[ADR-017](ADR-017-main-connectivity-placement.md) sau yêu cầu mới của chủ dự án.
Nội dung dưới đây giữ bối cảnh quyết định ban đầu: làm lại kế hoạch từ
`hardware/main/main.kicad_pcb` và `hardware/main/main.kicad_sch`.
Thay workflow R01–R12 và yêu cầu mở lại placement trong ADR-014/015;
không thay quyết định build firmware của ADR-015 hoặc envelope điện prototype.

## Bối cảnh

Các candidate tự dời A4950 ra vùng gá motor không lắp được. Kết quả router
và kiểm tra 2D trên chúng không chứng minh placement nguồn cần sửa.
MAIN nguồn đã place; người dùng yêu cầu routing trên chính baseline này.

## Quyết định

- Schematic giữ connectivity; PCB giữ pose/footprint/pad/outline/keepout.
  Không placer/seed search, không lấy candidate cũ làm đầu vào. Chỉ thay
  ngoài phạm vi sau khi có lỗi cụ thể, bằng chứng và người dùng duyệt.
- Spec/tickets MAIN mới là kế hoạch thực hiện; R01–R12 trong các ADR cũ là
  số ticket lịch sử, không phải gate đang hoạt động. Các lựa chọn điện/fab
  của ADR-014 vẫn là căn cứ prototype, không thành số đo hoặc chứng nhận.
- Routing gồm rules, track, via, GND/thermal và silk. Đề xuất đổi footprint
  paste trong ADR-014 không tự được thi công ở đây: giữ footprint baseline
  đã có bốn cửa paste. Mọi thay đổi footprint/process cần phạm vi riêng.
- Giữ hai lớp đồng hiện có và linh kiện mặt trước; ưu tiên đường hồi liên tục,
  không ép dùng cả hai mặt cho mọi net hoặc biến thành board một lớp.
- Dùng KiCad/Konnect và KiCadRoutingTools đã cài; không viết công cụ sửa CAD
  riêng. Bàn giao board nguồn đã route, không chuyển việc routing cho người dùng.
- Bỏ candidate/report/kế hoạch cũ khỏi cây làm việc sau khi lưu recovery ngoài
  repo; giữ CAD, BOM, dữ liệu cơ khí/datasheet và lịch sử ADR. Không lấy kết quả
  cũ để nghiệm thu layout mới.

## Hệ quả

Không cần duyệt lại toàn bộ placement mới được routing. Kiểm tra cơ khí/Z,
process fab và qualification vẫn cần trước các mốc phát hành/vận hành tương
ứng; DRC0 không chứng minh chúng. Nếu baseline thật sự không đáp ứng một rule,
phải nêu đúng xung đột thay vì tự dàn lại board hoặc bỏ rule.

Spec và tickets routing của board đã được archive khi dừng PCB theo [ADR-019](ADR-019-module-based-hardware.md); nội dung bên dưới giữ làm lịch sử.
