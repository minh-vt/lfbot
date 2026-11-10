# ADR-017: MAIN chốt lại placement theo connectivity

> **Superseded by [ADR-019](ADR-019-module-based-hardware.md)** cho phần cứng hiện hành: dừng PCB riêng, chuyển sang module. Các status và quyết định bên dưới là lịch sử, không phải quyền tiếp tục chế tạo.

**Status:** Accepted; baseline V1 được thay bởi
[ADR-018](ADR-018-main-spaced-placement.md) sau khi chủ dự án duyệt V2.
Các quyết định và số liệu dưới đây ghi mốc V1 theo yêu cầu tối ưu giao cắt;
không phải số liệu placement MAIN hiện tại. ADR này thay phần baseline của
ADR-016; giữ phạm vi routing, công cụ, pinout, BOM và envelope điện ADR-014/016.

## Bối cảnh

Sau khi chốt kế hoạch routing, chủ dự án mở lại placement để giảm giao cắt và
làm routing dễ hơn. Hai đường OUT của baseline cũ có cận dưới pad-edge vượt
56 mm khi tránh vùng motor, trong khi envelope nhánh là 50 mm. Căn chỉnh hình
thức hoặc chỉ đếm airwire không giải quyết xung đột đó.

## Quyết định

- Chọn các thay đổi cục bộ, không chạy auto-placer toàn board: xoay/dời hai
  cụm A4950, sắp điện trở encoder theo thứ tự chân MCU, đưa pull-up ToF gần
  hàng socket liên quan, dịch J3/J4 vào trong. Không đảo pin hoặc remap GPIO.
- Chấp nhận phương án 66 giao cắt MST tín hiệu thay vì phương án 62 nhưng
  giữ connector cũ: phương án 62 vẫn có hai cận dưới OUT vượt 50 mm. Vị trí
  cuối và phép đo nằm trong tài liệu placement đã archive cùng PCB (xem
  [ADR-019](ADR-019-module-based-hardware.md)).
- Giữ nguyên outline, lỗ gá, keepout motor/antenna, footprint/pad, pin-net,
  giá trị, DNP và mặt linh kiện. Không dùng phần trống trong hình chiếu DevKit
  làm bằng chứng về Z-stack hay tính hợp lệ của antenna.
- Bỏ 12 segment ngắn của baseline cũ trước khi nhận placement mới, tránh giữ
  copper mồ côi sau khi dời pad. Chưa route hoặc đổ copper mới. Chỉnh reference
  silk bằng CLI có sẵn, không sửa CAD bằng parser/writer tự viết.
- Đây là baseline cố định mới cho T1–T5, không phải board đã route hay tối ưu
  toàn cục. Chỉ dùng tools/CLI; không mở hoặc điều khiển desktop của người dùng.

## Hệ quả

Giao cắt MST giảm 94→66 và tổng chiều dài MST tín hiệu giảm khoảng 12,4%.
Đây là phép đo đường thẳng giữa pad, loại supply/GND, không phải chiều dài
track, số via hay bằng chứng rằng mọi net sẽ route đạt spec. Khoảng LSS–shunt
center-to-center tăng 2,76→3,53 mm; không đánh đổi đường hồi bằng việc chỉ tối
thiểu hóa giao cắt. T2 vẫn phải route trực tiếp, kiểm tra width và loop thực.

Các cận dưới OUT còn 36,50–48,67 mm; chỉ bỏ được dấu hiệu bất khả thi của
baseline cũ, chưa chứng minh giới hạn 50 mm sau routing. Thermal copper/via,
đường hồi, lắp ráp connector/cáp, kích thước DevKit/antenna và Z-stack vẫn cần
kiểm chứng ở các bước tương ứng. Không giảm mục tiêu nhiệt để nhận placement.

Spec và tickets routing của board đã được archive khi dừng PCB theo [ADR-019](ADR-019-module-based-hardware.md). Lịch sử T0 và ADR-016 được giữ nguyên bối cảnh, không coi kết quả cũ là nghiệm thu mới.
