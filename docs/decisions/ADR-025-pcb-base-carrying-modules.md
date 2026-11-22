# ADR-025: PCB làm khung xe, mang các module có sẵn

**Status:** Accepted, chủ dự án yêu cầu ngày 2026-11-18. Ngày 2026-11-22 chủ dự án chốt board xe là **bản 1 mặt làm tại nhà** ([single](../../hardware/base/single/README.md)); bản 2 lớp chỉ còn là nguồn vị trí linh kiện. Bản 0.7 qua DRC, chưa làm, chưa kiểm trên xe.

## Bối cảnh

Ở vòng loại xe chạy chậm và to hơn quy định. Khung là nhiều tấm in 3D xếp tầng, dây rời nối các module. Chủ dự án chỉ cần xe dùng được tới chung kết 2026-12-02.

## Quyết định

- Một PCB làm khung cho cả xe: motor ở cuối board, thanh IR, bi cầu và ToF ở đầu, ESP32 ở giữa với bi cầu nằm dưới gầm nó. Pin đặt trên tầng in 3D có sẵn, bắt vào mẫu lỗ M3 50 × 35 mm của [base.dxf](../../hardware/base/mechanical/base.dxf).
- Module cắm header cái (devkit, DRV8871, ToF), IMU và buck hàn phẳng; không IC SMD. Linh kiện thụ động dùng SMD 0805 loại pad rộng cho hàn tay, vì thanh trở SIP khó mua.
- Giữ tâm hai bánh 95,2 mm. Theo chủ dự án, bố trí cơ khí được ưu tiên hơn firmware: mắt IR cách trục khoảng 90 mm, và GPIO được gán lại theo vị trí trên board (bảng trong [hardware/base](../../hardware/base/README.md)). [wiring.md](../../hardware/wiring.md) và `pins.hpp` theo bảng của bản 1 mặt.
- Chia board thành ba vùng: công suất ở cuối (nguồn vào, motor, cầu đấu POWER và MOTOR của DRV8871), logic ở giữa (ESP32), analog ở đầu (thanh IR, chia áp, ToF). DRV8871 lấy nguồn qua cầu đấu POWER, nằm ngay cạnh cầu đấu MOTOR; header của module chỉ mang tín hiệu điều khiển. Nhờ vậy dòng motor quay vòng trong khoảng 30 mm ở cuối board, không đi qua vùng analog.
- Vùng analog đặt tay, không dùng kết quả của placer: điện trở chia áp IR xếp thành cột ngay sau header J3 để dây ADC không phải băng qua hàng chân J3, và bus I2C/XSHUT đi thẳng tới ToF. Placer để các dây này đi vòng: tổng dây dài hơn mức tối thiểu 241 mm, sau khi đặt tay còn 119 mm.
- **Bản 1 mặt làm tại nhà** (chủ dự án chốt 2026-11-22): chỉ đồng mặt dưới, cầu nối bằng 3 điện trở 0 Ω và 2 dây nhảy; SDA và TOF_XSHUT đổi chân ESP32 (GPIO15 / GPIO17) để bus I2C chui dưới module đúng thứ tự chân ToF. Chi tiết và cách sinh lại ở [single](../../hardware/base/single/README.md).
- Giá trị nào còn phải chỉnh thì thiết kế để chỉnh được, thay vì đo trước: khe bắt thanh IR và bi cầu, chia áp IR bằng điện trở rời thay được, jumper cho chân IR của thanh, diode cho phép cắm USB khi buck đang cấp, chia áp đo pin bằng điện trở 1% để biết tỉ lệ ngay từ BOM.
- Dùng công cụ có sẵn thay vì tự viết: placement tối ưu bằng placer của KiCadRoutingTools (khoá linh kiện cơ khí, chia nhóm chức năng, vùng cấm dưới các module kê cao, thử 12 phương án và route nhanh để chọn), route bằng router của nó và không hạ dưới luật của board. Từ bản 0.6, file KiCad là bản gốc.

## Đánh đổi

- Không có chống cắm ngược pin; dựa vào giắc có cực. Thêm MOSFET hoặc diode công suất thì tốn chỗ và gây sụt áp ở đường motor.
- Board 73 × 117 mm; phần dài thêm dùng cho kênh đi dây quanh các hàng chân dày.
- GND chỉ đổ ở mặt dưới: dòng motor quay về pin chung đồng với GND của IR và ESP32. Kiểm trên xe bằng nhật ký IR khi motor tăng tốc và đổi chiều. Mỗi DRV cần thêm hai dây từ cầu đấu POWER.
- Một số kích thước thân module ước từ ảnh. Chúng chỉ quyết định khoảng hở; cần kiểm bằng bản in 1:1 trước khi đặt in.
- Khung in 3D hiện tại giữ làm phương án dự phòng nếu board không về kịp.
