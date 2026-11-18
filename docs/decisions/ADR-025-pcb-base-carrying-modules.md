# ADR-025: PCB làm khung xe, mang các module có sẵn

**Status:** Accepted, chủ dự án yêu cầu ngày 2026-11-18. Thay phần "dừng PCB riêng" của [ADR-019](ADR-019-module-based-hardware.md) bằng một PCB base mang module; MAIN và IR-03 vẫn dừng; source của chúng đã xoá khỏi cây làm việc ngày 2026-11-18, lấy lại từ lịch sử git (commit `3a825ba`) nếu cần. Module của ADR-019 giữ nguyên; pin map đổi (xem bên dưới). Bản 0.7 đã route và qua DRC, chưa đặt in, chưa kiểm trên xe.

## Bối cảnh

Ở vòng loại xe chạy chậm và to hơn quy định. Khung là nhiều tấm in 3D xếp tầng, dây rời nối các module. Các lần làm PCB trước (MAIN, IR-03) dừng vì dùng nhiều linh kiện SMD và IC khó mua, không kịp hoàn thiện. Chủ dự án tìm được chỗ đặt in PCB và chỉ cần xe dùng được tới chung kết 2026-12-02.

## Quyết định

- Một PCB hai lớp làm khung cho cả xe: motor ở cuối board, thanh IR, bi cầu và ToF ở đầu, ESP32 ở giữa với bi cầu nằm dưới gầm nó. Pin đặt trên tầng in 3D có sẵn, bắt vào mẫu lỗ M3 50 × 35 mm của [base.dxf](../../hardware/base/mechanical/base.dxf).
- Module cắm header cái (devkit, DRV8871, ToF), IMU và buck hàn phẳng; không IC SMD. Linh kiện thụ động dùng SMD 0805 loại pad rộng cho hàn tay, vì thanh trở SIP khó mua.
- Giữ tâm hai bánh 95,2 mm. Theo chủ dự án, bố trí cơ khí được ưu tiên hơn firmware: mắt IR giờ cách trục khoảng 90 mm (firmware đang dùng 45), và GPIO được gán lại theo vị trí trên board (bảng trong [hardware/base](../../hardware/base/README.md)). [wiring.md](../../hardware/wiring.md) đã theo bảng mới; `pins.hpp` phải theo khi chuyển firmware lên board.
- Chia board thành ba vùng: công suất ở cuối (nguồn vào, motor, cầu đấu POWER và MOTOR của DRV8871), logic ở giữa (ESP32), analog ở đầu (thanh IR, chia áp, ToF). DRV8871 lấy nguồn qua cầu đấu POWER, nằm ngay cạnh cầu đấu MOTOR; header của module chỉ mang tín hiệu điều khiển. Nhờ vậy dòng motor quay vòng trong khoảng 30 mm ở cuối board, không đi qua vùng analog.
- Vùng analog đặt tay, không dùng kết quả của placer: điện trở chia áp IR xếp thành cột ngay sau header J3 để dây ADC không phải băng qua hàng chân J3, và SDA/XSHUT đổi GPIO cho nhau (17/15) để ba dây tới ToF đi song song qua khe giữa các chân J3. Placer để các dây này đi vòng: tổng dây dài hơn mức tối thiểu 241 mm, sau khi đặt tay còn 119 mm.
- Chia lớp chọn bằng số liệu (so 4 cách, xem hardware/base/REVIEW.md): GND đổ hai mặt, nguồn và motor ở mặt dưới, tín hiệu ở mặt trên.
- Giá trị nào còn phải chỉnh thì thiết kế để chỉnh được, thay vì đo trước: khe bắt thanh IR và bi cầu, chia áp IR bằng điện trở rời thay được, jumper cho chân IR của thanh, diode cho phép cắm USB khi buck đang cấp, chia áp đo pin bằng điện trở 1% để biết tỉ lệ ngay từ BOM.
- Dùng công cụ có sẵn thay vì tự viết: placement tối ưu bằng placer của KiCadRoutingTools (khoá linh kiện cơ khí, chia nhóm chức năng, vùng cấm dưới các module kê cao, thử 12 phương án và route nhanh để chọn), route bằng router của nó và không hạ dưới luật của board. Từ bản 0.6, file KiCad là bản gốc.

## Đánh đổi

- Không có chống cắm ngược pin; dựa vào giắc có cực. Thêm MOSFET hoặc diode công suất thì tốn chỗ và gây sụt áp ở đường motor.
- Board 73 × 117 mm vượt ngưỡng giá 100 × 100 mm của xưởng in; phần dài thêm dùng cho kênh đi dây quanh các hàng chân dày.
- Bản 0.7 có 36 via (23 via tín hiệu và nguồn, 13 via khâu). Khoảng 600 mm dây nguồn ở mặt dưới cắt vùng GND mặt dưới thành nhiều chỗ hẹp, nhưng mỗi mặt vẫn liền một mảng. Mỗi DRV cần thêm hai dây ngắn từ cầu đấu POWER.
- Một số kích thước thân module ước từ ảnh. Chúng chỉ quyết định khoảng hở; cần kiểm bằng bản in 1:1 trước khi đặt in.
- Khung in 3D hiện tại giữ làm phương án dự phòng nếu board không về kịp.
