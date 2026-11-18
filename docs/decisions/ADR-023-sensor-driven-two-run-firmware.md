# ADR-023: Firmware dùng đủ cảm biến, sự thật lắp ráp là hằng số, nghiệm thu bằng xe chạy

**Status:** Accepted, chủ dự án yêu cầu ngày 2026-11-11. Thay [ADR-022](ADR-022-control-strategy-and-codebase.md). Thay phần cử chỉ nút, đích 8 mắt trắng và "profile sân do người giữ nút đo" của [ADR-021](ADR-021-field-adaptive-two-run-design.md). Chưa triển khai. Phần điều khiển (vòng tốc độ bánh, profile độ cong) bị thay bởi [ADR-024](ADR-024-dimensionless-control.md).

## Bối cảnh

Sau ADR-022, firmware dựng ba tầng điều khiển tự suy hệ số, tự đo mô hình motor, tự đo hình học quay và lưu flash **trước khi xe bám được một mét line**. Khi chủ dự án yêu cầu đơn giản hoá, bản rút gọn lại bỏ hết phản hồi: xe chỉ còn IR và PWM mở vòng. Encoder, gyro, ToF, pin và flash bị bỏ không; nút Stop bị khoá; console gọi `Robot` từ task khác; mọi lý do dừng gộp thành một màu đỏ. Xe không chạy được, và không có dữ liệu để biết vì sao.

## Quyết định

- **Sự thật lắp ráp là hằng số**, kiểm một lần và ghi nguồn: thứ tự IR, chiều motor, chiều encoder, chiều gyro, mô hình motor đã đo. Robot chỉ tự đo những gì đổi theo sân: mức trắng/đen, cực tính IR, bias gyro.
- **Gain lái, tốc độ, gia tốc chỉnh một lần trên đường thử ở nhà**, ghi kèm ngày và cách đo. Bỏ công thức tự suy hệ số và cơ chế tự chọn tốc độ của ADR-022.
- **Mỗi cảm biến có vai trò cụ thể**: IR bám line và phân loại hình dạng; encoder cho vòng tốc độ, quãng đường, kẹt bánh; gyro cho quay có phản hồi, giao cắt, map; ToF cho tiếp cận và đo bề rộng gạch; pin cho bù feedforward và chặn Start. Accelerometer không dùng.
- **Một cơ chế `maneuver`** (quay tới góc / đi một quãng có phản hồi) dùng chung cho tự quét, góc, đích, vòng gạch.
- **Đích là chỗ line kết thúc**, nhận bằng line biến mất ở giữa thanh rồi xác nhận bằng đi thêm và quét.
- **Một nút, không cử chỉ giữ:** bấm là Start/Stop; chọn lượt tự động theo map trong flash; giữ nút lúc cắm pin là xoá map. LED màu đứng yên, không nháy.
- **Lượt 2 định vị bằng mốc**: trượt mốc thì dùng tốc độ lượt 1 tới mốc kế tiếp; không dùng điểm "độ tin cậy".
- **Mỗi lý do dừng là một enum**, có nhật ký hộp đen đọc được qua console.
- **Nghiệm thu là xe chạy thật.** Host test chỉ cho bất biến không phụ thuộc mặt sân; phần đọc line kiểm bằng nhật ký thật.

Chi tiết duy nhất ở [ARCHITECTURE](../ARCHITECTURE.md), thứ tự ở [PLAN](../PLAN.md).

## Phương án bị loại

- **Tự suy mọi hệ số tại sân (ADR-022):** tốn thời gian xây trước khi có bằng chứng xe chạy; mỗi bước đo là một nghi thức cho người vận hành.
- **Chỉ IR + PWM mở vòng:** không bù được vùng chết khác nhau giữa hai bánh, không quay đúng góc, không có map cho lượt 2.
- **Quét IR bằng quay theo thời gian:** không biết xe có quay thật không; đã thất bại trên xe.
- **Đích = 8 mắt trắng sau một khoảng thời gian:** không phân biệt được với mất line ở góc.

## Hệ quả

- Đổi pin, motor hoặc gá cảm biến thì phải kiểm lại các hằng số lắp ráp.
- Gain chỉnh trên đường thử ở nhà có thể kém tối ưu trên sân thi; lượt 1 ưu tiên về đích nên chấp nhận.
- Kích thước dọc gạch lấy lớn nhất theo luật, nên vòng gạch dài hơn mức cần.
