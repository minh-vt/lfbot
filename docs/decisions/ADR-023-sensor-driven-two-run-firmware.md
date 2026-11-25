# ADR-023: Dùng đủ cảm biến, sự thật lắp ráp là hằng số, nghiệm thu bằng xe chạy

**Status:** Accepted, chủ dự án yêu cầu ngày 2026-11-11. Điều khiển theo [ADR-024](ADR-024-dimensionless-control.md); góc, đích và né gạch theo [ADR-026](ADR-026-one-follower-virtual-line.md).

## Bối cảnh

Firmware từng tự suy mọi hệ số và tự đo mô hình motor trước khi xe bám được một mét line; bản rút gọn sau đó lại bỏ hết phản hồi (chỉ IR + PWM). Cả hai đều không cho xe chạy, và không để lại dữ liệu để biết vì sao.

## Quyết định

- **Sự thật lắp ráp là hằng số**, kiểm một lần và ghi nguồn trong `config.hpp`: thứ tự IR, chiều motor, chiều encoder, chiều gyro. Robot chỉ tự đo những gì đổi theo sân: mức trắng/đen, cực tính IR, bias gyro.
- **Gain và tốc độ chỉnh trên đường thử**, ghi kèm cách đo; không có công thức tự suy hệ số.
- **Mỗi cảm biến có vai trò cụ thể:** IR bám line và phân loại hình dạng; encoder cho quãng đường và kẹt bánh; gyro cho quay, hướng và vị trí của line ảo; ToF thấy gạch; pin cho bù duty và chặn Start.
- **Một nút:** bấm để quét, bấm để chạy, bấm để dừng; chọn lượt tự động theo bản ghi lượt 1; giữ nút lúc cắm pin là xoá bản ghi. LED màu đứng yên, không nháy.
- **Mỗi lý do dừng là một giá trị enum**, có trong nhật ký hộp đen đọc qua console.
- **Nghiệm thu là xe chạy thật.** Host test chỉ cho bất biến không phụ thuộc mặt sân; phần đọc line kiểm bằng nhật ký thật.

Chi tiết ở [ARCHITECTURE](../ARCHITECTURE.md), thứ tự việc ở [PLAN](../PLAN.md).

## Phương án bị loại

- **Tự suy mọi hệ số tại sân:** tốn thời gian xây trước khi có bằng chứng xe chạy.
- **Chỉ IR + PWM mở vòng:** không bù được vùng chết khác nhau giữa hai bánh, không có quãng đường cho lượt 2.
- **Quét IR bằng quay theo thời gian:** không biết xe có quay thật không.

## Hệ quả

- Đổi pin, motor hoặc gá cảm biến thì kiểm lại các hằng số lắp ráp.
- Gain chỉnh ở nhà có thể kém tối ưu trên sân thi; lượt 1 ưu tiên về đích nên chấp nhận.
