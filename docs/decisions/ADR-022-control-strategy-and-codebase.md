# ADR-022: Hệ điều khiển tự suy hệ số, UX một nút + LED, cấu trúc codebase

**Status:** Superseded bởi [ADR-023](ADR-023-sensor-driven-two-run-firmware.md) ngày 2026-11-11. Trước đó: Accepted, chủ dự án duyệt ngày 2026-11-11. Bổ sung [ADR-021](ADR-021-field-adaptive-two-run-design.md); thay phần phản hồi bằng âm thanh/LED GPIO39 và cử chỉ nút của ADR-021. Chưa triển khai.

## Bối cảnh

Thiết kế trước có module, profile và an toàn nhưng **không có hệ thống điều khiển và chiến thuật**. Các đề xuất phụ cũng có lỗ hổng: cần “núm độ gắt” trong khi chỉ có một nút; dựa vào âm thanh trong khi không có loa. Chủ dự án xác nhận LED RGB trên board ở GPIO48 và yêu cầu không hardcode.

## Quyết định

- **Chiến thuật:** lượt 1 phải về đích, ghi đường; lượt 2 lấy thời gian bằng biên dạng tốc độ. Lượt 2 vẫn đi trên line (luật cấm cắt line).
- **Điều khiển ba tầng:** tốc độ bánh (feedforward + PI theo IMC/lambda từ mô hình motor tự đo), tốc độ quay có phản hồi gyro, lái theo line với `k = 4ζ²v/d²` (thanh IR nằm trước trục tạo cản).
- **Không có núm chỉnh tay.** Tốc độ do robot tự chọn: trần ổn định từ băng thông đo được, gia tốc ngang học bằng tăng dần rồi lùi khi phát hiện trượt.
- Tách bạch **hằng số vật lý** (đo/học, không hardcode) và **quy tắc thiết kế** không đơn vị, có lý do, nằm trong một header duy nhất.
- **Né gạch:** đường vòng phải có hình dạng định trước, kích thước từ FAQ Q-13 + ToF + biên an toàn, bám bằng encoder/gyro, cảm biến giám sát liên tục. Chủ dự án chấp nhận.
- **UX:** giữ để xem trước, nhả để xác nhận (Start / Tự hiệu chuẩn / Xóa map / huỷ). LED GPIO48 là kênh phản hồi duy nhất: màu + số lần nháy. Motor phát âm chỉ là thử nghiệm phụ.
- **Codebase:** core `lib/robot` một điểm vào `Robot::step`; một luật lái cho hai lượt; mỗi tri thức một nơi; platform mỗi thiết bị một file; không framework.

Chi tiết duy nhất ở [ARCHITECTURE](../ARCHITECTURE.md).

## Phương án bị loại

- **PD chỉnh tay theo sân:** phụ thuộc người và laptop; robot vô địch Robotrace (UnderBird) dùng gain cố định chỉnh tay, nhưng sân của họ trong nhà và quen thuộc.
- **Menu chỉnh số bằng một nút:** thêm lỗi thao tác, mâu thuẫn poka-yoke.
- **Hai bộ điều khiển line riêng cho hai lượt:** trùng lặp tri thức; hai lượt chỉ khác nguồn tốc độ và feedforward.

## Hệ quả và rủi ro

- Mô hình tầng 3 giả định góc nhỏ, không trượt, trễ nhỏ; góc 90° dùng hành vi quay theo gyro riêng.
- Cơ chế tăng dần tốc độ có thể làm lượt 1 chậm hơn mức cần; chấp nhận vì lượt 1 phải về đích.
- Code hiện tại (`Board`, `Calibrator`, `service()`) phải được sắp lại theo cấu trúc mới ở bước A.

Nguồn: Åström & Hägglund, *PID Controllers* (relay feedback); Berner và cộng sự 2018, *Control Engineering Practice* (so sánh autotuner); a1k0n, “Fast line-following robots” (2018); Espressif ESP32-S3-DevKitC-1 user guide (LED GPIO48/38); ASQ, mistake-proofing (Shingo).
