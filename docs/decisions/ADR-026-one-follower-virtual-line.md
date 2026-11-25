# ADR-026: Một bộ bám line, né gạch bằng line ảo

**Status:** Accepted, chủ dự án yêu cầu ngày 2026-11-19. Dùng luật điều khiển của [ADR-024](ADR-024-dimensionless-control.md). Host test, chưa chạy trên xe.

## Bối cảnh

`race.cpp` có bốn máy trạng thái lồng nhau: quét, góc (đi 45 mm rồi dò hai phía), xác nhận đích, và vòng gạch sáu bước theo hình chữ nhật cố định. `maneuver` là một PID riêng cho "đi X mm / quay Y rad" chỉ để phục vụ các kịch bản đó. Mỗi kịch bản là một chuỗi bước có thể kẹt ở một bước, với tham số riêng không liên quan tới bám line. Bước tìm line cuối của vòng gạch nhận mọi vệt đen, nên có thể bám nhầm sang đoạn line khác (Q-15). Chủ dự án không muốn né vật cản bằng FSM và muốn codebase gọn hơn.

## Quyết định

**Robot chỉ làm một việc: bám một đường.** Mỗi tình huống chỉ khác nhau ở chỗ đường đó lấy từ đâu. `pilot` chọn độ lệch `e` (bước mắt) mỗi tick, và `e` luôn đi qua cùng một luật lái:

| Thấy | `e` lấy từ |
|---|---|
| Line | thanh IR |
| Vùng tối hoặc hai cụm (giao cắt) | `e` cũ, giữ hướng lái |
| Mất line ở mép, hoặc ngay sau vùng tối | ±tối đa về phía cuối cùng thấy; quay quá giới hạn gyro thì đổi phía |
| Mất line ở giữa thanh khi đang đi thẳng | 0, đi thẳng một đoạn; vẫn không thấy là đích |
| Đang né gạch | **line ảo** |

**Line ảo.** Khi ToF thấy gạch, robot vẽ một đường trong bộ nhớ, neo ở vị trí và hướng lúc thấy. Đường này lệch sang phải theo một cái bướu trơn: rộng nửa gạch + nửa xe + biên, dài gạch + biên, theo kích thước xấu nhất của Q-13. Mỗi tick robot tính thanh IR *sẽ đọc gì* nếu đường đó có thật (độ lệch ngang tại vị trí thanh, chia bước mắt) và đưa vào cùng luật lái. Không có bước "quay 90°, đi 210 mm".

**Chống bám nhầm line bằng hình học, không bằng trạng thái.** Trong lúc né, line thật chỉ được nhận khi thanh đã qua mặt sau gạch **và** nằm trong hành lang quanh trục line cũ. Ngoài cửa sổ đó, vệt đen bị bỏ qua.

**Gọn lại:**
- Không có cơ chế "đi X mm / quay Y rad" hay máy trạng thái cho từng kịch bản.
- `pilot.*` chọn `e` và giữ line ảo.
- `motion` thêm `x, y`.
- `drive` trộn bánh ưu tiên lái và giới hạn đổi tốc theo ADR-024.
- Tự quét IR lắc bằng gyro (đảo chiều khi quá biên độ), không dùng PID góc.
- Chế độ vận hành (chờ, hiệu chuẩn, sẵn sàng, chạy, dừng) vẫn ở `robot`. Đó là trình tự của nút bấm, không phải logic chuyển động.

## Phương án bị loại

- **Giữ FSM vòng gạch, chỉ thêm cửa sổ quãng đường:** vẫn là kịch bản mở vòng theo từng chặng, vẫn hỏng khi tốc độ hay góc tiếp cận khác lúc chỉnh.
- **Bám tường bằng ToF bên hông:** xe chỉ có một ToF hướng trước.
- **Trường thế (potential field) quanh gạch:** cần biết vị trí gạch theo thời gian thực, nhưng ToF mất gạch ngay khi xe quay đi. Line ảo chỉ cần một lần nhìn.

## Hệ quả

- Một bộ gain lái cho mọi tình huống; né gạch nhanh hay chậm là do `follow_forward`, không có tốc độ riêng cho từng chặng.
- **Giới hạn đã biết:** gạch cách góc 90° chưa tới một thân xe thì có thể không nhận lại được line, vì hành lang chặn line rẽ ngang. Host test ghi rõ trường hợp này.
- Hình học thanh IR (khoảng cách tới trục 90 mm theo ADR-025, bước mắt) thành sự thật lắp ráp bắt buộc, vì line ảo cần nó.
- Lượt 2 (`course`, ADR-024) và IR_CTRL làm sau, trên cấu trúc này.
