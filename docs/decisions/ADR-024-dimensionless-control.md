# ADR-024: Điều khiển không thứ nguyên, lượt 2 dùng "công sức" đã ghi ở lượt 1

**Status:** Accepted, chủ dự án yêu cầu ngày 2026-11-16. Triển khai ngày 2026-11-19 cùng [ADR-026](ADR-026-one-follower-virtual-line.md), chưa chạy trên xe. Khác mô tả dưới đây ở ba điểm: mỗi đoạn hai byte (công sức, cờ) thay vì một; chưa có `g_ToF`; đoạn lượt 1 đã vòng gạch được coi là khó.

## Bối cảnh

Báo cáo nghiên cứu ([line-following-and-obstacles](../research/line-following-and-obstacles.md)) mô tả điều khiển theo vật lý: m/s, rad/s, độ cong, gia tốc ngang, mô hình motor. Mỗi đại lượng đó cần một phép đo hoặc một giả định (hệ số bám, hằng số thời gian motor, khoảng cách thanh IR tới trục). Robot không đo được phần lớn chúng, và mỗi giả định sai làm cả mô hình lệch khỏi xe thật. Chủ dự án đã bỏ vòng tốc độ bánh vì nó làm xe phản ứng chậm. Code lúc đó đã lái bằng duty và vị trí line theo bước mắt, tức gần như không đơn vị, và đó là phần chạy được.

## Quyết định

Mọi đại lượng điều khiển là **tỉ số với thứ robot tự có**. Gain là số thuần, chỉnh trên sân thử.

| Đại lượng | Thước đo | Miền |
|---|---|---|
| Lệnh motor | phần của điện áp pin (duty × `V_tham_chiếu / V_pin`) | −1…1 |
| Vị trí line `e` | phần của nửa thanh IR (mắt dùng được ngoài cùng = ±1) | −1…1 |
| Thời gian | tick điều khiển | số đếm |
| Quãng đường | count encoder, chỉ dùng để đánh dấu chỗ trên đường đã đi | số đếm |
| Khoảng cách ToF | phần của khoảng cách dừng trước gạch | 0…∞ |
| "Công sức" `w` | `|lệnh lái|`, đã có sẵn ở dạng duty | 0…1 |

**Lái:** `lái = Kp·e + Kd·(e − e trước đó)`. Một phi tuyến đơn giản được phép (ví dụ thêm gain khi `|e|` gần 1) nếu thử trên sân cho thấy cần.

**Trộn hai bánh, lái được ưu tiên:** trái = tiến − lái, phải = tiến + lái. Nếu `|tiến| + |lái| > 1` thì giảm `tiến`, không cắt `lái`. Bánh bão hoà không được làm mất khả năng bẻ lái.

**Tiến:** `tiến = F · min(g_line, g_ToF, g_nhớ)`, mỗi `g` trong 0…1:
- `g_line = 1 − c·|e|`, có sàn.
- `g_ToF` giảm dần khi ToF thấy vật gần hơn vài lần khoảng cách dừng.
- `g_nhớ` là 1 ở lượt 1; lượt 2 lấy từ bản ghi.

`tiến` tăng tối đa `Δ_lên` mỗi tick, giảm tối đa `Δ_xuống` (hai số thuần, chỉnh bằng mắt: bánh không trượt, xe không chúi). Đây là chỗ duy nhất thay cho "gia tốc".

**Lượt 1 ghi công sức, không ghi hình học.** Mỗi đoạn N count encoder ghi một byte: `w` lớn nhất trong đoạn, cộng cờ sự kiện (góc, vùng tối, gạch). Đoạn khó là đoạn robot đã phải lái mạnh; robot không cần biết vì sao nó khó.

**Lượt 2 nhìn trước theo thời gian.** Cửa sổ nhìn trước dài `H` tick tính theo tốc độ count hiện tại. Trong cửa sổ có đoạn mà lượt 1 phải lái mạnh hơn ngưỡng, hoặc có sự kiện góc/vùng tối, thì `g_nhớ` về mức lượt 1; không có thì `g_nhớ` được lên `F_nhanh / F`. Vì cửa sổ tính theo tick, xe càng nhanh càng nhìn xa, không cần công thức phanh.

**Định vị lượt 2 bằng thứ tự sự kiện.** Sự kiện thứ k ở lượt 2 khớp sự kiện thứ k ở lượt 1 nếu nằm trong cửa sổ count quanh chỗ đã ghi; khớp thì đặt lại quãng đường. Gạch **không** là mốc vì vị trí gạch ngẫu nhiên giữa hai lượt. Sau một lần né gạch, hoặc trượt mốc, `g_nhớ` ở mức lượt 1 tới khi khớp mốc kế tiếp.

**Chỉnh trên sân.** Kp, Kd, c, F, F_nhanh, Δ_lên, Δ_xuống, H, ngưỡng công sức: chỉnh ở nhà bằng sa bàn thử, rồi để robot dùng thời gian tự chỉnh tại sân (luật cho phép, miễn không nạp code và không điều khiển từ xa).

## Phương án bị loại

- **Profile theo độ cong và gia tốc ngang:** cần κ từ gyro làm trơn, giả định hệ số bám, đổi đơn vị bằng khoảng cách trục. Ba nguồn sai số để ra một con số mà "lượt 1 đã phải lái mạnh ở đây" cho trực tiếp.
- **Vòng tốc độ bánh theo mô hình motor:** đã thử và bị bỏ vì trễ; duty có bù áp pin đủ cho bám line.
- **Feedforward `ω = v·κ`:** cần cùng bản đồ độ cong; lợi ích chính (đã biết trước cua) đạt được bằng giảm tốc theo công sức ghi lại.
- **Đường tắt / line ảo:** luật chỉ cho rời line để né vật cản.

## Hệ quả

- `config.hpp` chỉ còn hằng số thuần và sự thật lắp ráp; gain không mang đơn vị vật lý.
- Map nhỏ (một byte mỗi đoạn), không cần xử lý sau đích ngoài tìm sự kiện.
- Mất tối ưu kiểu Robotrace (vào cua đúng giới hạn bám). Đổi lại, mọi số đều quan sát được trên xe và chỉnh được tại sân.
- Hình học của gạch (205×95×95 mm, Q-13) vẫn tính bằng mm đổi ra count: đó là kích thước vật, không phải mô hình điều khiển.
- ARCHITECTURE §6.3 và §6.7 được viết lại theo ADR này.
