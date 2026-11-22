# Kế hoạch triển khai

Theo [ARCHITECTURE](ARCHITECTURE.md). Mỗi bước kết thúc bằng **xe chạy thật**; build và test xanh chỉ là điều kiện cần.

## Quy tắc làm việc

- Motor chỉ chạy khi chủ dự án nói "chạy" và đang quan sát.
- Mỗi bước: sửa code → `just format-check`, `just lint`, `just test`, `just build` → nạp → chạy trên xe → đổ `log` → ghi kết quả vào [IMPLEMENTATION](IMPLEMENTATION.md).
- Mở `/dev/ttyACM0` luôn với `dtr=False, rts=False`. Mở cổng làm board khởi động lại.
- Không stage, không commit.
- Số chỉnh trên đường thử ghi vào `config.hpp` kèm ngày và cách đo.

## Chuẩn bị (chủ dự án)

| Việc | Dùng cho |
|---|---|
| Đường thử: băng đen 20 mm trên nền trắng, có đoạn thẳng ≥ 1 m, một góc 90°, một góc nhọn, một giao cắt chữ thập, một đầu cụt | Bước 2–3 |
| Một viên gạch hoặc hộp cỡ 205×95×95 mm | Bước 4 |

## Bước 1 — Hạ tầng, an toàn, chuyển động có phản hồi

**Mục tiêu:** xe đi thẳng và quay đúng góc bằng encoder và gyro, dừng an toàn, có nhật ký.

| # | Việc | File |
|---|---|---|
| 1.1 | `types.hpp`: thêm `StopReason` (enum), bản ghi nhật ký, `v`/`ω` mục tiêu | `lib/robot/types.hpp` |
| 1.2 | `config.hpp`: sự thật phần cứng mục 3 kiến trúc; đảo encoder phải; mô hình motor đã đo | `lib/robot/config.hpp` |
| 1.3 | `motion`: tốc độ bánh từ encoder, `s`, `ψ`, bias gyro khi đứng yên, phát hiện kẹt bánh | `lib/robot/motion.*` |
| 1.4 | `drive`: `(v, ω)` → tốc độ hai bánh → duty; feedforward chia áp pin + PI back-calculation. Lấy lại từ thiết kế đã bám ±1% | `lib/robot/drive.*` |
| 1.5 | `maneuver`: quay tới góc, đi một quãng; có giới hạn thời gian thất bại | `lib/robot/maneuver.*` |
| 1.6 | `robot`: Stop không khoá, bỏ phase `Pulse`, nhật ký vòng 2000 bản ghi | `lib/robot/robot.*` |
| 1.7 | Driver IMU chỉ đọc gyro Z, ±1000 dps, DLPF; task sensors gyro 500 Hz, ToF theo data-ready | `src/esp32/imu.*`, `main.cpp` |
| 1.8 | ADC: trung bình mọi lượt quét giữa hai tick | `src/esp32/adc_inputs.*` |
| 1.9 | Hạn lệnh motor bằng esp_timer + nhịp tim control | `src/esp32/motors.*`, `main.cpp` |
| 1.10 | Console qua yêu cầu atomic; lệnh chỉ đọc `status`, `monitor`, `slots`, `log`. **Console không bao giờ điều khiển motor** | `src/esp32/console.*` |
| 1.11 | Bỏ LED đỏ nháy lúc boot; LED theo bảng màu kiến trúc | `main.cpp`, `robot.cpp` |
| 1.12 | ~~Chốt tỉ lệ chia áp pin~~ — đã khớp đồng hồ vạn năng (11,4 V) | — |
| 1.13 | Host test: Stop luôn dừng; mẫu cũ → coast; nhịp tim hết hạn → coast; odometry thẳng/quay | `test/` |

**Nghiệm thu trên xe** (chuỗi một lần bấm: tiến, quay +90°, quay −90°, lùi; không USB, đọc nhật ký từ flash):
- [x] Quay +90° và −90°: dừng ở ±87,8°, dấu gyro đúng (dung sai đã giảm để về gần 90°).
- [x] Tiến và lùi: ±2970 xung, giữ hướng lệch ≤ 1,2°. Chốt mm/xung = 0,0313 từ ToF.
- [x] Chốt `b` hiệu dụng qua `kTrackCountsPerRad` = 3040.
- [x] `log` đổ được nhật ký sau lần chạy, đọc từ flash sau khi cắm USB lại.
- [x] Thời gian tick tối đa 0,98 ms, không tick trễ (đọc khi chỉ cắm USB).
- [ ] Bấm nút trong lúc chạy: dừng ngay, LED vàng (đã có test host).
- [ ] Rút USB giữa chừng không làm motor chạy mãi (đã có test host cho hạn lệnh; chưa thử trên xe).

## Bước 2 — Tự quét IR và bám line chậm

**Trạng thái:** code xong, kiểm bằng xe giả lập, chưa chạy trên xe.

| # | Việc | File |
|---|---|---|
| 2.1 | Tự quét: bias gyro → quay trái/phải tăng dần bằng `maneuver` tới khi mọi mắt thấy hai mức → quay về hướng ban đầu | `lib/robot/line.*`, `race.*` |
| 2.2 | Cực tính từ dữ liệu quét (mức ít gặp hơn là line) | `lib/robot/line.*` |
| 2.3 | Đọc line: chuẩn hoá, vị trí, hình dạng `None/Line/Wide/Split`, phía cuối, mất ở mép/giữa | `lib/robot/line.*` |
| 2.4 | `race`: Quét → Bám; PD → `ω`; tốc độ chậm cố định | `lib/robot/race.*` |
| 2.5 | Ghi IR thô vào nhật ký trong lúc quét và bám | `robot.cpp` |
| 2.6 | Lý do dừng riêng: `sweep_no_turn`, `sweep_incomplete`, `stall`, `ir_stale`, `lost` | `types.hpp` |

**Nghiệm thu trên xe:**
- [ ] Bấm nút: LED cyan, xe tự quay qua line, chuyển xanh lá và bám.
- [ ] Đi hết đoạn thẳng + cua mềm của đường thử ở tốc độ chậm.
- [ ] Lấy nhật ký IR thật, chốt ngưỡng 0,2 / 0,5 / số mắt cho `Wide` và `Split`.
- [ ] Từ nhật ký thật dựng test hồi quy cho phần đọc line.

## Bước 3 — Góc, giao cắt, đích, tốc độ lượt 1

**Trạng thái:** góc, giao cắt, xác nhận đích đã viết; tốc độ và gain chưa chỉnh trên xe.

| # | Việc | File |
|---|---|---|
| 3.1 | Góc: mất ở mép → giảm tốc, `maneuver` quay về phía đó tới khi mắt giữa thấy line; giới hạn góc | `race.*` |
| 3.2 | Giao cắt: `Wide`/`Split` → giữ tốc độ quay hiện tại, giới hạn quãng đường | `race.*` |
| 3.3 | Mất line hẳn: quay về hướng cũ, lùi tìm, rồi `lost` | `race.*` |
| 3.4 | Đích: ứng viên → đi thêm một thân xe → quét ± góc nhỏ → trắng / bám tiếp | `race.*` |
| 3.5 | Chỉnh trên đường thử: gain PD, tốc độ hành trình, tốc độ vào góc. Ghi `config.hpp` kèm ngày | `config.hpp` |

**Nghiệm thu trên xe:**
- [ ] Qua góc 90° cả hai chiều.
- [ ] Qua góc nhọn.
- [ ] Đi thẳng qua giao cắt chữ thập, không rẽ nhầm.
- [ ] Đến đầu cụt: dừng, LED trắng.
- [ ] Thử ngược: góc gắt và giao cắt **không** bị báo đích.
- [ ] Ba lần liên tiếp đi hết đường thử mà không chạm tay.

**Mốc dự phòng:** nếu cuộc thi tới trước khi xong bước 4–5, firmware sau bước 3 phải đủ để chạy lượt 1 về đích trên sa bàn không có gạch.

## Bước 4 — Né gạch

| # | Việc | File |
|---|---|---|
| 4.1 | Tiếp cận: chỉ dùng mẫu ToF status 0; giảm tốc theo khoảng cách, dừng ở khoảng cách định | `race.*` |
| 4.2 | Đo bề rộng: quay phải chậm, tìm bước nhảy khoảng cách, `w = d·tan θ` | `race.*` |
| 4.3 | Ba chặng vòng phải bằng `maneuver`, ToF chặn va chạm | `race.*` |
| 4.4 | Bắt lại line trong cửa sổ quãng đường, quay về hướng tiếp cận | `race.*` |
| 4.5 | Lý do dừng: `blocked`, `bypass_lost` | `types.hpp` |
| 4.6 | Đo ToF so với thước ở 100/200/400 mm, ghi sai lệch | `config.hpp` |

**Nghiệm thu trên xe:**
- [ ] Gạch đặt ngang line, dọc line, lệch trái, lệch phải: xe vòng bên phải, không chạm gạch (chạm nhẹ hợp lệ theo Q-14, đẩy đổ thì không), bắt lại đúng line.
- [ ] Gạch đặt gần một góc của đường thử: không bắt nhầm sang đoạn line khác.

## Bước 5 — Map, lưu flash, lượt 2

| # | Việc | File |
|---|---|---|
| 5.1 | Ghi map mỗi 10 mm `{ψ, cờ}` trong lượt 1 | `lib/robot/course.*` |
| 5.2 | Sau đích: `κ(s)`, trần tốc độ, quét lùi/tiến | `lib/robot/course.*` |
| 5.3 | Phân vùng `course` 128 KB; ghi từ service khi đứng yên; version + checksum + cờ hoàn chỉnh | `partitions.csv`, `src/esp32/course_store.*` |
| 5.4 | Chọn lượt khi boot; giữ nút lúc cắm pin = xoá map; LED xanh dương/tím | `robot.*`, `main.cpp` |
| 5.5 | Lượt 2: `v(s)` + feedforward `ω = v·κ`; khớp mốc trong cửa sổ ±Δs; trượt mốc → tốc độ lượt 1 | `course.*`, `race.*` |
| 5.6 | Chỉnh `a_ngang`, `a_phanh`, `a_tăng` trên đường thử | `config.hpp` |
| 5.7 | Host test: `v(s)` không vượt trần, đủ quãng phanh; khớp mốc đúng/sai cửa sổ; map hỏng checksum bị từ chối | `test/` |

**Nghiệm thu trên xe:**
- [ ] Lượt 1 về đích, LED trắng; rút pin, cắm lại: LED tím.
- [ ] Lượt 2 về đích, **nhanh hơn lượt 1** theo đồng hồ.
- [ ] Đặt thêm vật làm lệch một đoạn: lượt 2 không mất line, quay về tốc độ lượt 1 tới mốc kế tiếp.
- [ ] Giữ nút lúc cắm pin: map bị xoá, LED xanh dương.

## Dọn dẹp sau mỗi bước

- Xoá code và file chết trong phạm vi vừa làm; cập nhật [IMPLEMENTATION](IMPLEMENTATION.md).
- Cập nhật [ARCHITECTURE](ARCHITECTURE.md) nếu số đo thật làm đổi thiết kế; đổi quyết định thì viết ADR mới.

## Rủi ro mở

| Rủi ro | Xử lý |
|---|---|
| Vùng chết bánh trái ~0,45 trên sàn, cao hơn số đo bánh treo | Vòng tốc độ có tích phân; phát hiện kẹt bánh |
| Ánh sáng sân ngoài trời (Q-16) làm lệch IR | Tự quét ở mỗi lần Start; theo dõi trong nhật ký |
| Gạch đặt sát một đoạn line khác | Cửa sổ bắt lại line; nghiệm thu riêng ở bước 4 |
| Một ToF phía trước không thấy bên hông gạch | Đo bề rộng bằng quay; chiều dọc lấy lớn nhất theo luật |
| Motor N20 thiếu mô-men khi tăng tốc lượt 2 | `a_tăng`, `a_phanh` đo trên đường thử, không suy từ datasheet |
