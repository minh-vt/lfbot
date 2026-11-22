# Trạng thái triển khai

Cập nhật 2026-11-11. Thiết kế đích ở [ARCHITECTURE](ARCHITECTURE.md) ([ADR-023](decisions/ADR-023-sensor-driven-two-run-firmware.md)); thứ tự việc ở [PLAN](PLAN.md).

## Bước 1 của [PLAN](PLAN.md): chuyển động có phản hồi (2026-11-11)

Core `lib/robot`: `motion` (encoder + gyro → tốc độ, quãng đường, hướng, bias gyro), `drive` (feedforward + PI từng bánh), `maneuver` (`go`, `turn`), `robot` (nút, an toàn, nhật ký). Nền tảng: gyro 500 Hz, ADC trung bình mọi lượt quét, watchdog `esp_timer`, hộp đen lưu flash. Chưa có bám line, né gạch, map, lượt 2.

**Chạy trên xe, trên sàn, không USB** (bấm nút = tiến 300 mm, quay +90°, quay −90°, lùi 300 mm):

| Kết quả | Số liệu |
|---|---|
| Cả chuỗi chạy trọn, kết thúc `done` | Nhật ký đọc từ flash sau khi cắm USB lại |
| Quay | +90° dừng ở 87,8°, −90° dừng ở −87,8° (dung sai khi đó 2,3°; đã đổi thành 1,1°) |
| Tiến và lùi | +2973 / −2971 xung; giữ hướng lệch tối đa 1,2° |
| Dấu gyro | `+1` đúng, không có `gyro_disagrees` |
| `kMmPerCount` | 0,0313 (ToF theo encoder, hai chiều 0,0331 và 0,0295; nhiễu ToF ±10 mm) |
| `kTrackCountsPerRad` | 3040 (hồi quy tốc độ quay theo hiệu tốc độ hai bánh); × mm/xung = 95 mm khớp bề ngang bánh |
| Mô hình motor trên sàn | gain 22480/21009 counts/s/duty, τ 0,62/0,55 s, vùng chết 0,21/0,22 |
| Đo ở nhà lúc treo bánh | **sai** gần gấp đôi gain và bốn lần τ so với trên sàn; không dùng lại |
| Thời gian tick (đọc khi chỉ cắm USB) | tối đa 0,98 ms, chu kỳ 2 ms, không tick trễ |

**Còn chưa kiểm trên xe:** bấm nút giữa lúc chạy (đã có test host và đường dừng), rút USB giữa lúc chạy để thử watchdog, `ready=0` khi pin thấp.

## Bước 2–3 của [PLAN](PLAN.md): tự quét, bám line, góc, giao cắt, đích (2026-11-11)

Code xong, **chưa chạy trên xe**. Một lần bấm nút: đo bias gyro → quét (xoay trái phải tăng dần tới khi mọi mắt thấy cả nền lẫn line) → quay về hướng ban đầu → bám line → xử lý góc / giao cắt → xác nhận đích → dừng, LED trắng.

- `line.*`: mức nền và mức line **của từng mắt** học từ histogram của lần quét (nền là giá trị hay gặp nhất, line là đuôi hiếm). Cực tính do cả thanh bỏ phiếu. Mắt có tương phản dưới 8% thang ADC (bão hoà, che nắng, mù) bị loại; còn ít hơn 6 mắt thì từ chối chạy (`scan_failed`). Mức nền trôi theo ánh sáng, có giới hạn.
- `race.*`: quét → quay về → bám → góc (line mất ở mép, hoặc tối một phía) → giao cắt (tối cả hai phía, đi thẳng qua) → xác nhận đích (line mất ở giữa: đi thêm 100 mm, quét ±26°, vẫn không có thì là đích).
- **Chỉ kiểm bằng xe giả lập** (`test_race`): đường thẳng trên 4 nền (nền sáng/line tối, nền tối/line sáng, nền chói/line nhạt, phòng tối), góc trái và phải, giao cắt, đặt lệch, hai mắt bão hoà vẫn chạy, ba mắt thì từ chối, không có line thì từ chối, nút dừng. Xe giả có hình học **giả định** (bước mắt 7 mm, line 20 mm, thanh trước trục 70 mm); không chứng minh xe thật chạy đúng.
- **Chưa đo trên xe, đang là giả định hoặc số tạm:** `kSteerPerEye`, `kFollowCps` (tốc độ chậm 94 mm/s), `kWideEyes` = 6 (giả định line phủ ít hơn 6 mắt), các góc quét. Log có cột `dark` (số mắt tối) để kiểm.

### Lần chạy thật đầu tiên của bước 2 (2026-11-11): `scan_failed`, và hai lỗi tìm ra từ đó

Xe quét 6 lần rồi từ chối chạy. Số IR thật của lần đó: nền đọc 3750–4025 (sát trần ADC 4095), line đọc 4095, tương phản chỉ ~250 xung, nhiễu của nền ~±60.

1. **Ngưỡng tương phản cố định 8% thang ADC loại cả tám mắt.** Thay bằng quy tắc theo từng mắt: mức nền và mức line phải cách nhau ít nhất 3 lần độ trôi của chính mắt đó trên nền (và ≥ 100 xung). Đưa lại đúng số IR thật của lần chạy đó qua mã mới: hợp lệ với 5 mắt dùng được (mắt 0, 3, 7 sát trần bị loại). Số mắt tối thiểu hạ từ 6 xuống 5.
2. **Ghi flash làm chết ADC.** Sau khi hộp đen ghi, `ir=[0,0,...]` và xe không bao giờ sẵn sàng lại. Ghi flash tắt cache hàng chục ms, ngắt DMA của ADC bị trễ, vòng DMA cạn nên dừng hẳn. Hậu quả: lượt 1 về đích → lưu map → ADC chết → lượt 2 không chạy. Sửa: ngắt ADC/I2C/PCNT/RMT nằm trong IRAM (`sdkconfig.defaults`), thêm tự khởi động lại ADC khi 5 ms không có lượt quét mới. Thử sáu lần chạy-ghi liên tiếp: ADC sống, `adc_restarts=0`. Khi ghi flash, tick điều khiển bị trễ tới 30 ms; chỉ xảy ra sau khi xe đã dừng.
3. Hộp đen giờ giữ **4 lần chạy gần nhất** (`slots`, `log [dòng [lần]]`), vì các lần thử nhiều nền bị ghi đè nhau.

### Bốn lần chạy thật (2026-11-11): bám được, cua sai, ngoài nắng mất hướng

Ba lần trong nhà bám được line nhưng **đâm thẳng ở ngã rẽ 90°** và mất line. Một lần ngoài nắng **mất hướng ngay khi ra nắng**. Hộp đen chỉ giữ được hai lần (xem dưới), nên chưa có số liệu cho lần ngoài nắng.

Từ 20 giây cuối của hai lần còn lại:
- Nền đọc 3800–4000, line 4095 (chạm trần ADC), mỗi lần bám chỉ 2–3 mắt tối: tương phản thật nhưng sát trần.
- **Lỗi cua:** chỗ giao tối ~170 ms (6 mắt tối) rồi hết line. Lần chạy mới nhất, quy tắc "tối một phía là góc, hai phía là giao cắt" bị mắt biên nhấp nháy đánh lừa: phán giao cắt, đi thẳng, dò ±26° không thấy gì và kết luận **"về đích" sai** (LED trắng). Lần trước, cùng loại sự kiện lại rẽ đúng.
- **Nhiễu "split" giả:** nhiều đợt `split` 10–150 ms với `dark=3` trong lúc bám bình thường: một mắt đơn lẻ gần trần ADC nhấp nháy qua ngưỡng, làm xe tạm ngừng lái.

Đã sửa: (1) giao điểm chỉ được phân loại **sau khi vượt qua**: line tiếp tục thẳng thì là giao cắt; hết line thì là góc hoặc ngã ba, và **không bao giờ** được coi là đích, xe rẽ về phía có nhiều "tối" hơn (hoà thì rẽ trái), không thấy thì thử phía kia; (2) mắt đơn lẻ cạnh một vệt thật bị bỏ; (3) ngưỡng "tối cả thanh" và "line mất ở mép" tính theo **số mắt dùng được**, không theo tám mắt. Bắt được lỗi (3) nhờ ca test hai mắt biên bị mất.

Hộp đen: lần chạy thứ ba/tư bị mất vì **rút pin ngay khi xe dừng, lúc đang ghi flash**. Đã thêm: LED **tím** trong lúc ghi (phải chờ hết tím mới rút pin), nhật ký 50 Hz × 4000 bản ghi (80 s, đủ một lượt), 5 lần chạy gần nhất. RAM dùng 75%.

### Vận hành hai chạm + sửa ngã rẽ và nắng (2026-11-11)

Ba quyết định của chủ dự án, đã thực hiện:
1. **Ngoài nắng `scan_failed` ngay từ bước quét:** chạy lại số IR thật của lần đó qua mã mới thấy hai lỗi: cực tính bỏ phiếu bằng **độ dài đuôi** để một mắt bị chói (đọc tụt xuống ~600) át bảy mắt còn lại; và ngưỡng tương phản `kContrastOverSpread=3` loại cả tám mắt vì nền ngoài nắng trôi ±200. Sửa: bỏ phiếu **một mắt một phiếu**; `kContrastOverSpread=2` (chạy lại đúng số liệu đó: 5/8 mắt dùng được, quét hợp lệ).
2. **Ngã rẽ 90°/chữ T khi tám mắt cùng tối:** thay quy tắc cũ (phán "giao cắt hay góc" ngay lúc thấy tối bằng mắt biên) bằng: chỉ phân loại **sau khi vượt qua** — line tiếp tục thẳng là giao cắt, hết line là ngã rẽ, và **không bao giờ** là đích; rẽ về phía có nhiều tối hơn, hoà hoặc suýt hoà thì **rẽ phải**. Đã có test chữ T phải rẽ phải.
3. **Hai chạm:** bấm lần 1 = quét rồi đứng yên chờ (LED **tím**); bấm lần 2 = mới chạy (LED xanh lá). Bấm khi đang quét hoặc đang chạy = dừng. LED trắng = về đích; chờ hết **tím** mới rút pin.

Nhật ký: 50 Hz × 4000 bản ghi (80 s) × 5 lần chạy gần nhất. RAM dùng 75%.

### Ngã rẽ tại chỗ, tăng tốc, né vật cản (2026-11-11)

Ba yêu cầu của chủ dự án, đã thực hiện:
1. **Rẽ tại ngã rẽ, không lướt qua rồi quay lại.** Trước đây "tối cả thanh" bị hiểu là giao cắt nên xe giữ hướng đi thẳng suốt vùng tối rồi mới quyết định. Giờ gặp vùng tối thì lập tức giảm còn `kNodeCrawlCps` (900 cps), giữ hướng: line xuất hiện lại thẳng = giao cắt (đi tiếp); line mất = ngã rẽ, xe lăn thêm `kCornerAdvanceMm` để trục bánh tới đúng vị trí rẽ rồi **quay tại chỗ** dò hai phía. Dò theo phía line đã lệch trước đó; đi thẳng tới thì **ưu tiên rẽ phải**. Test chữ T chốt "phải" (`y<-250`), đột biến đổi hướng làm test đỏ.
2. **Tăng tốc độ:** `kFollowCps` 3000 → 4500 (khoảng 140 mm/s). Vẫn là số chỉnh trên đường thật.
3. **Né vật cản:** khi ToF (tươi, `< kBlockApproachMm=150`) thấy gạch phía trước thì dừng và đi **vòng bên phải** cố định: quay phải 90°, đi ngang `kBlockSideClearMm`, quay trái 90°, đi dọc `kBlockLengthClearMm`, quay trái 90°, đi tìm line, rồi **quay về hướng ban đầu** trước khi bám lại (nếu không xe sẽ bám line theo hướng vuông góc và trượt qua bên kia). Test xe giả có gạch đi hết tới đích; đột biến bỏ bước chỉnh hướng làm test đỏ.

Các số `kCornerAdvanceMm` (khoảng cách thanh IR tới trục bánh) và kích thước vòng né vẫn là **số tạm**, phải đo/chỉnh trên xe thật; xe giả lập dùng thanh trước trục 70 mm. Mọi test đều là xe giả, không phải bằng chứng trên xe.

### Phản ứng nhanh hơn và tốc độ lượt 1 (2026-11-11)

Chủ dự án: robot phản ứng quá chậm, không kịp line, tăng tốc ngay từ lượt 1. Đã sửa:
- **Lái PD thay vì P.** Thêm số hạng theo **tốc độ lệch** (dịch chuyển của line theo thời gian), nên xe bắt đầu rẽ khi line vừa bắt đầu trôi ra mép, không đợi line chạm mép mới rẽ. `kSteerDamping`.
- **Tốc độ thích nghi:** full trên đường thẳng, tự giảm khi line lệch hoặc đang trôi (có sàn `kMinFollowFraction=0.35`), nên vào cua xe chậm lại đúng lúc rồi tăng tốc trở lại. `follow_omega`/`follow_speed` tách thành hàm thuần, có test trực tiếp.
- **Vòng tốc độ bánh nhanh gấp rưỡi** (`kSpeedLoopSpeedup` 2 → 3), vì độ trễ đáp ứng bánh là nút thắt thật, không phải tần số cảm biến.
- `kFollowCps` 3000 → 4500 (~140 mm/s).

Giới hạn còn lại, nói thẳng: mô hình motor đo trên sàn có τ ≈ 0,6 s (xe nặng), nên bánh mất ~0,3 s để đổi tốc độ dù vòng kín. Đó là vật lý của xe, không phải code. Trên đường thật, `kSteerDamping`, `kSteerPerEye`, `kSlowByPosition/Rate` phải chỉnh để không lắc mà vẫn kịp cua; các test trên xe giả không chứng minh được giá trị đó.

### Điều khiển bằng duty trực tiếp, bỏ vòng tốc độ (2026-11-11)

Chủ dự án hỏi: sao điều khiển theo counts/s (cps) mà không theo công suất (duty), cps có phải giả định vật lý? Đúng. Vòng tốc độ bánh cần mô hình motor (gain, τ, vùng chết) và cộng thêm ~0,3 s trễ — chính là thứ làm xe "phản ứng chậm". Đã bỏ nó:

- **Bám line giờ ra duty trực tiếp.** `Target{forward, turn}` → hai bánh ngay trong một tick. Lỗi line → lái đến thẳng bánh, không qua vòng tốc độ. Mô hình motor bị xoá khỏi firmware (chỉ còn trong xe giả của test).
- **Lái PD trên vị trí và tốc độ lệch** của line (`kSteerDutyPerEye`, `kSteerDutyDamping`): xe bắt đầu rẽ khi line vừa trôi, không đợi chạm mép.
- **Tốc độ thích nghi theo duty:** `kCruiseDuty=0.7`, giảm khi line lệch/trôi, sàn `kMinFollowFraction=0.45`.
- **Manoeuvre (đi đoạn, quay góc) thành PID trên khoảng cách/góc**, cũng bằng duty. Số hạng I thắng vùng chết (nếu không xe dừng cách đích ~10° vì duty dưới 0,21 không làm bánh quay); số hạng D phanh khi tới nơi.

Đánh đổi, nói thẳng: duty trực tiếp nhanh nhưng hai bánh có vùng chết lệch nhau nên đi thẳng hơi lệch; phản hồi line tự sửa. Manoeuvre PID chính xác nhưng chậm hơn vòng tốc độ cũ. Các gain là số chỉnh trên xe giả, chưa chỉnh trên xe thật; mô hình motor đo trên sàn (τ≈0,6 s) vẫn là giới hạn vật lý của bánh, không phải code.

### Lỗi đã gặp ở bước 1 và đã sửa

1. **Bảng phân vùng tuỳ chỉnh không bao giờ được nạp.** `partitions.csv` bị bỏ qua, board vẫn chạy bảng đơn mặc định, nên hộp đen không ghi được. Sửa bằng `board_build.partitions` trong `platformio.ini` và `CONFIG_PARTITION_TABLE_CUSTOM` trong `sdkconfig.defaults`; boot log phải thấy phân vùng `blackbox`.
2. **Cyan rồi đỏ khi bấm nút:** đo bias đòi 0,5 s liền không rung, mà ngón tay bấm nút làm xe rung. Sửa: tìm một khoảng 0,25 s đứng yên bất kỳ trong 3 s.
3. **Quay kẹt ở 72°:** ma sát tĩnh chặn khi tốc độ giảm theo sai số. Sửa: tốc độ quay tối thiểu 0,8 rad/s cho tới khi tới góc.
4. **ToF hiển thị mẫu cũ:** mẫu không hợp lệ không ghi đè mẫu cũ, nên log báo 951 mm suốt lúc xe đi. Sửa: mẫu cũ hơn 0,1 s ghi là `-1`; log có thêm cột `tof_st`.
5. **Mô hình motor treo bánh sai cho robot nằm trên sàn** (xem bảng trên).

### Việc bạn cần biết khi dùng xe

- Pin và USB **không được nối cùng lúc** (xem [wiring](../hardware/wiring.md)). Khi chạy bằng pin không có console; nhật ký lưu vào flash và đọc bằng `log` sau khi cắm USB lại. **Console chỉ đọc**: lệnh `go`/`turn` đã bị xoá để không bao giờ có chuyện motor chạy khi đang cắm USB.
- Mở cổng USB làm board khởi động lại; nhật ký trong RAM mất, nhật ký trong flash còn.
- Ghi chú công cụ: mở `/dev/ttyACM0` bằng `dtr=False, rts=False`.

## Lịch sử bring-up (firmware cũ đã bị xoá, giữ làm ghi chú bài học)

Bảng dưới mô tả firmware **trước** khi viết lại (cử chỉ nút, lưu NVS, đo motor tự động). Những tính năng đó không còn trong code; phần còn dùng được là các bài học về phần cứng.

Nạp qua CH343 (`/dev/ttyACM0`, UART0 console).

| Hạng mục | Kết quả |
|---|---|
| Boot, PSRAM tắt, ba task (control 500 Hz / I2C 5 ms / service 20 ms) | Ổn định, không panic |
| IR 8 kênh + pin trên ADC1 DMA | ~500 batch/s, không tràn; pin đọc 11,47–11,50 V khớp VOM |
| IMU MPU-6500, ToF VL53L1X | `0x70`, `0xEACC`, đều sẵn sàng |
| LED GPIO48 | Đã sửa thứ tự byte G-R-B; ba kiểu hiển thị đứng yên / nháy / thở |
| Nút GPIO40 | Cử chỉ hoạt động (chủ dự án xác nhận) |
| **Lưu flash** | Bản ghi NVS có version + checksum; ghi từ task service, chỉ khi đang `kReady`; đọc lúc boot. Sau khi cắt nguồn, robot khởi động lại với `calibrated=1 stored=1` và mô hình motor cũ, không phải đo lại |
| **Điều khiển bằng nút (không cần laptop)** | Bấm nhả = **hẹn giờ** rồi chạy thẳng 3 giây (bấm giữ lại để huỷ); giữ đến xanh dương = đo motor (bánh treo); giữ đến xanh lơ = đo hình học quay (trên sàn); giữ quá lâu = **không làm gì**. Không có cử chỉ nào phá huỷ dữ liệu; `clear` chỉ có trên console. `ready` chỉ bằng 1 khi đã đo **cả hai** |
| **Vòng tốc độ bánh (bước B)** | Chạy trên bánh treo: mục tiêu 2000/3000/6000/9000 counts/s đều bám trong ~1%, cả hai bánh, cả hai chiều; hết thời gian lệnh thì dừng |
| **Tự đo motor (end-to-end trên xe)** | **Thành công**: 12/12 bước, cả hai bánh, 4 mẫu mỗi bánh. Trái gain 11718 counts/s/duty, τ 0,149 s; phải 12114, τ 0,158 s; deadband 0,315 cả hai; chiều encoder −1 cả hai |

Các mục trên thuộc bản firmware trước khi rút gọn; driver vẫn giữ, phần core (vòng tốc độ, lưu flash, cử chỉ) đã bị xoá và sẽ làm lại theo [PLAN](PLAN.md).

### Lỗi đã gặp và đã sửa trong bring-up (đều là lỗi của firmware, không phải phần cứng)

1. PSRAM trong heap → panic `LoadProhibited` khi FreeRTOS cấp mutex. Tắt PSRAM.
2. IMU đọc sai `WHO_AM_I` ở lần boot đầu. Thử lại 5 lần.
3. ADC công bố mẫu mỗi 2 ms nhưng khai báo chu kỳ 375 µs → mẫu luôn bị coi là cũ. Producer tự khai báo chu kỳ đo được.
4. **Motor không quay:** MCPWM đặt force mức thấp bằng cơ chế *continue* nhưng nhả bằng lệnh của cơ chế *non-continue*, nên chân bị khoá ở 0. Nhả đúng cơ chế.
5. Lệnh `measure` từ console bị mất do race giữa hai task (control xoá cờ mỗi 2 ms). Đổi thành latch `std::atomic`, chỉ xoá khi đã hành động.
6. LED bị đảo đỏ/xanh lá do gửi R-G-B thay vì G-R-B; sau đó `rmt_transmit` hết transaction descriptor vì hàng đợi quá nông.
7. **Reset khi đo motor: tràn stack, không phải NVS.** `MotorCalibration::begin()` gán `*this = MotorCalibration{}`; đối tượng mang hai bộ đệm mẫu nên nặng 16,7 KB, và bản tạm đó nằm trên stack control task chỉ 6 KB. Nó ghi đè bộ nhớ kế cận, làm hỏng handle MCPWM và sinh ra các triệu chứng tưởng như deadlock NVS, watchdog, LED sai. Sửa: `reset()` gán từng trường, và lớp bị cấm copy để lỗi này không biên dịch được nữa. Đọc nước stack cao nhất (`monitor`, `stack ctl=`) là cách phát hiện rẻ nhất.
8. **Sai quy ước dấu motor.** `direction` chỉ nói encoder đếm âm hay dương khi duty dương; duty dương đã là chiều tiến. Nhân `direction` vào lệnh duty làm vòng kín thành phản hồi dương: bánh chạy hết tốc bất kể mục tiêu. Sửa ở `step_drive` và ở phép quay tự hiệu chuẩn.
9. **Anti-windup khoá cứng ở 0.** Khi feedforward một mình đã vượt trần duty, biên độ còn lại bằng 0 nên bộ tích phân không thể hạ lệnh, và bánh kẹt ở duty cực đại. Thay bằng back-calculation: phần lệnh không giao được thì lấy lại từ tích phân.
8. Task watchdog của IDF báo "task not found" và làm nghẽn console. Tắt WDT của IDF; an toàn motor dựa vào task I2C canh task control và cắt motor khi lệnh quá hạn.

### Lỗi cử chỉ nút (quan sát trên xe, 2026-11-11)

Giữ nút đến lúc LED xanh lơ nhưng xe lại chạy thẳng 3 giây. Nguyên nhân: mốc bắt đầu giữ bị đặt lại mỗi khi tiếp điểm nảy, nên một cú nảy giữa lúc giữ được đọc thành "bấm ngắn" và thời gian giữ bị tính lại từ cú nảy cuối. Sửa ba lớp:

1. Một lần **nhả** chỉ được công nhận khi nút đã rời thật `kGestureReleaseUs` (250 ms); nảy ngắn hơn bị coi là tiếp diễn và không sinh hành động nào.
2. Thời gian giữ tính từ **lúc tiếp điểm chạm** (`changed_us_`), không từ lúc bộ lọc đồng ý.
3. Hành động gây chuyển động chỉ **hẹn giờ** (`kStartCountdownUs`), và một lần bấm nữa trong lúc đếm ngược sẽ huỷ. Cử chỉ đến từ lúc dừng xe bằng nút cũng bị nuốt, nên không kích hoạt hành động khác.

Đồng thời bỏ cử chỉ "giữ quá lâu = xoá hiệu chuẩn": hành động phá huỷ không được nằm cuối một thao tác giữ.

### Ghi chú công cụ (đã tốn thời gian, ghi lại để khỏi lặp)

- Mỗi lần mở `/dev/ttyACM0` phải dùng `dtr=False, rts=False`. Đặt `rts=True` giữ chip ở trạng thái reset: board im lặng hoàn toàn, dễ bị chẩn đoán nhầm là firmware treo. `dtr=True` cũng vậy.
- Mở cổng sẽ reset board, mà hiệu chuẩn chưa lưu flash, nên mọi phép thử phải làm trong **một** phiên: mở cổng → `measure` → lệnh cần thử.

### Ghi chú phạm vi
Task host test: 6 bộ (`identify`, `motion`, `safety`, `mission`, `indicator`, và các ca end-to-end của đường đo motor). Trước đó một thời gian ngắn ba bộ chạy rỗng vì thân hàm bị xoá khi gộp binary; đã dựng lại.

## Kiểm chứng đã chạy (host)

`just check` (format, lint core/test, test host, build ESP32) và `just sanitize` đều xanh ngày 2026-11-10. Đó là test hợp đồng và cross-compile, **không** phải bằng chứng phần cứng hoặc hành vi.

## Lệch so với thiết kế đã duyệt

- `Board` gộp mọi thiết bị; I2C chặn trong task 2 ms.
- Lỗi khởi tạo gọi `ESP_ERROR_CHECK`, dẫn tới abort.
- `Calibrator` gộp ba quy trình; lý do lỗi là chuỗi chung; freshness định nghĩa ở ba nơi.
- Profile lưu nguyên struct; có cờ `*_ready` tách khỏi dữ liệu.
- Còn nợ sau lần chạy đầu: `ESP_ERROR_CHECK`/abort khi init, khóa `portMUX` quanh sao chép struct lớn, telemetry in qua `printf` chặn.
- `just lint` không quét `src/esp32`.
- Đo pin, phản hồi trạng thái, motor, mọi hành vi chạy: chưa có.
