# Kiến trúc firmware LF-Bot

Thiết kế hiện hành, duyệt ngày 2026-11-11 ([ADR-023](decisions/ADR-023-sensor-driven-two-run-firmware.md)); điều khiển theo [ADR-024](decisions/ADR-024-dimensionless-control.md) (2026-11-16); một bộ bám line và né gạch bằng line ảo theo [ADR-026](decisions/ADR-026-one-follower-virtual-line.md) (2026-11-19). Trạng thái thật của code nằm ở [IMPLEMENTATION](IMPLEMENTATION.md), thứ tự việc ở [PLAN](PLAN.md). Tài liệu này mô tả **cái sẽ xây**, không phải cái đã chạy.

## 1. Mục tiêu và tiêu chí

| Lượt | Mục tiêu | Nghiệm thu |
|---|---|---|
| Lượt 1 | **Về đích hợp lệ** trong 180 s, ghi lại đường | Xe đi hết sa bàn thử, tự dừng ở cuối line, map được lưu |
| Lượt 2 | **Nhanh hơn lượt 1 trên cùng đường hợp lệ** | Thời gian bấm đồng hồ nhỏ hơn lượt 1, vẫn về đích |

Quy tắc chung:
- Gặp tình huống chưa phân biệt được thì **giảm tốc hoặc dừng để lấy thêm số đo**, không đoán.
- Không tìm đường tắt, không cắt line (Q-15).
- Người vận hành chỉ đặt xe, cắm pin, bấm nút. Không chỉnh tham số, không hiệu chuẩn tay, không cần laptop ngày thi (Điều 4.7).
- **Build và test xanh không phải nghiệm thu.** Nghiệm thu là xe chạy thật.

### Sa bàn phải xử lý

Theo sa bàn mẫu vòng loại ([`docs/track/track_vongloai.png`](track/track_vongloai.png), chỉ là fixture, không đại diện sa bàn bí mật):

- Đường thẳng dài, S-curve.
- Zigzag góc nhọn, sóng vuông góc 90°.
- Hình vuông chồng nhau và vòng tròn chồng nhau: **giao cắt phải đi xuyên qua**.
- Hai đầu line cụt: **đích là chỗ line kết thúc**.
- Gạch 205×95×95 mm đặt trên line (FAQ Q-13), vòng **bên phải thân xe**.

Sa bàn chung kết ([`docs/track/track_chungket.png`](track/track_chungket.png), BTC công bố; theo `track_chungket.json` ảnh 10 000 × 1 500 px = sân 10 000 × 1 500 mm, 1 mm mỗi pixel; ô đích 160 × 160 mm ở cuối line) thêm:

- Ba đường thẳng dài nối các cụm: phần lợi thời gian của lượt 2 nằm ở đây.
- Cụm hình chữ nhật chồng nhau, tổ ong lục giác, chuỗi hình thoi chạm đỉnh: nhiều ngã ba/ngã tư liên tiếp ở 90°, 60° và 45°.
- Sóng vuông nhiều góc 90° liền nhau và một bậc lõm có ô vuông: góc dừng-quay là chỗ tốn thời gian nhất.
- Vòng tròn tự cắt ở góc phải trên; hai sóng sin; zigzag nhọn.
- Hai vạch đậm ngang line và hai đoạn line đậm hơn.
- Một ô đen đặc gần cuối hàng 3, dài 164 mm theo ảnh: `kNodeStraightMm` = 250 mm để vùng tối này không bị coi là `wide_too_long`; đo lại trên sa bàn thật.
- Sóng vuông: các cạnh đứng cách nhau khoảng 100 mm theo ảnh, tức hai góc 90° cách nhau chưa tới một thân xe.

## 2. Vận hành bằng một nút

| Thao tác | Kết quả |
|---|---|
| Bấm lần 1 khi chờ | Tự quét IR rồi **đứng yên chờ** (LED tím) |
| Bấm lần 2 khi đã quét xong | **Chạy** |
| Bấm khi đang quét hoặc chạy | **Dừng ngay.** Không có khoá nào chặn Stop |
| Giữ nút trong lúc cắm pin | Xoá map, quay về lượt 1 |

**Chọn lượt tự động:** có map hoàn chỉnh trong flash thì chạy lượt 2, không có thì chạy lượt 1. Lượt 1 không về đích thì không lưu map hoàn chỉnh, nên lần sau vẫn là lượt 1.

LED chỉ có màu đứng yên, **không nháy, không đếm số lần nháy**:

| Màu | Ý nghĩa |
|---|---|
| Xanh dương | Sẵn sàng lượt 1 |
| Hồng (magenta) | Sẵn sàng lượt 2 (có bản ghi lượt 1 trong RAM hoặc flash) |
| Cyan | Đang tự quét IR |
| Tím | Đã quét xong, đứng yên chờ bấm chạy (hoặc đang ghi flash) |
| Xanh lá | Đang chạy |
| Trắng | Về đích, map đã lưu |
| Vàng | Dừng bằng nút |
| Đỏ | Dừng vì lỗi; lý do trong nhật ký (`log`) |
| Cam | Không cho Start: pin yếu hoặc cảm biến bắt buộc không sẵn sàng |

## 3. Sự thật phần cứng

Kiểm một lần, ghi thẳng vào `config.hpp` kèm nguồn. Không đo lại ở sân.

| Sự thật | Giá trị | Nguồn |
|---|---|---|
| Thứ tự IR | D1 ngoài cùng trái (nhìn theo hướng tiến, phía ToF) → D8 ngoài cùng phải; ADC1 ch {0,1,3,4,5,6,7,8} | Chủ dự án, [wiring](../hardware/wiring.md) |
| Chiều motor | Đảo bánh trái | Chủ dự án nhìn bằng mắt, 2026-11-11 |
| Chiều encoder | Sau khi đảo motor, lệnh tiến cho trái +14190, phải −14203: **đảo encoder phải** | Số đo trên xe, 2026-11-11 |
| Mô hình motor (bánh treo) | Trái gain 11718 counts/s/duty, τ 0,149 s; phải 12114, 0,158 s; vùng chết ≈ 0,315 | Đo trên xe; vòng tốc độ đã bám ±1% |
| Bánh xe | Ø 43 mm | Chủ dự án đo |
| Bề ngang thân | 110 mm mép ngoài (**không phải** khoảng cách hai bánh) | Chủ dự án đo |
| Chiều gyro Z | Kiểm ở lần chạy đầu bằng dấu chênh lệch encoder, rồi ghi hằng số | Chưa có |
| Tỉ lệ chia áp pin | Danh định 5,0 | **Cần một lần đo bằng đồng hồ vạn năng** |
| Chân IR của thanh (J4 → GPIO18) | Chưa biết có bật/tắt LED không, mức nào là bật | Đo bằng lệnh console `irctl` ([wiring](../hardware/wiring.md#thanh-ir-8-mắt)), ghi vào `pins::kIrCtrlWiring` |

Thông tin luật: line 20 mm đen trên nền trắng (Điều 4.2), gạch 205×95×95 mm (Q-13).

Những gì đổi theo sân và **robot tự đo** mỗi lần Start: mức trắng/đen từng mắt IR, cực tính, bias gyro.

## 4. Vai trò từng cảm biến

| Giai đoạn | IR | Encoder | Gyro Z | ToF | Pin |
|---|---|---|---|---|---|
| Chờ Start | Kiểm tra có mẫu | — | **Đo bias** khi đứng yên | Kiểm tra sẵn sàng | **Chặn Start** nếu yếu |
| Tự quét IR | Ghi min/max từng mắt | Quay có phản hồi | **Giới hạn góc**, xác nhận xe quay thật | — | Bù feedforward |
| Bám line | **Vị trí và hình dạng line** | Quãng đường `s` (count) | Hướng `ψ` | Giảm tốc khi có vật | Bù feedforward |
| Mất line (góc, đích) | Line quay lại dưới thanh | Lăn tới góc | **Giới hạn góc dò** mỗi phía | — | — |
| Giao cắt | Nhận nhiều cụm / vùng rộng | Giới hạn độ dài vùng | — | — | — |
| Vòng gạch | Chỉ nhận lại line trong hành lang | **Vị trí `x, y`** so với line ảo | **Hướng** so với line ảo | **Một lần nhìn**: khoảng cách tới mặt gạch | — |
| Xác nhận đích | Dò hai phía không thấy | Đi thêm một đoạn | Quay về hướng cũ trước | — | — |
| Ghi map (lượt 1) | Sự kiện, **công sức lái** | **`s`** | — | — | — |
| Lượt 2 | Như lượt 1 | `s` để tra map | Như lượt 1 | Như lượt 1 | — |
| Mọi lúc | Mẫu cũ thì dừng | **Kẹt bánh** thì dừng | — | — | Dưới ngưỡng thì dừng |

**Accelerometer không dùng.** Không việc nào trong chiến thuật cần nó; driver chỉ đọc gyro Z.

## 5. Luồng dữ liệu

```text
IR ─┐
Encoder ─┤        ┌─ line: vị trí, hình dạng ─┐
Gyro Z ──┼─ số đo ─┤                           ├─ pilot: chọn e, ra (tiến, lái)   
ToF ─────┤ (có thời ├─ motion: s, ψ, x, y ─────┤          ↑ course: công sức, mốc   
Pin ─────┘  gian)  └──────────────────────────┘          ↓
                                       drive: (tiến, lái) → trộn ưu tiên lái, bù pin → duty
                                                          ↓
                                       robot: kiểm tra an toàn → motor, LED, nhật ký
```

**Chỉ một nơi phát lệnh motor.** Bám line, né gạch, xử lý góc đều chỉ trả về `(tiến, lái)` không thứ nguyên, không ghi PWM.

## 6. Thuật toán

### 6.1 Tự quét IR có phản hồi gyro

Cách của Pololu QTR, thêm gyro để biết xe quay thật:

1. Xe đặt trên line, đứng yên 0,5 s: đo bias gyro.
2. Quay tại chỗ trái rồi phải với duty cố định, đảo chiều khi gyro qua biên độ, biên độ tăng dần, cho tới khi **mọi mắt đã thấy cả hai mức**.
3. **Cực tính:** mức nào xuất hiện ít hơn trong lúc quét là line (line 20 mm, nền rộng). Không gán "ADC nhỏ là đen".
4. Quay về hướng ban đầu theo gyro (`turn_toward`: tỉ lệ góc còn lại, giảm chấn theo tốc độ quay, không yếu hơn vùng chết).
5. Gyro ngược dấu encoder: `gyro_disagrees`. Quá `kCalibrateTimeoutUs`: `scan_failed`.

### 6.2 Đọc line mỗi tick

- Chuẩn hoá từng mắt: 0 là nền, 1 là line, theo min/max của chính mắt đó.
- **Vị trí:** trọng tâm các mắt có giá trị > 0,2, đơn vị bước mắt, −3,5 (D1) đến +3,5 (D8).
- **Hình dạng** theo các cụm mắt > 0,5:

| Hình dạng | Điều kiện |
|---|---|
| `None` | Không mắt nào |
| `Line` | Một cụm 1–3 mắt |
| `Wide` | Từ 6 mắt trở lên |
| `Split` | Hai cụm tách rời |

- Nhớ **phía cuối cùng còn thấy line** và **line biến mất ở mép hay ở giữa**.
- Các ngưỡng 0,2 / 0,5 / 3 / 6 là điểm khởi đầu; chốt bằng nhật ký IR ghi trên đường thử.

### 6.3 Điều khiển không thứ nguyên

Theo [ADR-024](decisions/ADR-024-dimensionless-control.md). Mọi đại lượng là tỉ số với thứ robot tự có; gain là số thuần, chỉnh trên sân thử.

- **Vị trí line `e`:** −1…1, mắt dùng được ngoài cùng là ±1.
- **Lái:** `Kp·e + Kd·(e − e trước)`, theo tick. Cho phép một phi tuyến đơn giản nếu thử trên sân cần.
- **Tiến:** `F · g_line · nhịp`. `g_line = 1 − c·|e|` có sàn; `nhịp` là 1 ở lượt 1, ở lượt 2 lấy từ bản ghi lượt 1 (6.7). Chưa có `g_ToF`: giả lập cho thấy lượt 2 vẫn vòng gạch không chạm khi chỉ dựa vào giới hạn `kDetourDuty`; nếu xe thật tới gạch quá nhanh thì thêm ở đây.
- **Đổi tốc:** `tiến` tăng tối đa `Δ_lên`, giảm tối đa `Δ_xuống` mỗi tick.
- **Trộn:** trái = tiến − lái, phải = tiến + lái. `|tiến| + |lái| > 1` thì giảm tiến, giữ lái. Nhân `V_tham_chiếu / V_pin`.

Không vòng tốc độ bánh, không mô hình motor, không độ cong. Encoder đếm quãng đường và phát hiện kẹt; gyro cho tự quét, góc dò, vị trí `x, y` của line ảo.

### 6.4 Một bộ bám line, `e` lấy từ đâu

Theo [ADR-026](decisions/ADR-026-one-follower-virtual-line.md). `pilot` chọn độ lệch `e` (bước mắt) mỗi tick; mọi trường hợp đi qua cùng `follow_turn` / `follow_forward`.

| Thấy | `e` (guide) | Tiến |
|---|---|---|
| `Line` | thanh IR (`line`) | theo `follow_forward` |
| `Wide` hoặc `Split` | `e` cũ (`memory`): vòng tròn tự cắt giữ độ cong | bò; vùng tối dài quá `kNodeStraightMm` là `wide_too_long` |
| `None` | lăn tới khi trục ở chỗ thanh mất line, rồi `±kSearchAimEyes` về phía line lệch lần cuối (mặc định phải), quá `kSearchTurnRad` thì đổi phía (`search`) | bò, rồi 0 |
| Hai phía đều không có | quay về hướng cũ, đi thẳng `kConfirmMm` (`straight`) | 0 tới khi thẳng hướng, rồi bò |
| Đang vòng gạch | line ảo (`virtual`) | `follow_forward`, tối đa `kDetourDuty` |

### 6.5 Đích

Đích là chỗ line kết thúc: line mất, dò hai phía không thấy, đi thẳng thêm `kConfirmMm` vẫn không thấy. Thời gian do cổng BTC đo; robot chỉ cần dừng trong ô đích 160 × 160 mm. Báo đích sớm là DNF, báo muộn gần như vô hại, nên mọi lần mất line đều dò hai phía trước.

### 6.6 Né gạch: line ảo bên phải

1. Đang bám line thật mà ToF (tươi) thấy vật gần hơn `kBrickSeeMm`: vẽ line ảo, neo ở vị trí `x, y` và hướng `ψ` lúc đó.
2. Line ảo là một bướu trơn lệch phải `kDetourSideMm` (nửa gạch 205 + nửa xe + biên, Q-13 xấu nhất), ra hết trước mặt gạch `kDetourLeadMm`, giữ tới sau mặt sau gạch `kDetourTailMm`, mỗi bậc dài `kDetourRampMm`.
3. Mỗi tick tính thanh IR **sẽ đọc gì** nếu line ảo có thật (độ lệch ngang tại thanh, chia `kEyePitchMm`) và đưa vào cùng luật lái.
4. **Nhận lại line thật** chỉ khi thanh đã qua mặt sau gạch **và** cách trục line cũ không quá `kRejoinCorridorMm`. Vệt đen ngoài hành lang là đoạn khác của sa bàn (Q-15).
5. Đi quá cuối line ảo `kDetourSearchMm` mà chưa thấy line: `line_lost`.

Giới hạn đã biết: gạch cách góc 90° chưa tới một thân xe thì line rẽ ngang có thể nằm ngoài hành lang.

### 6.7 Hai lượt

**Lượt 1 (ưu tiên về đích):** `nhịp = 1`, tốc độ `F` đã kiểm chứng. Mỗi đoạn `kStretchMm` (20 mm) ghi một `Stretch` hai byte: công sức `|lái|` lớn nhất trong đoạn (255 = `kTurnDutyMax`) và cờ: góc (bắt đầu dò), vùng tối (bắt đầu nhớ), gạch (đang đi line ảo). Lượt 1 không về đích thì bản ghi không được giữ.

**Sau đích:** bản ghi thành `course` trong RAM (bấm tiếp là lượt 2 ngay) và được lưu vào phân vùng flash `course` khi xe đã dừng. Không tính gì thêm.

**Lượt 2:**
- Cùng code bám line, góc, giao cắt, gạch như lượt 1.
- Nhìn trước `kLookaheadTicks` tick theo tốc độ count hiện tại. Trong cửa sổ mọi đoạn đều êm (công sức ≤ `kHardEffort`, không cờ nào): `nhịp = kFastPace`. Có đoạn khó, góc, vùng tối, hay đoạn lượt 1 đã vòng gạch (line dưới gạch chưa ai thấy): `nhịp = 1`. Cửa sổ chạm cuối bản ghi (đích): `nhịp = 1`.
- `nhịp` chỉ nhân vào `follow_forward` khi bám line thật; dò, nhớ, line ảo giữ tốc độ lượt 1.
- **Định vị:** `s` từ encoder; mỗi lần bắt đầu dò (góc) hoặc nhớ (vùng tối), tìm mốc cùng loại gần nhất, sau mốc khớp trước đó, trong `±kLandmarkWindowCounts` quanh chỗ bản ghi đang mong; khớp thì đặt lại `s`.
- **Gạch không phải mốc** (vị trí ngẫu nhiên giữa hai lượt). Sau né gạch hoặc trượt mốc: `nhịp = 1` tới khi khớp mốc kế tiếp.
- Hết bản ghi: chạy như lượt 1.

## 7. Luồng chạy trên ESP32-S3

| Task | Core | Nhịp | Việc |
|---|---|---|---|
| control | 1, một mình | 500 Hz | IR, encoder, ảnh chụp gyro/ToF, `Robot::step`, motor. **Chủ sở hữu duy nhất** của `Robot` |
| sensors | 0 | Gyro 500 Hz; ToF kiểm tra data-ready mỗi 20 ms | Không bao giờ chờ ToF |
| service | 0 | 50 Hz | LED, console, ghi flash khi đứng yên, xuất nhật ký |

Hiệu năng:
- **IR: trung bình mọi lượt quét ADC giữa hai tick** (~5 lượt ở 2667 lượt/s) thay vì chỉ lấy lượt cuối.
- **IR bật/tắt LED** (chỉ khi `kIrCtrlWiring` đã đo): mỗi tick đọc với LED ở một trạng thái rồi đổi; bỏ 2 lượt quét đầu sau mỗi lần đổi (một lượt còn trong hàng đợi DMA, một lượt cho mắt ổn định, chưa đo); gửi `(4095 + bật − tắt) / 2`. Ánh sáng phòng bị trừ; chiều của line vẫn do tự quét tìm.
- Gyro: DLPF, dải ±1000 dps để quay tại chỗ không bão hoà; chỉ đọc 2 byte trục Z.
- Control không cấp phát động, không `printf`, không dùng `double`.
- Đo **thời gian thực thi tối đa và số tick trễ hạn**, hiện trong `status`.
- Giao tiếp giữa task:
  - Control công bố ảnh chụp trạng thái dưới spinlock.
  - Console gửi yêu cầu qua biến atomic, control nhận ở đầu tick.
  - Không task nào khác gọi hàm của `Robot`.
- **Nhật ký hộp đen:** vòng 4000 bản ghi 50 Hz trong RAM (pha, lý do dừng, `s`, `ψ`, tốc độ và duty hai bánh, ToF, IR thô, nguồn `e`, hình dạng, vị trí line, nhịp lượt 2); 5 lần chạy gần nhất giữ trong flash. Lệnh `log` đổ ra.

## 8. An toàn

- **Hạn lệnh motor:** esp_timer kiểm tra nhịp tim của control mỗi 10 ms; quá hạn thì coast. Không phụ thuộc task khác.
- **Kẹt bánh:** có lệnh tiến mà encoder gần như đứng yên 300 ms thì dừng, lý do `stall`.
- **Mẫu cũ:** IR quá 3 chu kỳ không có mẫu mới thì dừng; không lái theo mẫu cũ.
- **Pin:** dưới ngưỡng LiPo 3S thì không cho Start, đang chạy thì dừng.
- Nút Stop thắng mọi trạng thái. Lúc boot không phát lệnh motor.
- Coast (`IN1=IN2=0`) và brake (`IN1=IN2=1`) của DRV8871 được chọn rõ cho từng tình huống dừng.

Mỗi lý do dừng là một giá trị enum, có trong nhật ký. Không gộp chung thành "lỗi".

## 9. Cấu trúc code

```text
firmware/core/       C++17 phẳng, không ESP-IDF, chạy trên host
  types.hpp       số đo, đầu ra, enum lý do dừng
  config.hpp      mọi hằng số; mỗi số ghi nguồn: sự thật lắp ráp / đo ở nhà / chỉnh trên đường thử / luật
  line.*          tự quét, chuẩn hoá, vị trí, hình dạng
  motion.*        encoder + gyro → tốc độ bánh, s, ψ, x, y; bias gyro
  drive.*         (tiến, lái) → duty hai bánh, ưu tiên lái, giới hạn đổi tốc, bù pin, kẹt bánh
  pilot.*         tự quét; chọn e (line, nhớ, dò, thẳng, line ảo); luật lái; Detour
  course.*        lượt 1 ghi công sức và mốc; lượt 2 nhìn trước, khớp mốc, ra nhịp
  robot.*         nút và chế độ (chờ, hiệu chuẩn, sẵn sàng, đua, dừng), an toàn, LED, nhật ký
firmware/esp32/      chương trình ESP32 (src_dir của PlatformIO)
  main.cpp        tạo task, nối driver với Robot
  console.*       lệnh đọc trạng thái, nhật ký, irctl
  pins.hpp        bảng chân của board 1 mặt
  drivers/        adc_inputs encoders imu tof (+ tof_platform) motors status_led button
  storage/        blackbox (nhật ký), course_store (bản ghi lượt 1)
firmware/test/       test hợp đồng trên host; plant.hpp là xe giả
firmware/third_party/vl53l1x/   ST ULD cho ToF
```

- Không có cơ chế "đi X mm / quay Y rad": mọi chuyển động trong lúc đua là bám một đường, thật hoặc ảo.
- Không interface ảo, factory, plugin, event bus hay framework máy trạng thái.
- Phân vùng flash `course` 64 KB; bản ghi tối đa 4096 đoạn (82 m) = 8 KB, sa bàn 30 m dùng khoảng 3 KB. Bản ghi có checksum và kích thước đoạn; firmware cắt line khác thì từ chối bản ghi cũ.

### Test

- Host test chỉ cho **bất biến không phụ thuộc mặt sân**:
  - Stop luôn dừng.
  - Mẫu cũ không giữ motor chạy.
  - Trộn bánh không bao giờ cắt lệnh lái; `tiến` không đổi nhanh hơn `Δ`.
  - Vòng gạch không chạm gạch; không nhận line nằm ngoài hành lang.
  - Lượt 2 không vượt tốc độ lượt 1 khi cửa sổ nhìn trước có đoạn khó; nhanh hơn lượt 1 trên cùng đường giả lập; vẫn vòng gạch mới không chạm.
  - Khớp mốc đúng/sai cửa sổ; sau line ảo thì không chắc vị trí.
  - Toán odometry.
- Phần đọc line kiểm bằng **nhật ký thật ghi từ xe**, không bằng tín hiệu tự dựng.
- Nghiệm thu mỗi bước là xe chạy thật ([PLAN](PLAN.md)).
