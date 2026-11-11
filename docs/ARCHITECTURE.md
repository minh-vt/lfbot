# Kiến trúc firmware LF-Bot

Thiết kế hiện hành, duyệt ngày 2026-11-11 ([ADR-023](decisions/ADR-023-sensor-driven-two-run-firmware.md)). Trạng thái thật của code nằm ở [IMPLEMENTATION](IMPLEMENTATION.md), thứ tự việc ở [PLAN](PLAN.md). Tài liệu này mô tả **cái sẽ xây**, không phải cái đã chạy.

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

Theo sa bàn mẫu vòng loại (`firmware/assets/track_vongloai.png`, chỉ là fixture, không đại diện sa bàn bí mật):

- Đường thẳng dài, S-curve.
- Zigzag góc nhọn, sóng vuông góc 90°.
- Hình vuông chồng nhau và vòng tròn chồng nhau: **giao cắt phải đi xuyên qua**.
- Hai đầu line cụt: **đích là chỗ line kết thúc**.
- Gạch 205×95×95 mm đặt trên line (FAQ Q-13), vòng **bên phải thân xe**.

## 2. Vận hành bằng một nút

| Thao tác | Kết quả |
|---|---|
| Bấm khi chờ | Start: tự quét IR rồi chạy luôn |
| Bấm khi đang quét hoặc chạy | **Dừng ngay.** Không có khoá nào chặn Stop |
| Giữ nút trong lúc cắm pin | Xoá map, quay về lượt 1 |

**Chọn lượt tự động:** có map hoàn chỉnh trong flash thì chạy lượt 2, không có thì chạy lượt 1. Lượt 1 không về đích thì không lưu map hoàn chỉnh, nên lần sau vẫn là lượt 1.

LED chỉ có màu đứng yên, **không nháy, không đếm số lần nháy**:

| Màu | Ý nghĩa |
|---|---|
| Xanh dương | Sẵn sàng lượt 1 |
| Tím | Sẵn sàng lượt 2 |
| Cyan | Đang tự quét IR |
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

Thông tin luật: line 20 mm đen trên nền trắng (Điều 4.2), gạch 205×95×95 mm (Q-13).

Những gì đổi theo sân và **robot tự đo** mỗi lần Start: mức trắng/đen từng mắt IR, cực tính, bias gyro.

## 4. Vai trò từng cảm biến

| Giai đoạn | IR | Encoder | Gyro Z | ToF | Pin |
|---|---|---|---|---|---|
| Chờ Start | Kiểm tra có mẫu | — | **Đo bias** khi đứng yên | Kiểm tra sẵn sàng | **Chặn Start** nếu yếu |
| Tự quét IR | Ghi min/max từng mắt | Vòng tốc độ bánh | **Giới hạn góc**, xác nhận xe quay thật | — | Bù feedforward |
| Bám line | **Vị trí và hình dạng line** | Vòng tốc độ, quãng đường `s` | Hướng `ψ` | Giảm tốc khi có vật | Bù feedforward |
| Góc 90° / nhọn | Phía line biến mất | Vòng tốc độ | **Quay tại chỗ có phản hồi**, giới hạn góc | — | — |
| Giao cắt | Nhận nhiều cụm / vùng rộng | Giới hạn độ dài vùng | **Giữ tốc độ quay hiện tại** | — | — |
| Tiếp cận gạch | Vẫn bám | Vòng tốc độ | — | **Khoảng cách**, dừng ở khoảng cách định | — |
| Đo bề rộng gạch | — | — | Góc quay | **Mép gạch** (khoảng cách nhảy vọt) | — |
| Vòng gạch | Chờ bắt lại line | **Quãng đường từng chặng** | **Góc từng chặng** | Chặn va chạm phía trước | — |
| Xác nhận đích | Line mất ở giữa thanh | Đi thêm một đoạn | Đang đi thẳng | Không có vật | — |
| Ghi map (lượt 1) | Sự kiện | **`s`** | **`ψ(s)`** | Vị trí gạch | — |
| Lượt 2 | Như lượt 1 | `s` để tra map | Như lượt 1 | Như lượt 1 | — |
| Mọi lúc | Mẫu cũ thì dừng | **Kẹt bánh** thì dừng | — | — | Dưới ngưỡng thì dừng |

**Accelerometer không dùng.** Không việc nào trong chiến thuật cần nó; driver chỉ đọc gyro Z.

## 5. Luồng dữ liệu

```text
IR ─┐
Encoder ─┤        ┌─ line: vị trí, hình dạng ─┐
Gyro Z ──┼─ số đo ─┤                           ├─ race: chọn việc và mục tiêu (v, ω)
ToF ─────┤ (có thời ├─ motion: v, ω, s, ψ ─────┤          ↑ course: map, v(s), mốc
Pin ─────┘  gian)  └──────────────────────────┘          ↓
                                       maneuver (quay tới góc / đi một quãng)
                                                          ↓
                                       drive: (v, ω) → tốc độ hai bánh → duty
                                                          ↓
                                       robot: kiểm tra an toàn → motor, LED, nhật ký
```

**Chỉ một nơi phát lệnh motor.** Bám line, né gạch, xử lý góc đều chỉ trả về mục tiêu `(v, ω)`, không ghi PWM.

## 6. Thuật toán

### 6.1 Tự quét IR có phản hồi gyro

Cách của Pololu QTR, thêm gyro để biết xe quay thật:

1. Xe đặt trên line, đứng yên 0,5 s: đo bias gyro.
2. Quay tại chỗ trái rồi phải bằng vòng tốc độ bánh, biên độ tăng dần, cho tới khi **mọi mắt đã thấy cả hai mức**. Gyro giới hạn góc tối đa.
3. **Cực tính:** mức nào xuất hiện ít hơn trong lúc quét là line (line 20 mm, nền rộng). Không gán "ADC nhỏ là đen".
4. Quay về hướng ban đầu theo gyro.
5. Lệnh quay mà gyro không thấy quay, hoặc encoder đứng yên: dừng với lý do riêng (`sweep_no_turn`, `stall`), không gộp chung.

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

### 6.3 Ba lớp điều khiển

1. **Tốc độ bánh** (encoder, 500 Hz): feedforward từ mô hình motor chia theo áp pin, cộng PI, chống bão hoà kiểu back-calculation. Lớp này giải quyết việc hai bánh có vùng chết khác nhau.
2. **Lái theo line:** PD trên vị trí line, ra tốc độ quay thân `ω`. Lượt 2 cộng feedforward `ω = v·κ(s)`.
3. **Tốc độ tiến `v`:** do `race` chọn (6.6).

Hai bánh nhận `v ± ω·b/2`. `b` là khoảng cách hiệu dụng hai bánh, đo bằng chênh lệch encoder so với góc gyro khi quay (tự quét đã quay, nên đo luôn được).

Gain PD, tốc độ hành trình và các gia tốc: **chỉnh một lần trên đường thử ở nhà**, ghi vào `config.hpp` kèm ngày và cách đo.

### 6.4 Tình huống trên line

| Thấy | Làm |
|---|---|
| `Line` | Bám line |
| `Wide` hoặc `Split` | Giữ **tốc độ quay hiện tại** theo gyro và đi qua: trên vòng tròn chồng nhau xe giữ độ cong, không bẻ sang nhánh khác. Giới hạn bằng quãng đường |
| `None`, line mất ở **mép** | Góc: giảm tốc, quay tại chỗ về phía đó tới khi mắt giữa thấy line; giới hạn góc theo gyro |
| `None`, line mất ở **giữa**, đang đi thẳng, ToF không thấy vật | Ứng viên đích → xác nhận (6.5) |
| Quay hết giới hạn vẫn không thấy | Quay về hướng cũ, lùi theo đường vừa đi để tìm; vẫn không thấy thì dừng, đỏ, lý do `lost` |

### 6.5 Đích

Đích là chỗ line kết thúc. Thời gian do cổng BTC đo, robot chỉ cần dừng và lưu map.

- **Ứng viên:** line biến mất ở giữa thanh, đang đi gần thẳng, không vật cản.
- **Xác nhận:** đi thêm khoảng một thân xe, quét ± một góc nhỏ theo gyro. Không thấy line thì là **đích**: dừng, LED trắng, lưu map. Thấy thì bám tiếp.
- Lượt 2 thêm điều kiện: ứng viên phải nằm gần điểm cuối map.
- Báo đích sớm là DNF, báo muộn gần như vô hại, nên nghiêng về bảo thủ.

### 6.6 Né gạch (vòng bên phải)

1. ToF hợp lệ (range status 0) thấy vật phía trước: giảm tốc theo khoảng cách, dừng ở khoảng cách định trước.
2. **Đo bề rộng gạch:** quay phải chậm theo gyro. Khi tia ToF trượt khỏi mép gạch, khoảng cách nhảy vọt. Góc `θ` tại đó cho nửa bề rộng `w = d·tan θ`.
3. Ba chặng, mỗi chặng có phản hồi encoder và gyro:
   - Quay +90°, đi ngang `w + 55 mm + biên`.
   - Quay −90°, đi dọc `205 mm + chiều dài xe + biên`. Dùng kích thước lớn nhất theo luật vì ToF không thấy chiều dọc gạch.
   - Quay −90°, đi tới khi IR thấy line.
4. Chỉ nhận line **trong cửa sổ quãng đường dự kiến**; sau đó quay về hướng `ψ` lúc tiếp cận và bám tiếp.
5. ToF thấy vật trên chặng thẳng thì dừng. Quá quãng đường giới hạn thì dừng với lý do `bypass_lost`.

Lệnh theo thời gian chỉ dùng làm **giới hạn thất bại**, không dùng để điều khiển chuyển động.

### 6.7 Hai lượt

**Lượt 1 (ưu tiên về đích):**
- Tốc độ hành trình cố định ở mức đã kiểm chứng.
- Tự giảm khi vị trí line lệch lớn, gặp `Wide`/`Split`/góc, hoặc ToF thấy vật.
- **Ghi map:** mỗi 10 mm theo encoder một mẫu 4 byte `{ψ, cờ sự kiện}`. Cờ: góc, giao cắt, gạch, đích.

**Sau đích** (task service, xe đứng yên):
- Độ cong `κ(s) = dψ/ds`, làm trơn.
- Trần tốc độ: `v(s) ≤ √(a_ngang/|κ|)`; góc 90° có tốc độ vào góc riêng; gạch dùng tốc độ tiếp cận.
- Quét lùi theo `a_phanh`, quét tiến theo `a_tăng`.
- Ghi map và `v(s)` vào phân vùng flash `course`.

**Lượt 2:**
- Cùng code bám line, góc, giao cắt, gạch như lượt 1.
- Tốc độ lấy từ `v(s)`, cộng feedforward độ cong.
- **Định vị:** `s` từ encoder, **chỉnh lại tại mỗi mốc** (góc, giao cắt, gạch) khi mốc xuất hiện trong cửa sổ ±Δs quanh vị trí đã ghi.
- **Mốc không xuất hiện trong cửa sổ:** dùng tốc độ lượt 1 cho tới khi khớp mốc tiếp theo. Không có biến "độ tin cậy".
- Map dang dở: dùng phần đã có, sau điểm cuối map chạy như lượt 1.

## 7. Luồng chạy trên ESP32-S3

| Task | Core | Nhịp | Việc |
|---|---|---|---|
| control | 1, một mình | 500 Hz | IR, encoder, ảnh chụp gyro/ToF, `Robot::step`, motor. **Chủ sở hữu duy nhất** của `Robot` |
| sensors | 0 | Gyro 500 Hz; ToF kiểm tra data-ready mỗi 20 ms | Không bao giờ chờ ToF |
| service | 0 | 50 Hz | LED, console, ghi flash khi đứng yên, xuất nhật ký |

Hiệu năng:
- **IR: trung bình mọi lượt quét ADC giữa hai tick** (~5 lượt ở 2667 lượt/s) thay vì chỉ lấy lượt cuối.
- Gyro: DLPF, dải ±1000 dps để quay tại chỗ không bão hoà; chỉ đọc 2 byte trục Z.
- Control không cấp phát động, không `printf`, không dùng `double`.
- Đo **thời gian thực thi tối đa và số tick trễ hạn**, hiện trong `status`.
- Giao tiếp giữa task:
  - Control công bố ảnh chụp trạng thái dưới spinlock.
  - Console gửi yêu cầu qua biến atomic, control nhận ở đầu tick.
  - Không task nào khác gọi hàm của `Robot`.
- **Nhật ký hộp đen:** vòng 2000 bản ghi nhỏ trong RAM (pha, vị trí line, hình dạng, `v`, `ω`, ToF, lý do dừng). Lệnh `log` đổ ra sau mỗi lần chạy.

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
firmware/lib/robot/          core: C++17, không ESP-IDF, chạy trên host
  types.hpp       số đo, đầu ra, enum lý do dừng
  config.hpp      mọi hằng số; mỗi số ghi nguồn: sự thật lắp ráp / đo ở nhà / chỉnh trên đường thử / luật
  line.*          tự quét, chuẩn hoá, vị trí, hình dạng
  motion.*        encoder + gyro → v, ω, s, ψ; bias gyro; phát hiện kẹt bánh
  drive.*         (v, ω) → tốc độ hai bánh → duty (feedforward + PI, bù pin)
  maneuver.*      quay tới góc / đi một quãng có phản hồi
  course.*        ghi map, tính v(s), khớp mốc
  race.*          một switch trạng thái: Quét, Bám, Góc, Giao cắt, Tiếp cận, Vòng gạch, Bắt line, Xác nhận đích, Xong, Lỗi
  robot.*         nút, an toàn, LED, nhật ký; Robot::step()
firmware/src/esp32/
  adc_inputs  encoders  imu  tof (+ tof_platform)  motors  status_led  button  console
  course_store    phân vùng flash course
  main.cpp        tạo task, nối driver với Robot
```

- `maneuver` là **một** cơ chế "quay tới góc / đi một quãng có phản hồi" dùng chung cho tự quét, góc 90°, xác nhận đích và vòng gạch.
- Không interface ảo, factory, plugin, event bus hay framework máy trạng thái.
- Phân vùng flash thêm `course` 128 KB; sa bàn 30 m cần khoảng 12 KB.

### Test

- Host test chỉ cho **bất biến không phụ thuộc mặt sân**:
  - Stop luôn dừng.
  - Mẫu cũ không giữ motor chạy.
  - `v(s)` không vượt trần và đủ quãng phanh.
  - Khớp mốc đúng/sai cửa sổ.
  - Toán odometry.
- Phần đọc line kiểm bằng **nhật ký thật ghi từ xe**, không bằng tín hiệu tự dựng.
- Nghiệm thu mỗi bước là xe chạy thật ([PLAN](PLAN.md)).
