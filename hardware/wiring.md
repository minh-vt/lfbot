# Đấu dây robot trên PCB base

Trạng thái: pin map và cách đấu theo [PCB base](base/README.md) bản 0.7 (ngày 2026-11-18). Board **chưa đặt in, chưa lắp và chưa kiểm trên xe**. `firmware/src/esp32/pins.hpp` theo bảng dưới (2026-11-19); firmware đó mới chạy trên khung dây rời cũ, chưa chạy trên board. Quyết định: [ADR-019](../docs/decisions/ADR-019-module-based-hardware.md) (dùng module), [ADR-025](../docs/decisions/ADR-025-pcb-base-carrying-modules.md) (PCB làm khung). Danh mục: [BOM](../docs/HARDWARE_BOM.csv). Pin map của khung dây rời cũ có trong lịch sử git của file này.

## Cấu hình

- ESP32-S3 N16R8 TDM2533 cắm trên hai hàng header cái, pin LiPo 3S (12,6 V khi đầy), hai motor GA12-N20 có encoder.
- Thanh **IR analog 8 mắt** của Thegioiic làm cảm biến line chính. Hai MKE-S16 mua dự phòng, không dùng trên board.
- Hai module **DRV8871**, mỗi module một motor; không dùng DRV8833 với 3S.
- Buck MP1584 ra 5 V, ToF400C (VL53L1X), GY-9250 (thực tế là MPU-6500, xem [IMU](#gy-9250-imu)).
- Đo pin bằng chia áp 47k/10k 1% có sẵn trên board, **không dùng module voltage sensor** nữa.

Nối module theo **tên chân in trên board và trên module**, không theo số thứ tự chân header.

## Nguồn

```text
Pin 3S (J1: BAT+ / GND) -> F1 PTC -> J2 công tắc -> VBAT_SW ┬-> C1 220 µF
                                                         ├-> V+ của hai DRV8871 (dây từ cầu đấu POWER)
                                                         ├-> U4 buck MP1584 -> D1 SS14 -> +5V -> chân 5V của devkit
                                                         └-> R1 47k / R2 10k -> GPIO1 (đo pin)
Devkit 3V3 -> thanh IR, ToF, IMU, VCC hai encoder
```

- **Không có chống cắm ngược pin.** Dùng giắc có cực (XT30) ở dây pin và kiểm cực theo chữ BAT+ / GND trên board.
- **Dòng motor đi riêng ở cuối board:** pin, cầu chì, công tắc, cầu đấu POWER rồi tới motor, không đi qua chân GND của devkit hay đường mass của cảm biến. Hai đầu cuộn motor chỉ nối vào hai đầu ra cầu H, không đầu nào nối cố định xuống GND.
- **Cắm USB khi buck đang chạy:** được, vì có diode SS14 từ buck vào chân 5V. Chỉnh biến trở của buck lên khoảng 5,2–5,3 V để bù sụt áp qua diode. Nạp firmware khi motor đang có nguồn thì vẫn phải tắt công tắc hoặc giữ motor ngắt.
- **Công tắc:** nút Start không thay công tắc nguồn.
- **Ngân sách dòng 3V3:** ổn áp trên devkit gánh toàn bộ (8 LED IR, ToF, IMU, hai encoder), chưa đo.

## Pin map

Đọc thẳng từ schematic của board. Hàng trước của devkit quay về thanh IR, hàng sau quay về DRV và motor.

| GPIO | Net | Nối tới | Ghi chú |
|---|---|---|---|
| 10, 9, 3, 8 | ADC_IR1…4 | IR D1…D4 qua chia áp | ADC1 |
| 7, 6, 5, 4 | ADC_IR5…8 | IR D5…D8 qua chia áp | ADC1 |
| 17 | I2C_SDA | ToF, IMU, J10 | |
| 16 | I2C_SCL | ToF, IMU, J10 | |
| 15 | TOF_XSHUT | ToF SHUT | Open-drain: kéo thấp để reset, nhả để chạy |
| 18 | IR_CTRL | Jumper J4 | Chỉ dùng khi cắm jumper chọn GPIO cho chân IR của thanh |
| 11 | BTN_START | SW1, J9 | Nút nối xuống GND, input pull-up |
| 12 | BTN_2 | SW2, J9 | Như trên; chức năng do firmware định |
| 13 | IMU_INT | IMU INT | |

**Bản 1 mặt ([base/single](base/single/README.md)) đổi hai chân:** SDA ở GPIO15, TOF_XSHUT ở GPIO17 (SCL giữ GPIO16). Firmware chạy trên board đó cần bảng chân riêng. Bản xưởng và bản 2 mặt giữ bảng trên.
| 38 / 39 | MOT_L_IN1 / IN2 | DRV8871 trái | PWM 20 kHz |
| 41 / 42 | MOT_R_IN1 / IN2 | DRV8871 phải | PWM 20 kHz |
| 21 / 47 | ENC_L_A / B | Encoder trái C1 / C2 | Input 3,3 V |
| 40 / 2 | ENC_R_A / B | Encoder phải C1 / C2 | Input 3,3 V |
| 1 | VBAT_SENSE | Chia áp pin 47k/10k | ADC1, tỉ lệ 5,7 |
| 48 | — | LED RGB WS2812 trên devkit | Không nối ra board; điều khiển bằng RMT |

**Chân để trống có chủ ý:**
- GPIO0, 45, 46: strapping.
- GPIO19/20: USB.
- GPIO35–37: Octal PSRAM.
- GPIO43/44: UART console.
- GPIO14: còn trống.

**GPIO dùng được dù có chức năng phụ:**
- **GPIO3 (IR D3)** là chân strapping, nhưng nó chỉ chọn nguồn JTAG và bị bỏ qua khi `EFUSE_STRAP_JTAG_SEL` = 0, là mặc định; board chỉ dùng nó làm đầu vào analog.
- **GPIO39–42** là chân JTAG phần cứng, nhưng mặc định JTAG đi qua USB.

Cả hai suy từ datasheet ESP32-S3 (xem [REVIEW](base/REVIEW.md#đối-chiếu-chân)).

ADC_IR và VBAT_SENSE đều ở ADC1 nên đọc được khi Wi-Fi bật.

## Thanh IR 8 mắt

- **Header J3** nằm trên hàng chân của thanh; nối bằng dây ngắn từ mặt dưới, **theo tên chân**: GND, IR, D1…D8, VCC (chữ in trên board). Không cắm header thẳng một-một.
- **Thứ tự kênh:** nhìn theo hướng tiến (phía ToF), D1 ngoài cùng trái, D8 ngoài cùng phải (chủ dự án xác nhận 2026-11-11). Nếu gá đảo thì firmware đảo thứ tự, không suy trái/phải từ ảnh nhìn mặt dưới thanh.
- **Chân IR của thanh** (điều khiển LED?) đi tới jumper J4 và chọn được 3V3, GPIO18 hoặc GND. Chưa biết thanh xử lý chân này ra sao khi để trống, nên cắm jumper trước khi chạy.
  - **Tìm xem chân này làm gì** (một lần, trên board): cắm J4 vào GPIO18, xe đứng yên, mở console. Gõ `irctl high`, rồi `monitor` hai lần: một lần thanh trên giấy trắng, một lần nhấc thanh lên không. Lặp lại với `irctl low`. Mức làm số đọc đổi theo giấy trắng là mức **bật LED**; nếu cả hai mức cho số như nhau thì chân này không điều khiển LED. `irctl off` để thả chân.
  - Ghi kết quả vào `pins::kIrCtrlWiring` (`kLedOnHigh` / `kLedOnLow`). Từ đó ADC đọc mỗi mắt lúc LED bật và lúc tắt, xen kẽ từng tick, và gửi hiệu hai số: ánh sáng phòng có ở cả hai lần nên bị trừ đi. Để `kNotUsed` thì firmware không đụng GPIO18; khi đó cắm J4 vào 3V3 hoặc GND.
- **Chia áp:** mỗi kênh có 10k nối tiếp và 47k xuống GND, SMD 0805, xếp thành cột ngay sau J3: hàng 10k (R10…R17) ở trước, hàng 47k (R20…R27) ở sau, cột trái nhất là D1. Tỉ lệ 47/57: 3,0 V thành 2,47 V, trong vùng tuyến tính của ADC tới khoảng 2,9 V. Thay điện trở để đổi tỉ lệ. Chia áp không bảo vệ được khi đấu nhầm 5 V hay 3S.

```text
IR Dn -> 10kΩ -+-> GPIO ADC trong bảng
               |
              47kΩ -> GND
```

**Đo ngày 2026-11-10** (nối thẳng D1–D8 vào ADC, chưa có chia áp): mức trắng đọc khoảng 1880–2850 trên thang 4095, khác nhau giữa các mắt, chưa kẹt đỉnh. Có chia áp thì các số này giảm theo tỉ lệ 47/57. Hiệu chuẩn đen/trắng, cực tính và phát hiện mẫu lỗi là việc của firmware.

## Hai DRV8871

- **Cắm module** vào header 4 chân (U2 trái, U3 phải), mặt linh kiện lên trên. Header chỉ mang IN2, IN1, GND; chân VM của header **không nối**.
- **Mỗi module có 4 dây ngắn** ở cạnh cầu đấu:
  - cầu đấu MOTOR vào pad M1 / M2;
  - cầu đấu POWER vào pad V+ / V− (**đúng cực theo chữ trên board**).
- **Motor** nối qua giắc encoder (M1, M2 nằm ở hai đầu giắc).
- **Đảo chiều từng bánh** bằng cấu hình firmware, không đổi dây.

DRV8871 làm việc 6,5–45 V, nhận logic 3,3 V. Hai chân IN có điện trở kéo xuống 100 kΩ nên lúc khởi động motor thả trôi. 3,6 A là dòng đỉnh, không phải dòng liên tục của mọi module. Điện trở ILIM trên module quyết định mức giới hạn dòng; mức này chưa đo.

## Encoder

Giắc JST ZH 1,5 mm 6 chân đứng: J7 bên trái, J8 bên phải. Thứ tự trên board: M1, GND, C1, C2, VCC, M2 (chủ dự án đọc từ motor).

- **Dây ZH bán sẵn có loại nối thẳng và loại đảo đầu.** Nếu đảo, M1/M2 của motor rơi vào VCC/GND encoder. Đo thông mạch chân 1 hai đầu dây trước khi cắm.
- **Cấp 3,3 V** cho encoder. Không cấp 5 V rồi đưa A/B thẳng vào ESP32. Tài liệu shop chưa xác nhận encoder chạy được ở 3,3 V với đúng lô hàng này.

## ToF400C (VL53L1X)

Cắm thẳng đứng vào J5 ở mép trước, mặt cảm biến hướng ra trước (suy từ ảnh; cắm thử để xác nhận).

| Chân module | Nối tới |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SDA | GPIO17 |
| SCL | GPIO16 |
| INT | Không nối |
| SHUT / XSHUT | GPIO15, open-drain; dùng pull-up có sẵn trên module |

- **Địa chỉ I2C** 7-bit mặc định là `0x29`.
- **XSHUT:** không đẩy lên 5 V, không thêm pull-up 5 V.
- **Trình tự khởi động:** không dùng trình tự hay thanh ghi của VL53L0X.

## GY-9250 (IMU)

Hàn thẳng header của module xuống J6 để module nằm phẳng dưới gầm ESP32, trục gyro Z thẳng đứng. Board tự nối các chân cấu hình:

| Chân module | Nối tới |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCL / SCLK | GPIO16 |
| SDA / SDI | GPIO17 |
| EDA, ECL | Không nối |
| AD0 / SDO | GND, địa chỉ `0x68` |
| INT | GPIO13 |
| NCS | 3V3, chọn I2C |
| FSYNC | GND |

**Đo ngày 2026-11-10:**
- **Chip thật:** module trả `WHO_AM_I=0x70`, tức chip **MPU-6500** (không có từ kế), dù tên bán là GY-9250.
- **Đã đọc được:** gyro và gia tốc qua I2C địa chỉ 0x68.
- **Firmware:** nhận cả họ 0x70/0x71/0x73 và không dùng từ kế.

ToF và IMU không trùng địa chỉ. Pull-up bus có sẵn trên module; đừng chồng thêm.

## Nút, I2C mở rộng và đo pin

- **Nút:**
  - SW1 (START, GPIO11) và SW2 (nút 2, GPIO12) là nút 6 mm trên board.
  - J9 (BTN_START, BTN_2, GND) cho nút gắn ngoài, song song với hai nút trên board.
  - Hành vi của nút Start theo [ARCHITECTURE](../docs/ARCHITECTURE.md#2-vận-hành-bằng-một-nút). Đích do robot tự nhận bằng thanh IR, không có cảm biến đích.
- **J10, I2C mở rộng:** +5V, +3V3, GND, SDA, SCL, cho cảm biến I2C sau này. Bus dùng chung với ToF và IMU. Trên bản 1 mặt, chân +5V của J10 để trống.
- **Đo pin:**
  - R1 47k và R2 10k (đều 1%) cùng C2 100 nF vào GPIO1, tỉ lệ 57/10 = 5,7: 12,6 V cho 2,21 V tại ADC.
  - Nhờ điện trở 1%, tỉ lệ biết ngay từ BOM. Vẫn nên đo một lần bằng đồng hồ vạn năng và lưu vào profile robot.
  - Đo tổng pack không thay được bảo vệ từng cell, sạc cân bằng hay ngắt khi xả quá.

## Nguồn đối chiếu

- **ESP32-S3:** [GPIO và hạn chế bộ nhớ/strapping/USB](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html), [ADC và hardware checklist](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html).
- **Thanh IR:** [Thegioiic](https://www.thegioiic.com/mach-cam-bien-do-line-8-kenh-khoang-cach-2-10mm), analog, nguồn 3,3–5 V; board cấp 3,3 V.
- **DRV8871:** [datasheet TI](https://www.ti.com/lit/ds/symlink/drv8871.pdf), [module Đức Huy](https://dientuduchuy.com/products/drv8871-3-6a-dc-motor-driver-mach-cau-h-pwm-dieu-khien-toc-do-dong-co-dc).
- **Các module khác:** [ToF400C Hshop](https://hshop.vn/cam-bien-khoang-cach-tof400c-vl53l1x-tof-laser-distance-sensor), [GY-9250 LKCG](https://lkcg.vn/cam-bien-gia-toc-9-truc-gy-9250-9dof-imu-mpu9250), [motor LKCG](https://lkcg.vn/dong-co-dc-giam-toc-servo-ga12-n20-12v-300rpm).

Nguồn của shop và ảnh pinout không thay cho việc xác nhận pinout encoder đúng lô, mức ILIM, ngân sách dòng, nhiệt hay tính hợp lệ thi đấu. Tài liệu này chốt cách đấu trên board; không khẳng định robot đã chạy.
