# Đấu dây robot dùng module

Trạng thái: pin map đã chốt với chủ dự án, đối chiếu ảnh DevKitC ngày 2026-11-10. Đây là kế hoạch đấu dây, **chưa phải firmware đã triển khai hoặc phần cứng đã kiểm chứng**. Firmware hiện tại chỉ đọc sensor, chưa điều khiển motor ([IMPLEMENTATION](../docs/IMPLEMENTATION.md)). Quyết định chuyển sang module: [ADR-019](../docs/decisions/ADR-019-module-based-hardware.md). Danh mục: [BOM](../docs/HARDWARE_BOM.csv).

## Cấu hình

- ESP32-S3 N16R8 TDM2533, pin LiPo thường 3S (12,6 V khi đầy), hai motor GA12-N20 encoder 12 V.
- Một thanh **IR analog 8 mắt** của Thegioiic làm cảm biến line chính. Hai MKE-S16 đã mua để dự phòng, không đấu vào cấu hình này.
- Hai module **DRV8871**, mỗi module một motor; chủ dự án xác nhận đã dùng đúng driver này (2026-11-10). Không dùng DRV8833 với 3S.
- Một module **voltage sensor** (chia áp khoảng 1:5) để đo pin; xem [Nút và đo pin](#nút-và-đo-pin).
- Buck 5 V, một ToF400C VL53L1X và GY-9250 đã có theo xác nhận của chủ dự án.

Đấu theo **tên GPIO trên board**, không theo số chân IC, số thứ tự header hoặc số pad của PCB cũ. Ảnh người dùng cung cấp là DevKitC tham khảo: mặt linh kiện hướng về người đọc, anten trên, USB dưới. Trái/phải trong bảng vị trí là hàng chân trong ảnh, không phải phía robot. Tên GPIO đã đối chiếu; vị trí header TDM2533 không được suy ra chỉ từ ảnh DevKitC.

## Nguồn

```text
Pin 3S (+) -> cầu chì / công tắc nguồn ->+-> VM driver trái
                                        +-> VM driver phải
                                        +-> IN+ buck 5V
Pin 3S (-) -------------------------------> GND chung / IN- buck

Buck OUT+ 5V -> chân 5V ESP32-S3
Buck OUT-   -> GND chung
ESP32 3V3   -> IR VCC, GY-9250 VCC, ToF VIN, VCC hai encoder
```

GND driver đi bằng dây công suất về điểm phân phối nguồn, không qua chân GND ESP32/breadboard. Hai đầu cuộn motor chỉ vào hai đầu ra cầu H, **không đầu nào nối cố định xuống GND**. GND encoder là dây riêng. Có chung tham chiếu điện không có nghĩa cho dòng motor đi qua dây mass cảm biến.

Nguồn encoder 3,3 V là phương án cho encoder hỗ trợ 3,3 V; tài liệu shop chưa khóa pinout/mức logic đúng lô. Không cấp 5 V rồi đưa A/B thẳng vào ESP32; không đoán màu hoặc thứ tự giắc sáu dây. Khả năng tải/nhiệt rail 3V3 của DevKit và dây nguồn chưa được đo; không coi pin map là xác nhận ngân sách dòng đã đạt.

Khi nạp USB: ngắt nguồn pin và tháo dây **buck OUT+ -> ESP32 5V** để tránh backfeed khi chưa xác minh sơ đồ nguồn TDM2533. Không dùng nút phần mềm thay công tắc ngắt nguồn. Khi firmware chưa migrate, giữ nguồn motor ngắt.

## Pin map

| GPIO | Nối tới | Hàng trong ảnh | Cách nối |
|---|---|---|---|
| 1 | IR D1 | Phải | Qua chia áp IR |
| 2 | IR D2 | Phải | Qua chia áp IR |
| 4 | IR D3 | Trái | Qua chia áp IR |
| 5 | IR D4 | Trái | Qua chia áp IR |
| 6 | IR D5 | Trái | Qua chia áp IR |
| 7 | IR D6 | Trái | Qua chia áp IR |
| 8 | IR D7 | Trái | Qua chia áp IR |
| 9 | IR D8 | Trái | Qua chia áp IR |
| 10 | Điện áp pin | Trái | Chân S của module voltage sensor; bắt buộc theo ADR-021 |
| 11 | DRV8871 trái IN1 | Trái | Logic 3,3 V / PWM |
| 12 | DRV8871 trái IN2 | Trái | Logic 3,3 V / PWM |
| 13 | DRV8871 phải IN1 | Trái | Logic 3,3 V / PWM |
| 14 | DRV8871 phải IN2 | Trái | Logic 3,3 V / PWM |
| 15 | SDA chung ToF + IMU | Trái | I2C, không kéo lên 5 V |
| 16 | SCL chung ToF + IMU | Trái | I2C, không kéo lên 5 V |
| 17 | Encoder trái A / C1 | Trái | Input logic 3,3 V |
| 18 | Encoder trái B / C2 | Trái | Input logic 3,3 V |
| 21 | Encoder phải A / C1 | Phải | Input logic 3,3 V; không còn là LED_EN |
| 47 | Encoder phải B / C2 | Phải | Input logic 3,3 V |
| 40 | Nút Start / Hủy | Phải | Nút nối xuống GND, input pull-up |
| 41 | ToF SHUT / XSHUT | Phải | Open-drain: kéo thấp reset, nhả để chạy |
| 39 | LED trạng thái (dự phòng) | Phải | Chỉ dùng nếu motor phát âm không đạt; khi đó không dùng IMU INT |
| 42 | ToF INT | Phải | Dự phòng; ban đầu không nối |

Hai chân 3V3 nằm trên hàng trái; 5V gần cuối hàng trái. RST là reset, không phải GPIO. Các nhãn ADC/SPI/UART trên ảnh là chức năng có thể chọn: GPIO15/16 vẫn làm I2C, GPIO17/18 làm encoder và GPIO11–14 làm PWM được.

Không dùng GPIO0/3/45/46 (strapping), GPIO19/20 (USB), GPIO26–37 (vùng bộ nhớ; đặc biệt 35–37 với Octal PSRAM), GPIO43/44 (giữ UART), GPIO38/48 (tránh LED RGB khác revision). Không chọn GPIO3 làm kênh IR dù nó có ADC.

## IR 8 mắt và chia áp ADC

D1–D8 của thanh được chọn là **analog**, không phải digital. Cấp VCC=3V3, GND chung. Định nghĩa thứ tự kênh robot từ trái sang phải theo hướng chạy; gá đảo thì firmware đảo thứ tự, không suy trái/phải từ ảnh nhìn mặt dưới thanh.

Từng kênh nối độc lập:

```text
IR Dn -> 10kΩ ->+-> GPIO ADC trong bảng
                |
               47kΩ
                |
               GND
```

Cần tám trở 10kΩ và tám trở 47kΩ, ưu tiên 1%. Tỷ lệ danh định `47/57` đưa 3,3 V xuống khoảng 2,72 V để có headroom so với vùng ADC S3 được đặc tả tới khoảng 2,9 V ở attenuation lớn nhất. Đây là phép tính lý tưởng, không phải đo output impedance hoặc sai số toàn mạch. Chia áp này không phải bảo vệ đấu nhầm 5 V/3S.

Có thể thêm 10 nF từ mỗi ADC xuống GND gần ESP32; chưa coi RC/settling/harness là đã kiểm chứng. Không áp dụng lịch LED off/on của PCB TCRT5000 cũ: module mua sẵn chưa có contract điều khiển emitter tương đương. Hiệu chuẩn đen/trắng, polarity, acquisition timestamp và phát hiện mẫu lỗi vẫn là phần firmware phải triển khai. Không dùng geometry cong IR-03 hoặc calibration sim làm kích thước thanh thật.

## Hai DRV8871

Mỗi module: VM/nguồn (+) vào 3S sau công tắc; GND/nguồn (-) về GND nguồn; OUT1/OUT2 hoặc hai chân MOTOR vào hai dây cuộn motor tương ứng. IN1/IN2 theo bảng GPIO. Đảo chiều từng bánh bằng cấu hình firmware khi cần, không đổi pin map.

DRV8871 làm việc 6,5–45 V, nhận logic 3,3 V; 3,6 A là rating **đỉnh**, không phải dòng liên tục của mọi module. Điện trở ILIM trên module quyết định current regulation; không kế thừa mức khoảng 1 A, shunt, nhiệt hoặc giới hạn copper từ mạch A4950. Tụ cục bộ, dây công suất, bảo vệ pin và ngắt motor cần phù hợp hệ thống. Pin map không xác nhận mức ILIM hoặc khả năng bảo vệ motor đã đạt.

## ToF400C VL53L1X

| Chân module | Nối tới |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SDA | GPIO15 |
| SCL | GPIO16 |
| SHUT / XSHUT | GPIO41, output open-drain; dùng pull-up của module |
| INT | Để trống ban đầu; GPIO42 được dành nếu triển khai ngắt |

Địa chỉ I2C 7-bit mặc định `0x29`. Không push-pull XSHUT lên 5 V hoặc thêm pull-up 5 V. Không dùng trình tự/thanh ghi VL53L0X trong adapter cũ cho VL53L1X.

## GY-9250 dùng I2C

| Chân module | Nối tới |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA / SDI | GPIO15 |
| SCL / SCLK | GPIO16 |
| AD0 / SDO | GND, chọn địa chỉ `0x68` |
| NCS | 3V3, chọn I2C |
| FSYNC | GND |
| EDA, ECL | Để trống |
| INT | Để trống ban đầu; GPIO39 được dành nếu triển khai ngắt |

Dùng pull-up bus phù hợp 3,3 V, không chồng thêm tùy tiện lên pull-up có sẵn. ToF và IMU không trùng địa chỉ. Có module IMU không có nghĩa core hiện đọc gyro: `SensorMeasurements` chưa có kênh IMU; cần producer/consumer và tests trước khi sử dụng trong quyết định robot. Không tạo mẫu zero hợp lệ để lấp phần chưa triển khai.

## Nút và đo pin

Nút nối GPIO40 với GND; input pull-up. Hành vi theo [ARCHITECTURE](../docs/ARCHITECTURE.md#2-vận-hành-bằng-một-nút): nhấn ngắn là Start (khi nhả tay), nhấn giữ là tự hiệu chuẩn, nhấn khi đang chạy là Hủy. Đích do robot tự nhận bằng thanh IR, không có cảm biến đích. Nút **không** thay công tắc nguồn.

Đo pin bằng module voltage sensor đã mua ([icdayroi](https://icdayroi.com/cam-bien-voltage-sensor)), thay mạch 180k/22k trước đây:

```text
3S (+) sau công tắc -> VCC (đầu vào) module
3S (-) / GND chung  -> GND (đầu vào) module
module S            -> GPIO10   (+ 100 nF từ GPIO10 xuống GND, sát ESP32)
module -            -> GND
module +            -> không nối (chân trống trên phần lớn module loại này)
```

Shop ghi tối đa 25 V đầu vào, tức tỉ lệ khoảng 1:5: 12,6 V cho khoảng 2,5 V tại ADC, trong dải ADC S3 ở attenuation lớn nhất. **Tỉ lệ thật phải đo một lần** bằng đồng hồ vạn năng (áp pin chia áp đo tại S) và lưu trong profile robot; không dùng con số của shop. Kiểm tra bằng đồng hồ rằng chân S không vượt 3,3 V trước khi nối GPIO10. Trở chia của module khá lớn, nên giữ tụ 100 nF. Đo tổng pack không thay bảo vệ từng cell, sạc cân bằng hoặc ngắt quá xả.

## Nguồn đối chiếu

- [ESP32-S3 GPIO và hạn chế bộ nhớ/strapping/USB](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html), [ADC và hardware checklist](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html).
- [Thanh IR đã chọn](https://www.thegioiic.com/mach-cam-bien-do-line-8-kenh-khoang-cach-2-10mm): analog, mô tả nguồn 3,3–5 V; phần tóm tắt shop có dải khác, không dùng dải đó để cấp quá 3,3 V trong sơ đồ này.
- [DRV8871 TI](https://www.ti.com/lit/ds/symlink/drv8871.pdf), [module Đức Huy](https://dientuduchuy.com/products/drv8871-3-6a-dc-motor-driver-mach-cau-h-pwm-dieu-khien-toc-do-dong-co-dc).
- [ToF400C Hshop](https://hshop.vn/cam-bien-khoang-cach-tof400c-vl53l1x-tof-laser-distance-sensor), [GY-9250 LKCG](https://lkcg.vn/cam-bien-gia-toc-9-truc-gy-9250-9dof-imu-mpu9250), [motor LKCG](https://lkcg.vn/dong-co-dc-giam-toc-servo-ga12-n20-12v-300rpm).

Nguồn shop và ảnh pinout không thay xác nhận pinout encoder đúng lô, mức ILIM, nguồn/timing/nhiệt hoặc tính hợp lệ thi đấu. Tài liệu này chốt cách phân bổ GPIO; không tuyên bố robot đã chạy.
