# BASE: PCB làm khung xe

PCB hai lớp 73 × 117 mm vừa là khung xe vừa là mạch nối các module. Board chia ba vùng. **Vùng công suất** ở cuối board: motor, nguồn vào, cầu đấu POWER và MOTOR của hai DRV8871, giắc encoder. **Vùng logic** ở giữa: ESP32 cắm trên header cái; dưới gầm nó có bi cầu, IMU, buck và chia áp đo pin. **Vùng analog** ở đầu board: thanh IR, chia áp IR, ToF. Dòng motor chỉ quay vòng trong vùng công suất. Pin 3×1S đặt trên tầng in 3D có sẵn, bắt vào bốn lỗ M3 cách nhau 50 × 35 mm (mẫu lỗ của [base.dxf](mechanical/base.dxf)). Lý do chọn hướng này ghi ở [ADR-025](../../docs/decisions/ADR-025-pcb-base-carrying-modules.md).

**Trạng thái: bản 0.7, đã route, chưa đặt in.**
- **DRC** (luật siết theo năng lực JLCPCB, có đổ lại vùng đồng): 0 lỗi, 0 cảnh báo, 0 chân hở, khớp schematic. Gerber và file khoan đạt ngưỡng xưởng.
- **Kích thước:** khoảng hở tối thiểu 0,2 mm; via 0,8/0,4 mm; không có via nằm trong pad SMD.
- **Via:** 36 (23 via tín hiệu và nguồn, 13 via khâu nối hai vùng GND).
- **Chia lớp:** GND đổ đồng cả hai mặt; nguồn và motor đi ở mặt dưới, tín hiệu ở mặt trên.
- **Review đầy đủ:** [REVIEW.md](REVIEW.md).
- **Chưa làm:** chưa thử gá trên bản in 1:1, chưa có board thật.

## Mở, đặt linh kiện và route

Mở [base.kicad_pro](base.kicad_pro) bằng KiCad 10. Cần thư viện Espressif cài qua Plugin and Content Manager (tên `PCM_Espressif`).

- **Schematic và footprint riêng** ([Base.pretty](Base.pretty/)) sinh từ [gen/design.py](gen/design.py) và [gen/gen_fp.py](gen/gen_fp.py) qua [gen/run_all.sh](gen/run_all.sh); script không đụng tới board. Sau khi đổi chân, cập nhật pad của board bằng Update PCB from Schematic rồi chạy DRC có kiểm khớp schematic.
- **Placement:**
  - Khoá tay 20 linh kiện mà vị trí là quyết định thiết kế: lỗ, gá motor, ESP32, hai DRV, ToF, header IR, các linh kiện dưới gầm ESP32, và cụm nguồn vào (J1, F1, J2, C1) dọc mép sau.
  - Phần còn lại do placer của **KiCadRoutingTools** (plugin PCM `com.github.drandyhaas.kicadroutingtools`) đặt:
    - 4 nhóm chức năng;
    - vùng cấm dưới thân ESP32 và hai DRV;
    - vùng khay pin chỉ cấm nút bấm;
    - thử 12 phương án, chọn phương án 2 vì route nhanh ra ít via nhất (20).
  - **Vùng analog đặt tay** (sau khi đo thấy placer để dây đi vòng): 16 điện trở chia áp IR xếp thành 8 cột ngay sau header J3, mỗi cột thẳng sau chân IR của nó (10k ở hàng trước, 47k ở hàng sau). Dây ADC đi thẳng về ESP32, còn XSHUT, SCL, SDA đi thẳng qua khe giữa các chân J3 tới ToF. J4 dời sang trái các cột, J10 sang phải.
- **Route:** bản 0.7 được route bằng router của **KiCadRoutingTools**:
  - đổ GND ở F.Cu và B.Cu trước;
  - route nguồn và motor ưu tiên B.Cu, tín hiệu ưu tiên F.Cu;
  - khâu via nối hai vùng GND, cách nhau 16 mm;
  - giữ via cách pad SMD 0,2 mm để hàn tay;
  - không bao giờ xuống dưới luật của board, và không sửa file `.kicad_pro`.

  Sau đó sửa tay và route lại vùng trước (REVIEW.md). Script route cả board đã bỏ vì nó xoá các sửa đó; nó còn trong lịch sử git.
- **Route lại vài net:** [gen/route.sh](gen/route.sh) gỡ đồng của các net được nêu ([gen/strip_copper.py](gen/strip_copper.py)), route lại chúng theo từng lượt và theo thứ tự cho trước, giữ nguyên mọi đồng khác, rồi chạy `tidy_tracks.py` và DRC. Đầu file có ví dụ đúng cách vùng trước của bản 0.7 đã được route.
- **Vì sao chọn cách chia lớp này:** so 4 cách trên cùng placement, số liệu trong REVIEW.md.
- **Nhãn silkscreen:** sắp bằng `beautify_labels.py` của KRT sau khi route.
- **Làm gọn đường mạch:** [gen/tidy_tracks.py](gen/tidy_tracks.py) gộp khúc thẳng hàng, bỏ khúc gãy, nắn về 0/45/90° và vát góc 90°. Nó tự chạy DRC và lùi lại chỗ nào dính lỗi. Chạy được nhiều lần; cần file `.kicad_pro` nằm cạnh board.
- **File KiCad là bản gốc** của layout; sửa trong KiCad. `route.sh` chỉ thay đồng của các net được nêu, nên sửa tay trên các net đó (liệt kê ở các mục Kiểm gia công và Hình học đường mạch của REVIEW.md) sẽ mất.

## Gán chân GPIO

Chân được gán theo vị trí trên board để dây đi thẳng, không bắt chéo. Hàng chân phía trước của ESP32 (quay về thanh IR) mang IR, I2C và nút bấm. Hàng phía sau (quay về DRV và motor) mang PWM, encoder và đo pin.

| Chức năng | GPIO |
|---|---|
| IR D1…D8 (trái → phải, như trên thanh) | 10, 9, 3, 8, 7, 6, 5, 4 |
| I2C SDA / SCL | 17 / 16 |
| ToF XSHUT, IR_CTRL (jumper J4) | 15, 18 |
| Nút START, nút 2, IMU INT | 11, 12, 13 |
| Motor trái IN1 / IN2, phải IN1 / IN2 | 38 / 39, 41 / 42 |
| Encoder trái A / B, phải A / B | 21 / 47, 40 / 2 |
| Đo pin (47k/10k, tỉ lệ 5,7) | 1 |

GPIO3 là chân strapping, nhưng chỉ dùng làm đầu vào analog. Cách đấu từng module và chân để trống có trong [wiring.md](../wiring.md). `firmware/src/esp32/pins.hpp` chưa cập nhật theo bảng này.

## Những gì thiết kế tự hấp thụ

| Chưa chắc | Cách xử lý |
|---|---|
| Lỗ thanh IR lệch bao nhiêu so với hàng mắt | Khe M3 ±3 mm theo trục dọc xe |
| Vị trí và chiều cao bi cầu | Khe M2 ±3 mm; kê long đền dưới bi nếu xe chúi |
| Chiều cao thanh IR so với sàn | Chiều dài cột M3 và long đền |
| Cắm USB khi buck đang cấp nguồn | Diode SS14 từ buck vào 5Vin; chỉnh buck lên khoảng 5,2–5,3 V để bù sụt áp |
| Tỉ lệ chia áp IR | Mỗi kênh 10k + 47k SMD 0805 (3,0 V → 2,47 V); thay điện trở để đổi tỉ lệ |
| Chân IR của thanh (bật LED?) | Jumper J4: IR_EN nối 3V3, GPIO18 hoặc GND |
| Dòng cầu chì | Footprint PTC chân hướng kính, chọn giá trị khi lắp |
| Pad của module buck | Lỗ khoan 1,3 mm, hàn bằng chân điện trở cắt thừa (vị trí pad đo từ ảnh, ±0,3 mm) |

## Còn là ước lượng

Các mục sau chỉ quyết định khoảng hở, không ảnh hưởng mạch điện:

- **Thân devkit TDM2533:** khoảng cách hai hàng chân 25,4 mm do chủ dự án đo; kích thước thân ước từ ảnh.
- **Thân DRV8871:** 21 × 21 mm ước từ ảnh shop.
- **Thanh IR:** 66 × 17 mm và khoảng cách từ lỗ tới mắt ước từ ảnh.
- **Bi cầu:** coi là Ø23 mm; nó nằm vừa giữa hai hàng chân ESP32 (tâm mỗi hàng cách tâm bi 12,7 mm).
- **Motor + encoder:** coi dài 33 mm. Cả dải ±7,5 mm quanh trục để trống, nên encoder dài hơn cũng chỉ lấn vào phần board trống.

Trước khi đặt in, in [fitcheck/fitcheck_top_1to1.pdf](fitcheck/fitcheck_top_1to1.pdf) và [fitcheck_bottom_1to1.pdf](fitcheck/fitcheck_bottom_1to1.pdf) đúng 100% (không "fit to page"), rồi đặt devkit, DRV, gá motor, thanh IR, bi cầu và buck lên giấy để kiểm. Có [STEP](fitcheck/base_v0.7.step) để ghép trong CAD.

## Lắp ráp

- **Không có chống cắm ngược pin.** Dùng giắc có cực (XT30) ở dây pin và kiểm cực theo chữ BAT+ / GND trên board.
- **Pad pin, cầu chì, tụ lớn và pad công tắc** nằm dọc mép sau, dưới mép khay pin.
- **Mỗi DRV8871 có 4 dây ngắn ở cạnh cầu đấu:**
  - từ cầu đấu MOTOR vào pad M1/M2;
  - từ cầu đấu POWER vào pad V+ / V− (**đúng cực theo chữ trên board**).
  - Header 4 chân chỉ mang IN1, IN2 và GND; chân VM của header không nối.
- **Header IR (J3)** nằm trên hàng chân của thanh IR, nối bằng dây ngắn từ mặt dưới. Chữ trên board ghi thứ tự GND, IR, D1…D8, VCC.
- **Điện trở chia áp IR:** silkscreen không ghi tên từng con, chỉ ghi nhãn hàng "10k" và "47k" bên phải. Cột trái nhất là kênh D1 (R10, R20), cột phải nhất là D8 (R17, R27). Tên từng con có trên bản in fitcheck (lớp Fab).
- **IMU (GY-9250):** hàn thẳng header của module xuống board để module nằm phẳng dưới gầm ESP32, gyro Z thẳng đứng.
- **Rút ESP32 ra** khi cần vặn vít bi cầu hoặc chỉnh biến trở 5 V của buck.
- **ToF:** cắm thẳng đứng vào J5 ở mép trước, mặt cảm biến hướng ra trước.

## Linh kiện thêm so với cấu hình module

- **Header và giắc:**
  - header cái: 1×22 ×2 (devkit), 1×4 ×2 (DRV), 1×6 (ToF);
  - giắc JST ZH 1,5 mm 6 chân đứng ×2;
  - header đực: 1×10 (IMU), 1×11, 2×3 kèm jumper, 1×5, 1×3.
- **Thụ động SMD 0805:**
  - điện trở: 8 × 10k và 8 × 47k (chia áp IR), 47k và 10k 1% (đo pin);
  - tụ 100 nF.
- **Khác:**
  - tụ hoá 220 µF 25 V;
  - diode SS14;
  - PTC chân hướng kính;
  - nút 6 mm ×2;
  - vít và cột M3, M2.
- **Không cần** module voltage sensor.
