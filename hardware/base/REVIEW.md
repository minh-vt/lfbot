# Review thiết kế BASE v0.7

Ngày 2026-11-18. Phạm vi: [base.kicad_sch](base.kicad_sch) và [base.kicad_pcb](base.kicad_pcb) bản 0.7. Công cụ: các analyzer của kicad-happy, KiCad 10 DRC/ERC, ngspice, KiCadRoutingTools.

## Kết luận

**Về điện và layout: không còn lỗi chặn.**
- ERC 0 lỗi.
- DRC 0 lỗi, 0 cảnh báo, 0 chân hở, khớp schematic.
- SPICE: 10/10 phép thử đạt.
- 40 via (27 via tín hiệu, 13 via khâu).

**Chưa sẵn sàng đặt in** vì 5 việc phải làm trước. Đều là kiểm trên đồ thật, không cần sửa board:

| # | Việc | Vì sao chặn |
|---|---|---|
| 1 | Thử gá trên bản in 1:1 (devkit, DRV, gá motor, thanh IR, bi cầu, buck) | Thân module ước từ ảnh; lỗ lệch là board bỏ đi |
| 2 | Đo thông mạch dây ZH của motor: chân 1 ở đầu motor ra chân nào ở đầu kia | Dây JST bán sẵn có loại nối thẳng và loại đảo. Nếu đảo, M1/M2 của motor rơi vào VCC/GND encoder |
| 3 | Xác nhận vỏ bi cầu ≤ Ø23 mm | Bi cầu nằm giữa hai hàng chân ESP32, mỗi bên chỉ hở khoảng 0,35 mm |
| 4 | Chọn vị trí jumper J4 (chân IR của thanh) | Không cắm jumper thì chân này để trống; chưa biết thanh xử lý trạng thái đó ra sao |
| 5 | Ước dòng 3V3 (8 LED IR của thanh + ESP32 + ToF + IMU + 2 encoder) | Ổn áp trên devkit gánh toàn bộ; dòng LED IR của thanh chưa có số |

Ngoài board: `firmware/src/esp32/pins.hpp` và [wiring.md](../wiring.md) chưa theo bảng GPIO mới. Firmware cũng vẫn coi mắt IR cách trục 45 mm, trong khi trên board là khoảng 90 mm.

## Thay đổi so với v0.6 và lý do

v0.6 qua DRC sạch, nhưng có một lỗi thiết kế mà DRC không bắt được. DRV8871 lấy nguồn VM qua header 4 chân, và header đó quay về phía ESP32; cụm nguồn vào lại nằm ở góc trước. Kết quả là dòng motor chạy từ đầu board, xuyên qua vùng ESP32 và vùng analog, ra tới motor ở cuối, rồi quay về theo lớp GND cũng qua đúng vùng đó. Chỉ đổi cách chia lớp không sửa được, bảng dưới cho thấy điều này.

**Sửa ở gốc:**
1. **Footprint DRV** có thêm pad V+ / V− cạnh M1/M2, nối từ cầu đấu POWER của module. Cầu đấu này nằm cùng cạnh với cầu đấu MOTOR, tức là sát motor. Chân VM của header bỏ trống; header chỉ còn IN1, IN2, GND.
2. **Cụm nguồn vào (J1, F1, J2, C1)** khoá dọc mép sau. Dòng motor giờ đi pin → cầu chì → công tắc → V+ của DRV → motor, khoảng 30 mm, toàn bộ nằm ở cuối board.
3. **Placement** đặt lại bằng placer của KRT với 4 nhóm chức năng (bảng thử 12 phương án ở dưới). Phương án được chọn route nhanh ra 20 via.
4. **Chia lớp** chọn bằng số liệu, xem bảng dưới.
5. **Via khâu** giảm từ 8 mm xuống 16 mm một via (13 via thay vì 64).
6. **Sửa footprint khay pin:** trước đây đường viền khay nằm trên lớp F.Fab, nên KRT tưởng cả vùng giữa board đã bị chiếm; giờ chuyển sang Dwgs.User.
7. **Via dưới thân motor:** board bật phủ mặt nạ hàn (tenting) cho via ở cả hai mặt, và không via nào ghi đè thiết lập này (đã đọc thẳng file), nên vỏ motor tì lên không chạm đồng.

**So 4 cách chia lớp.** Cùng một placement; GND đổ hai mặt trước rồi mới route.

| Placement | Cách chia lớp | Via tín hiệu | Dây mặt dưới | Đồng GND trên / dưới | Tín hiệu qua chỗ hở GND (EMC) |
|---|---|---|---|---|---|
| v0.6 | A: KRT mặc định | 26 | 266 mm | 2 mảng / 1 mảng | 7 lỗi, 84 mm |
| v0.6 | B: ép mặt dưới làm mặt phẳng GND | 51 | 122 mm | 3 / 1 | 7 lỗi, 60 mm |
| v0.6 | C: nguồn ở mặt dưới, tín hiệu ở mặt trên | 30 | 583 mm | 2 / 1 | 7 lỗi, 75 mm |
| v0.6 | D: chỉ một lớp | thiếu 5 net | 0 | 20 / 1 | — |
| Ba vùng (v0.7) | A | 32 | 244 mm | 1 / 1 | 12 lỗi, 97 mm |
| Ba vùng (v0.7) | B | 49 | 123 mm | 1 / 1 | 10 lỗi, 77 mm |
| **Ba vùng (v0.7)** | **C (đã chọn)** | **27** | 652 mm | **1 / 1** | 12 lỗi, 79 mm |

Chọn C vì:
- ít via nhất;
- mức EMC ngang cách tốt nhất (B);
- vùng đồng GND mỗi mặt đều liền một mảng.

Lỗi EMC ở bản ba vùng nhiều hơn bản v0.6 nhưng chỉ là heuristic: đó là các đoạn tín hiệu chậm đi qua khe giữa các đường nguồn ở mặt dưới. Bảng này không đo vòng dòng motor. Thay đổi quan trọng nhất (vòng dòng motor gói gọn ở cuối board) đọc được từ bố trí, không có trong bảng.

## Các analyzer đã chạy

| Analyzer | Kết quả | Ghi chú |
|---|---|---|
| ERC (kicad-cli) | 0 lỗi | |
| DRC + parity (kicad-cli, đổ lại vùng đồng) | 0 lỗi / 0 cảnh báo / 0 chân hở / 0 lệch | Luật gốc của project, khoảng hở 0,2 mm |
| `analyze_schematic.py` | 2 lỗi, 4 cảnh báo | Đều là cảnh báo giả, xem bên dưới |
| `analyze_pcb.py --full --proximity` | 2 lỗi, 8 cảnh báo | Đều là cảnh báo giả, xem bên dưới |
| `cross_analysis.py` | 0 | Schematic và PCB đồng bộ |
| `analyze_emc.py` | 12 lỗi, 24 cảnh báo | Đều là heuristic; đã lọc |
| `analyze_thermal.py` | 0 | Không có ổn áp hay shunt trên board |
| `analyze_gerbers.py` (Gerber xuất tạm) | 1 cảnh báo | Đủ 9 lớp, có khoan PTH và NPTH, board 73,1 × 116,9 mm |
| `simulate_subcircuits.py` (ngspice) | 10/10 đạt | 9 bộ chia áp và tụ C1 |

## Đối chiếu chân

Loại bằng chứng: **datasheet** (PDF của hãng, có số trang), **ảnh** (ảnh module của chủ dự án), **đo** (số chủ dự án đo), **file** (đọc thẳng file `.kicad_pcb`), **suy luận**.

| Linh kiện | Kiểm gì | Bằng chứng | Kết quả |
|---|---|---|---|
| U1 devkit TDM2533 | 44 chân khớp nhãn in trên board; hai hàng cách 25,4 mm | ảnh + đo + file | Khớp |
| U2/U3 DRV8871 | Header IN2, IN1, VM, GND theo chữ in trên module; cầu đấu MOTOR rồi POWER (+, −) ở cạnh đối diện | ảnh shop + ảnh chủ dự án | Khớp, với điều kiện module cắm mặt linh kiện lên trên. VM của header không nối; V+/V− đi dây từ cầu đấu POWER |
| DRV8871 điện | VM 6,5–45 V (tr. 5), pin 3S tối đa 12,6 V; PWM ≤ 200 kHz (tr. 4), board dùng 20 kHz; IN có điện trở kéo xuống 100 kΩ (tr. 5) nên lúc khởi động motor thả trôi; tụ 0,1 µF + tụ lớn (tr. 3, 7) có sẵn trên module | datasheet TI SLVSCY9B | Đạt |
| J7/J8 giắc encoder ZH | Thứ tự M1, GND, C1, C2, VCC, M2; bước 1,5 mm | chủ dự án nêu + đo | Khớp với chân 1 trên board. **Chưa biết dây nối thẳng hay đảo** (việc chặn số 2) |
| J5 ToF400C | VIN, GND, SDA, SCL, INT, SHUT | ảnh | Khớp. VIN ở bên trái xe nên mặt in chữ quay ra sau; trong ảnh, mặt in chữ không có chip VL53L1X, vậy cảm biến nhìn ra trước (**suy luận từ ảnh**; cắm thử để xác nhận) |
| J6 GY-9250 | VCC…FSYNC; ADO→GND (địa chỉ 0x68), NCS→3V3 (chọn I2C), FSYNC→GND | ảnh + wiring.md | Khớp; nhìn từ trên khi module nằm phẳng thì không bị lật gương |
| U4 MP1584 | IN+ trên trái, IN− trên phải, OUT+ dưới trái, OUT− dưới phải | ảnh có thước | Khớp; vị trí pad sai khoảng ±0,3 mm, bù bằng lỗ 1,3 mm |
| J3 thanh IR | GND, IR, D1…D8, VCC; D1 ở bên trái xe | ảnh + wiring.md | Khớp nếu **nối dây theo tên chân**; không dùng header thẳng một-một |
| D1 SS14, C1 | Cathode nối +5V; cực + của C1 nối VBAT_SW | file | Đúng chiều |
| GPIO3 (IR D3) | Chân strapping chỉ chọn nguồn JTAG, bị bỏ qua khi EFUSE_STRAP_JTAG_SEL = 0 (mặc định) | datasheet ESP32-S3 v2.2 tr. 34–35 | Dùng làm đầu vào analog được |
| GPIO39–42 | Là chân JTAG phần cứng, nhưng mặc định JTAG đi qua USB | datasheet tr. 26, 35 | Dùng làm GPIO được (suy luận từ eFuse mặc định) |
| Chia áp IR | 3,0 V × 47/57 = 2,47 V, trong vùng tuyến tính 0–2,9 V (datasheet tr. 66, bảng 5-5) | datasheet + SPICE | Đạt |
| Chia áp đo pin | 12,6 V × 10/57 = 2,21 V | SPICE + tính tay | Đạt |

## Nguồn

```text
Vùng công suất (cuối board):
BAT 3S (J1) → F1 PTC → J2 công tắc → VBAT_SW ┬→ C1 220 µF
                                              ├→ V+ của U2/U3 (cầu đấu POWER) → motor (cầu đấu MOTOR → M1/M2 → J7/J8)
                                              ├→ U4 MP1584 (dưới ESP32, khoảng 0,2 A) → SS14 → +5V → devkit 5Vin
                                              └→ R1/R2 → VBAT_SENSE (GPIO1)
devkit 3V3 → thanh IR, ToF, IMU, 2 encoder
```

Khả năng chịu dòng theo IPC-2221 (lớp ngoài, đồng 1 oz, nóng thêm 10 °C):

| Đường | Độ rộng | Chịu được |
|---|---|---|
| VBAT, VBAT_F, motor | 0,8 mm | 2,03 A |
| VBAT_SW | 0,6 mm | 1,65 A |
| +5V, +3V3 | 0,5 mm | 1,45 A |
| GND | đồng đổ hai mặt | không có dây GND riêng |

**VBAT_SW có đoạn bị thu hẹp xuống 0,3 mm (khoảng 0,88 A):**
- **3,6 mm trên đường cấp cho DRV trái,** chỗ dây lách qua pad V− của DRV phải. Đoạn ngắn này toả nhiệt sang phần đồng rộng hai đầu. Dòng kẹt khoảng 0,67 A mỗi motor (datasheet GA12-N20 bản chung, không phải đúng bản 1000 RPM có encoder; **suy luận**).
- **Các đoạn còn lại** nằm trên nhánh tới buck và chia áp, chỉ mang khoảng 0,2 A và dòng µA.

Đầu nối ZH chịu 1 A mỗi chân.

## Lọc cảnh báo giả

| Analyzer báo | Đánh giá |
|---|---|
| DS-001, SS-001: thiếu datasheet và MPN | Board hàn tay bằng linh kiện mua lẻ, không lắp máy. Review chỉ đối chiếu được datasheet cho DRV8871 và ESP32-S3; các phần còn lại là kiểm tính nhất quán |
| RS-001: VBAT, VBAT_F, VBAT_SW, +5V_BUCK không có nguồn khai báo | Nguồn vào qua pad hàn dây và module buck; ERC vẫn sạch |
| PM-002: H4/H5 lòi khỏi mép board 0,75 mm | Cố ý: hộp số motor chìa ra phía bánh |
| PM-002: C1, F1, J1, J2, J5, J6, SW2 gần mép board | Khoảng cách đồng–mép board vẫn đạt 0,5 mm theo DRC; cụm nguồn và ToF cố ý đặt ở mép |
| TE-001: không có test point | Board hàn tay; đo trực tiếp trên chân header |
| GP-001 (EMC): tín hiệu đi qua chỗ hở lớp GND | Analyzer tự xếp là heuristic; tín hiệu chậm (PWM 20 kHz, I2C, ADC), mỗi đoạn vài mm |
| RP-001 (EMC): thiếu via khâu tại chỗ đổi lớp | Tín hiệu tốc độ thấp; có 13 via khâu, mỗi via cách nhau 16 mm |
| GR-004: lớp paste chỉ có một phần pad | Đúng như mong đợi: phần lớn là linh kiện xuyên lỗ; paste chỉ cho SMD |

## Chưa làm và giới hạn

- **Thiếu datasheet cho phần lớn linh kiện.** Chỉ đối chiếu datasheet thật cho DRV8871 và ESP32-S3. Thứ tự chân của ToF, IMU, buck, thanh IR và giắc encoder dựa vào ảnh và lời chủ dự án.
- **Không kiểm vòng đời linh kiện:** không có MPN.
- **Chưa xét thời gian lấy mẫu ADC với nguồn trở kháng khoảng 8,2 kΩ** (10k song song 47k) ở tần số quét hiện tại.
- **Nhiệt:** không có ổn áp trên board; ổn áp 3V3 nằm trên devkit, ngoài phạm vi review. DRV8871 tiêu tán I²·R với R khoảng 0,565 Ω, không đáng kể ở dòng chạy thường.
- **Analyzer không đo vòng dòng motor.** Việc dòng motor gói gọn ở cuối board được kiểm bằng cách xem đường mạch và vị trí pad, không phải bằng một chỉ số tự động.
