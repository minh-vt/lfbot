# Review thiết kế BASE v0.7

Ngày 2026-11-18. Phạm vi: [base.kicad_sch](base.kicad_sch) và [base.kicad_pcb](base.kicad_pcb) bản 0.7. Công cụ: các analyzer của kicad-happy, KiCad 10 DRC/ERC, ngspice, KiCadRoutingTools.

## Kết luận

**Về điện, layout và gia công: không còn lỗi chặn.**
- ERC 0 lỗi.
- DRC 0 lỗi, 0 cảnh báo, 0 chân hở, khớp schematic, chấm theo bộ luật đã siết theo năng lực JLCPCB (mục [Kiểm gia công](#kiểm-gia-công)).
- Gerber và file khoan xuất đủ lớp; mọi lỗ, vành đồng và khe đạt ngưỡng của xưởng.
- SPICE: 10/10 phép thử đạt.
- 36 via (23 via tín hiệu và nguồn, 13 via khâu).

**Chưa nên đặt in** vì 5 việc phải làm trước. Đều là kiểm trên đồ thật, không cần sửa board:

| # | Việc | Vì sao chặn |
|---|---|---|
| 1 | Thử gá trên bản in 1:1 (devkit, DRV, gá motor, thanh IR, bi cầu, buck) | Thân module ước từ ảnh; lỗ lệch là board bỏ đi |
| 2 | Đo thông mạch dây ZH của motor: chân 1 ở đầu motor ra chân nào ở đầu kia | Dây JST bán sẵn có loại nối thẳng và loại đảo. Nếu đảo, M1/M2 của motor rơi vào VCC/GND encoder |
| 3 | Xác nhận vỏ bi cầu ≤ Ø23 mm | Bi cầu nằm giữa hai hàng chân ESP32, mỗi bên chỉ hở khoảng 0,35 mm |
| 4 | Chọn vị trí jumper J4 (chân IR của thanh) | Không cắm jumper thì chân này để trống; chưa biết thanh xử lý trạng thái đó ra sao |
| 5 | Ước dòng 3V3 (8 LED IR của thanh + ESP32 + ToF + IMU + 2 encoder) | Ổn áp trên devkit gánh toàn bộ; dòng LED IR của thanh chưa có số |

Ngoài board: [wiring.md](../wiring.md) đã theo bảng GPIO mới, nhưng `firmware/src/esp32/pins.hpp` thì chưa. Firmware cũng vẫn coi mắt IR cách trục 45 mm, trong khi trên board là khoảng 90 mm.

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
| ERC (kicad-cli, mọi mức) | 0 lỗi | |
| DRC + parity (kicad-cli, đổ lại vùng đồng) | 0 lỗi / 0 cảnh báo / 0 chân hở / 0 lệch | Luật đã siết, xem [Kiểm gia công](#kiểm-gia-công) |
| `analyze_schematic.py` | 1 lỗi, 4 cảnh báo | Đều là cảnh báo giả, xem bên dưới |
| `analyze_pcb.py --full --proximity` | 2 lỗi, 8 cảnh báo | Đều là cảnh báo giả, xem bên dưới |
| `cross_analysis.py` | 0 | Schematic và PCB đồng bộ |
| `analyze_emc.py` | 16 lỗi, 21 cảnh báo | Đều là heuristic; đã lọc (xem mục Đặt lại vùng analog) |
| `analyze_thermal.py` | 0 | Không có ổn áp hay shunt trên board |
| `analyze_gerbers.py --stage pre_fab` (Gerber xuất tạm) | 1 cảnh báo | Đủ 9 lớp, có khoan PTH và NPTH, board 73,1 × 116,9 mm |
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

Mọi đường nguồn và motor giữ đúng một độ rộng từ đầu tới cuối. Ngoại lệ duy nhất là nhánh VBAT_SW tới R1 (chia áp đo pin, dòng µA): nhánh này rộng 0,3 mm suốt chiều dài, vì nó lách giữa hai chân header của ESP32. Dòng kẹt khoảng 0,67 A mỗi motor (datasheet GA12-N20 bản chung, không phải đúng bản 1000 RPM có encoder; **suy luận**).

Đầu nối ZH chịu 1 A mỗi chân.

## Kiểm gia công

Bản DRC "0 lỗi" trước đây chấm theo bộ luật lỏng: một số kiểm tra bị tắt hoặc chỉ là cảnh báo, và vài ngưỡng thấp hơn năng lực xưởng. Lần kiểm này siết luật trong [base.kicad_pro](base.kicad_pro) rồi sửa mọi thứ lộ ra.

**Luật đã siết:**
- **Ngưỡng:** vành đồng via ≥ 0,125 mm; chữ silkscreen cao ≥ 0,8 mm, nét ≥ 0,15 mm; lỗ cách lỗ ≥ 0,25 mm; khoảng hở và dây ≥ 0,2 mm; đồng cách mép board ≥ 0,5 mm.
- **Nâng lên mức lỗi:** chữ quá nhỏ hoặc quá mảnh, lỗ trùng hoặc quá gần nhau, đồng cô lập, dây hoặc via treo, mảnh đồng hẹp, độ rộng chỗ nối, padstack.
- **Bật lại ở mức cảnh báo:** dây không vào đúng tâm via, loại footprint (SMD/xuyên lỗ) lệch với symbol.
- **Vẫn tắt:**
  - lọc footprint của symbol, vì 5 module (DRV, IMU, buck) cố ý dùng symbol header chung với footprint riêng;
  - hình học dây tuning, vì board không có đường cần chỉnh độ dài.

**Lỗi tìm thấy và đã sửa:**

| Lỗi | Hậu quả | Sửa |
|---|---|---|
| 23 chữ silkscreen nét 0,1 mm, 293 nét vẽ silkscreen mảnh | Xưởng in mờ hoặc mất chữ | Nâng lên 0,15 mm (sửa cả trong `gen_fp.py`) |
| Chữ "5Vin" đè lên pad 21 của ESP32 | Bị mặt nạ hàn cắt | Dời chữ |
| H2 (khe thanh IR) lệch thư viện | Footprint trên board khác bản trong Base.pretty | Thay bằng bản thư viện, giữ nguyên vị trí; khe không đổi |
| MOT_R_A chỉ chạm góc pad M1 của DRV phải | Chỗ nối dòng motor rộng chưa tới 0,4 mm | Kéo dây vào tâm pad |
| VBAT_SW tới DRV phải thắt còn 0,3 mm suốt 3,6 mm | Chịu khoảng 0,88 A, chưa tới dòng kẹt của hai motor cộng lại | Đi thấp hơn 0,1 mm, giữ 0,6 mm |
| VBAT_SW tới buck vòng khoảng 100 mm dưới ESP32, thắt ba chỗ (vít H1, giữa hai chân U1, giữa hai pad GND của buck) | Dây dài, cắt vùng GND mặt dưới | Đi thẳng dọc mép trái, 0,6 mm; đường cũ chỉ còn cấp cho R1 |
| 9 mối nối mà đầu dây trượt khỏi nhau hoặc khỏi tâm via 0,1–0,4 mm | Chỉ dính nhờ đồng chồng lên nhau; sửa trong KiCad dễ đứt | Gắn đúng điểm; một đoạn cụt nối lại vào pad R21 |

DRC không bắt được 4 lỗi cuối vì đồng vẫn chạm nhau. Chúng được tìm bằng script kiểm từng đầu dây: đầu dây phải trùng một đầu dây khác, trùng tâm via, hoặc nằm trọn trong pad.

**So với năng lực JLCPCB (2 lớp, chuẩn):**

| Hạng mục | Ngưỡng xưởng | Board |
|---|---|---|
| Dây và khoảng hở | 0,127 mm | Luật 0,2 mm; dây mảnh nhất 0,3 mm |
| Via | 0,45 / 0,2 mm | 0,8 / 0,4 mm, 36 via |
| Vành đồng PTH | 0,125 mm | Nhỏ nhất 0,15 mm (footprint JST ZH chuẩn của KiCad); các chân khác ≥ 0,35 mm |
| Lỗ nhỏ nhất | 0,2 mm | 0,4 mm (via), 0,73 mm (chân) |
| Khe NPTH | — | 4 khe phay (2 cho thanh IR, 2 cho bi cầu); hẹp nhất 2,2 mm |
| Chữ silkscreen | Cao 0,8 mm, nét 0,15 mm | Đạt, DRC kiểm |
| Đồng cách mép board | 0,3 mm (khuyên 0,5) | 0,5 mm, DRC kiểm |
| Lỗ cách mép board | — | Gần nhất 1,35 mm (F1) |
| Kích thước | — | 73,1 × 116,9 mm |

Vùng GND mỗi mặt vẫn liền một mảng sau khi sửa. Nếu dùng `gen/route.sh` route lại các net trong bảng trên, sửa tay trên chúng sẽ mất.

## Hình học đường mạch

Router để lại nhiều dấu vết của lưới 0,1 mm: đoạn thẳng bị cắt thành nhiều khúc, khúc gãy ngắn, góc 90°, đoạn lệch khỏi 0/45/90°. Không lỗi nào trong số này làm hỏng mạch, nhưng chúng làm board khó đọc và dễ đứt khi sửa tay trong KiCad.

**Cách làm:**
1. **Bộ làm mượt của KRT** (`awx/smooth_board.py`) cho 11 đoạn ngắn hơn. Nó kiểm khoảng hở theo ngưỡng nới (0,15 mm) nên tạo 7 lỗi khoảng hở; 5 net bị lỗi được trả về đồng cũ.
2. **[gen/tidy_tracks.py](gen/tidy_tracks.py)** làm phần còn lại:
   - chia dây thành từng đoạn chạy giữa các điểm neo (pad, via, chỗ rẽ nhánh, chỗ đổi độ rộng);
   - gộp các khúc thẳng hàng, gấp bỏ khúc gãy ngắn hơn 0,25 mm;
   - nắn đoạn lệch về 0/45/90° bằng cách trượt điểm gãy dọc đoạn trước, hoặc bẻ hai khúc;
   - vát góc 90° thành hai góc 45°, mỗi cạnh vát tối đa 1 mm.

   Sau khi viết lại, DRC của KiCad chạy với luật của project. Đoạn nào dính lỗi thì lùi về phương án nhẹ hơn: chỉ vát góc, rồi chỉ gộp khúc thẳng hàng (giữ nguyên hình đồng), cuối cùng là trả lại đồng cũ.
3. **Sửa tay:**
   - Dây VBAT_SW vào U2 và C1 đi thẳng vào pad xuyên lỗ ở F.Cu. Bỏ được hai via đổi lớp đặt sát pad; via ở U2 còn nằm đè góc pad.
   - Hai chân I2C_SDA và ADC_IR2 cạnh header J3 đi lại theo 0/45°.
   - Một via I2C_SDA dời 0,14 mm để bỏ khúc gãy 0,1 mm.
   - Một đầu +5V kéo dài vào giữa pad 21 của ESP32.

| Chỉ số | Trước | Sau |
|---|---|---|
| Số đoạn dây | 317 | 292 |
| Via | 40 | 38 |
| Đoạn lệch khỏi 0/45/90° | 8 | 0 |
| Góc gãy lệch (không phải 45° hay 90°) | 5 | 0 |
| Góc 90° | 10 | 3, đều nằm trong pad 0805 |
| Khúc ngắn hơn 0,2 mm | 31 | 2 (khúc gãy 0,1 mm; DRC không cho gấp) |
| Chỗ một đoạn thẳng bị cắt đôi | 21 | 2, đều là dây đi xuyên qua pad |
| Tổng chiều dài dây | 1324 mm | 1322 mm |

Kiểm sau cùng:
- **DRC:** sạch, kể cả khi bật mọi luật.
- **`check_weird.py` của KRT:** không có đầu dây treo, mối nối chồng mép, đồng trùng hay via nổi. Nó còn báo vài đoạn "bỏ được", đó là các chân ngắn đi vào tâm pad hoặc via, giữ lại để mối nối chắc.
- **Kiểm từng đầu dây:** không còn mối nối lỏng.
- **Độ rộng và GND:** độ rộng từng net không đổi; vùng GND mỗi mặt vẫn liền một mảng.

## Đặt lại vùng analog cho dây thẳng

Làm gọn chỉ sửa hình dáng; dây vẫn đi vòng. Để đo, so chiều dài từng net với cây nối ngắn nhất giữa các pad của nó khi chỉ đi 0/45/90°. Toàn board dài hơn mức đó 241 mm. TOF_XSHUT dài 108 mm cho quãng 35 mm: nó chạy lùi xuống gầm ESP32, vòng mép phải rồi mới lên ToF.

**Thử chỉnh router trước, không hiệu quả:**

| Cách | Dây dư | Via |
|---|---|---|
| Board lúc đó | 241 mm | 25 |
| Route lại 10 net tệ nhất (A* tối ưu, phạt đổi hướng, bỏ hướng ưu tiên theo lớp) | 232 mm | 29, kèm 6 lỗi DRC |
| Route lại cả board, cài đặt mặc định | 223 mm | 27, kèm 15 lỗi DRC |
| Route lại cả board, cài đặt ưu tiên dây thẳng | 283 mm | 31 |

**Nguyên nhân nằm ở bố trí:**
- Header J3 (11 chân xuyên lỗ) chắn ngang đầu board.
- Điện trở 10k của bộ chia áp nằm trước J3, còn ESP32 nằm sau, nên mọi dây ADC phải băng qua hàng chân J3.
- Dây I2C và XSHUT tới ToF không còn khe để qua, nên đi vòng qua đầu hàng J3.

**Đã sửa:**
1. 16 điện trở chia áp xếp thành 8 cột ngay sau J3, mỗi cột thẳng sau chân IR của nó: 10k ở hàng trước, 47k ở hàng sau. Dây ADC đi thẳng về ESP32, nhóm trái toả sang trái, nhóm phải toả sang phải.
2. Khe giữa các chân J3 thẳng hàng với chân của ToF (cùng bước 2,54 mm, lệch nửa bước). XSHUT, SCL, SDA đi qua đó.
3. SDA và TOF_XSHUT đổi GPIO cho nhau (SDA 17, XSHUT 15). Trước đó thứ tự ba chân ở ESP32 ngược với thứ tự ở ToF nên hai dây buộc phải bắt chéo nhau.
4. J4 dời sang trái các cột, J10 sang phải.
5. Silkscreen bỏ tên từng điện trở (không còn chỗ), ghi nhãn hàng "10k" và "47k".
6. Dây nguồn và motor giữ nguyên. Chỉ gỡ và route lại các net ở vùng trước, theo thứ tự: ba dây tới ToF trước để giữ khe giữa J3, rồi +3V3 và +5V, cuối cùng là dây IR. Sau đó chạy `tidy_tracks.py`; thêm vào tool hai bước: gắn đầu dây tự do vào tâm via, và kéo đầu dây chỉ chạm mép pad vào trong pad.

| Chỉ số | Trước | Sau |
|---|---|---|
| Tổng chiều dài dây | 1322 mm | 1209 mm |
| Dây dư so với mức tối thiểu | 241 mm | 119 mm |
| Số chỗ gãy | 185 | 167 |
| Via tín hiệu và nguồn | 25 | 23 |
| Số đoạn dây | 292 | 268 |
| TOF_XSHUT | 108 mm, 9 chỗ gãy | 39 mm, một đường thẳng ở mặt dưới, không via |

Kiểm sau cùng:
- **DRC:** sạch, kể cả khi bật mọi luật; khớp schematic (schematic sinh lại từ `design.py`, khác bản cũ đúng hai nhãn GPIO).
- **Mối nối và hình học:** không còn mối nối lỏng; `check_weird.py` không có đầu dây treo, mối nối chồng mép, đồng trùng hay via nổi.
- **Độ rộng và GND:** độ rộng từng net không đổi; vùng GND mỗi mặt vẫn liền một mảng.
- **EMC:** GP-001 lên từ 12 lên 16 lỗi, tổng chiều dài tín hiệu không có GND bên dưới từ 80 lên 88 mm. Phần tăng là 8 đoạn IR_Dx dài 2,9 mm từ chân J3 tới điện trở 10k, nằm trong vòng hở quanh chân J3. Các lỗi cũ dài hơn của IR_CTRL, ENC_L_A, IR_D6 đã hết.

Phần còn đi vòng chủ yếu là +3V3 và +5V, vì J10 nằm xa nguồn 5 V. Muốn giảm thêm thì phải dời J10 hoặc bỏ nó.

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
