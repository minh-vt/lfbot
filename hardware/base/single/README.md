# BASE 1 mặt: làm tại nhà với cầu nối 0 Ω

Cùng linh kiện với [PCB base](../README.md) bản 0.7; mạch điện giống, trừ hai chân ESP32 đổi chỗ và J10 không có +5V (xem [Thiết kế lại cho 1 mặt](#thiết-kế-lại-cho-1-mặt)). Toàn bộ đường mạch nằm ở **mặt dưới**. Chỗ phải bắc qua đường khác thì dùng **điện trở 0 Ω dán ở mặt dưới** hoặc **dây nhảy ở mặt trên**. So với [bản 2 mặt tại nhà](../home/README.md): chỉ ăn mòn một mặt, không phải căn hai phim, không có mối hàn nào ở mặt trên.

**Trạng thái: bản 0.7, file làm mạch đã xuất, chưa làm thử.**
- **DRC** theo luật làm tay trong [single.kicad_pro](single.kicad_pro) (dây 0,3 mm, khe hở 0,25 mm): 0 lỗi, 0 cảnh báo, 0 chân hở, khớp schematic.
- **[home_check.py](../home/gen/home_check.py):** 0 mối hàn mặt trên.
- **`check_weird.py` của KiCadRoutingTools:** không có đầu dây treo, mối nối mỏng, đồng trùng hay via nổi. Còn 2 đoạn đồng thừa ngắn (0,2 và 0,4 mm) nằm trong đồng cùng net (vô hại).
- **Firmware:** board này cần SDA = GPIO15 và TOF_XSHUT = GPIO17 (bản xưởng ngược lại). Chưa sửa `pins.hpp`.
- **Lưu ý khi tự kiểm:** `board_score.py` và `route_review.py` của KiCadRoutingTools báo nhiều net "hở" trên board này. Chúng không hiểu footprint cầu nối (hai pad cùng số, KiCad coi là nối với nhau). Đã kiểm: tắt cờ cầu nối thì KiCad cũng báo đúng các chỗ đó, bật lại thì 0. Với board này, DRC của KiCad là chuẩn.

## So với hai bản kia

| | Bản xưởng (`../fab`) | 2 mặt tại nhà (`../home`) | **1 mặt (bản này)** |
|---|---|---|---|
| Ăn mòn | — (xưởng làm) | 2 mặt, căn hai phim | **1 mặt** |
| Qua đường khác | 36 via mạ lỗ | 53 via xỏ dây | **3 điện trở 0 Ω + 3 dây nhảy** |
| Lỗ khoan | — | 202 | **155** |
| Mối hàn ở mặt trên | — | 6 (kênh C1) | **0** |
| GND | đổ hai mặt | đổ hai mặt | đổ mặt dưới |

Khác so với bản xưởng:
- D1 (SS14) nằm ở mặt dưới và dời sang trái 1,6 mm để tránh bi cầu, vì mặt trên không có đồng.
- J4 (chọn chân IR của thanh) dời lên dải trước, cạnh J3. J10 (I2C mở rộng) dời vào giữa mép phải, ngay trên bus.
- SDA và TOF_XSHUT đổi chân ESP32; J10 chân 1 để trống.

## Cầu nối

- **3 điện trở 0 Ω** dán ở mặt dưới, tên JP… trên `so_do_lap_mat_duoi.pdf`:
  - JP1 (1206): VBAT_SW qua MOT_L_A, gần DRV trái;
  - JP2 (0805): ENC_L_B qua ENC_L_A, gần giắc encoder trái;
  - JP3 (1206): GND qua +3V3, cạnh U1.1, nối mảnh GND quanh chân AD0 của IMU (J6.7).
  - Mua dư: vài cái 0805 và 1206.
- **3 dây nhảy ở mặt trên** (6 lỗ), tổng khoảng 14 mm, dài nhất 6,4 mm:
  - +3V3 qua ba đường bus, ngay dưới J10;
  - VBAT_SW, dưới module ESP32 gần D1;
  - IMU_INT qua ba đường bus, dưới module ESP32.
  Dây đồng có vỏ bọc (hoặc dây trần nếu ngắn và không chạm gì), xỏ qua hai lỗ riêng, hàn ở mặt dưới.
- **2 lỗ dây nhảy IMU_INT nằm dưới bi cầu**, ở (4,5; −51,5) và (7,3; −55,4): giũa mối hàn mặt dưới cho phẳng.

## File để làm ([release/v0.7](release/v0.7/))

Mọi trang 1:1 có **vạch 100 mm**. In xong đo vạch bằng thước: lệch là máy in đã co giãn.

| File | Dùng làm gì |
|---|---|
| `phim/dong_mat_duoi.pdf` | Phim mặt dưới, đồng màu đen. Không lật gương: úp mặt in vào đồng |
| `phim/dong_mat_duoi_AM-BAN.pdf` | Cùng phim, dạng âm bản (đồng trong suốt) |
| `phim/chu_mat_tren_ui_lat_guong.pdf` | Chữ in mặt trên để ủi (tuỳ chọn) |
| `so_do_day_nhay_mat_tren.pdf` | Vị trí 3 dây nhảy, nhìn từ trên. **Không phải phim** |
| `so_do_lap_mat_tren.pdf`, `so_do_lap_mat_duoi.pdf` | Sơ đồ lắp; mặt dưới vẽ như nhìn từ dưới lên, có các điện trở 0 Ω |
| `khoan/single-PTH.drl`, `single-NPTH.drl`, `*-drl_map.pdf` | File khoan và bản đồ lỗ |
| `LF-Bot_base-1side_v0.7_gerber.zip` | Nếu tìm được chỗ phay mạch CNC: đưa file này |

**Chọn phim theo loại keo** như bản 2 mặt: phim khô hoặc keo UV loại âm dùng `AM-BAN`; tấm phủ sẵn cảm quang dương hoặc ủi mực laser dùng bản thường.

## Các bước

1. **Mặt trên không giữ đồng:** dùng phíp 1 mặt, hoặc với phíp 2 mặt thì không phủ keo mặt trên để FeCl₃ ăn hết đồng mặt đó. Nếu còn sót đồng ở mặt trên, mọi chân linh kiện xuyên lỗ sẽ chập vào nhau.
2. **Tạo mạch mặt dưới:** in phim, phủ keo, chiếu UV, rửa hiện hình, ăn mòn, tẩy keo.
3. **Ủi chữ mặt trên** (tuỳ chọn).
4. **Khoan** theo chấm tâm (155 lỗ):

   | Mũi | Lỗ | Số lỗ |
   |---|---|---|
   | 0,7 mm | lỗ dây nhảy (6); chân giắc JST ZH 0,73 mm (12) | 18 |
   | 0,8 mm | chân C1 | 2 |
   | 1,0 mm | header, socket, IMU (93); chân F1 (2) | 95 |
   | 1,2 mm | nút (8); pad dây công tắc (2); pad dây DRV (8) | 18 |
   | 1,3 mm | chân buck | 8 |
   | 1,8 mm | pad dây pin | 2 |
   | 2,2 mm | lỗ gá motor (4); 2 khe bi cầu | 4 + 2 khe |
   | 3,2 mm | lỗ khay pin (4); 2 khe thanh IR | 4 + 2 khe |
5. **Linh kiện dán ở mặt dưới:**
   - 18 điện trở và tụ;
   - D1, vạch quay về phía +5V;
   - 3 điện trở 0 Ω, đặt theo tên JP trên sơ đồ lắp mặt dưới.
   - 16 điện trở chia áp IR nằm thành 8 cột sau J3, 10k ở hàng gần J3, 47k ở hàng sau. Nhìn từ mặt dưới, cột D1 ở bên phải.
6. **3 dây nhảy ở mặt trên** theo `so_do_day_nhay_mat_tren.pdf`, hàn ở mặt dưới. Làm trước các linh kiện cao để dễ luồn dây.
7. **Linh kiện xuyên lỗ:** tất cả hàn ở mặt dưới như mạch 1 mặt bình thường, kể cả C1 (lắp sát board được).
8. **Kiểm trước khi cắm module:**
   - đo thông GND giữa J1, pad GND của U2/U3 và chân GND của ESP32;
   - đo thông hai đầu mỗi dây nhảy và mỗi điện trở 0 Ω;
   - đo không chập VBAT_SW–GND, +5V–GND, +3V3–GND;
   - cấp nguồn khi chưa cắm ESP32, chỉnh buck lên khoảng 5,2–5,3 V.

Cách đấu module: [wiring.md](../../wiring.md). Danh sách mua linh kiện chung: [../fab/release/v0.7](../fab/release/v0.7/README.md); thêm các điện trở 0 Ω ở trên.

## Sinh lại

Đọc vị trí từ `../fab/base.kicad_pcb`:

- [gen/make_single.sh](gen/make_single.sh), khoảng 3 phút:
  - gỡ đồng;
  - [gen/prepare.py](gen/prepare.py): chuyển linh kiện dán và D1 xuống dưới, dời J4 và J10, gán net theo `VARIANT_PINS` trong `../gen/design.py`, đánh dấu các chân nối sẵn bên trong linh kiện, cấm đồng mặt trên ở mọi pad, cấm dây nhảy dưới thân linh kiện áp sát board;
  - [gen/guide.py](gen/guide.py): vẽ sẵn bus I2C/ToF và IR_CTRL, khoá lại;
  - đổ GND mặt dưới;
  - route phần còn lại bằng KiCadRoutingTools, mặt trên đắt gấp 30 lần để chỉ dùng khi mặt dưới hết đường. Thứ tự: nguồn, kênh IR, dây ToF, phần còn lại, rồi một lượt nữa cho kênh IR. GND không đưa vào lượt nào, vì bước tự nối GND của KiCadRoutingTools sẽ kéo đồng GND lên mặt trên;
  - làm gọn (`../gen/tidy_tracks.py`), đưa đầu dây vào tâm lỗ (`../home/gen/center_vias.py`);
  - [gen/jumpers.py](gen/jumpers.py), mỗi bước chỉ giữ khi DRC không báo lỗi mới: đổi dây nhảy ngắn thành 0 Ω 0805 hoặc 1206, trượt dọc dây cho đường bên dưới nằm giữa hai pad; gộp chuỗi "dây – đoạn dưới – dây" thành một dây; thay 0 Ω không bắc qua gì bằng đoạn đồng. Chạy cả hai thứ tự (0 Ω trước, gộp trước) rồi giữ bản ít lỗ hơn;
  - [gen/gnd_bridges.py](gen/gnd_bridges.py): mảnh GND nào bị đường mạch bao kín thì nối sang mảnh gần nhất bằng 0 Ω (hoặc dây nhảy nếu xa), giữ khi DRC cho phép;
  - sắp nhãn, chạy DRC.
- [gen/export.sh](gen/export.sh): xuất `release/<bản>`.
- Schematic `single.kicad_sch` do `../gen/run_all.sh` sinh cùng hai bản kia, với các chân đổi trong `VARIANT_PINS`. Footprint `Jumper_0R_0805` và `Jumper_0R_1206` do `../gen/gen_fp.py` sinh.

## Thiết kế lại cho 1 mặt

Bản trước (route tự động hoàn toàn) cần 24 cầu nối, 18 lỗ. Ba điều về hình học của board quyết định số đó:

- **Dải chia áp chỉ có một khe trống.** 8 cột điện trở có 9 khe, mỗi khe chỉ lọt một đường 0,3 mm. 8 đường ADC đã cần 8 khe, chỉ còn khe x = 1,27 mm (giữa cột IR4 và IR5, thẳng hàng với khe giữa chân 6 và 7 của J3).
- **GND của các điện trở 47k chỉ vào được từ phía module.** Mỗi pad GND kẹp giữa hai đường ADC, nối được qua khe giữa các chân hàng trước của ESP32. Dải ngay sau hàng chân đó phải để cho GND.
- **Thứ tự chân của ToF (SDA, SCL, XSHUT), IMU và J10 cố định.**

Thiết kế lại theo đó:

- **Đổi SDA và TOF_XSHUT trên ESP32** (SDA sang U1.8 = GPIO15, XSHUT sang U1.10 = GPIO17). Ba đường bus khi chui xuống dưới module thì nằm cạnh nhau đúng thứ tự ToF cần.
- **Bus đi vòng, không bắt chéo gì:** từ U1.8–U1.10 xuống dưới module, chạy dưới thân IMU (cách dải GND sau hàng chân), rẽ lên mép phải, đi xuyên qua chân SDA/SCL của IMU (J6.4, J6.3) rồi của J10 (J10.4, J10.5; XSHUT đi khe bên cạnh), dọc mép trước rồi vào ToF đúng thứ tự chân. IMU giữ nguyên chỗ: chân SDA/SCL của nó vốn nằm đúng trên đường đi lên.
- **IR_CTRL lấy khe trống duy nhất** của dải chia áp để lên J4, nay đặt ở dải trước.
- **Bỏ +5V khỏi J10:** nhánh này trước đi vòng gần hết board.
- **Cầu nối miễn phí:** các chân mà linh kiện đã nối sẵn bên trong được khai báo là nối với nhau, nên đường mạch được dùng chúng để đi qua: hai chân cùng phía của mỗi nút 6 mm, các chân GND và hai chân 3V3 của DevKit ESP32, GND vào và ra của MP1584.

Kết quả: 3 điện trở 0 Ω + 3 dây nhảy (6 lỗ). Mỗi cầu còn lại có lý do riêng: IMU_INT và +3V3 phải vượt ba đường bus để tới IMU và phía sau board; hai cầu ở phía sau (VBAT_SW, encoder trái) có từ bản đầu.

## Đã thử giảm cầu nối

Mục tiêu: ít cầu nối nhất mà vẫn nối đủ mọi net. Mỗi cách đều xây lại toàn bộ board rồi đếm:

| Cách | Kết quả |
|---|---|
| Cho đường mạch chạy dưới thân linh kiện có sẵn (D1, điện trở, tụ) thay cho cầu nối | KiCadRoutingTools vốn được phép làm vậy (bản xưởng có đường chạy dưới C2, R21, R22). Ở bản 1 mặt nó không thấy lợi ở chỗ nào |
| Thay điện trở 0 Ω không bắc qua gì bằng dây đồng thẳng | Bớt được 2 cái (17 → 15) |
| Trượt 0 Ω dọc dây nhảy: khe 0,8 mm của 0805 chỉ vừa một đường 0,3 mm với khe hở 0,25 mm mỗi bên, mà router không đặt đường vào giữa | Thêm được 0 Ω ở những chỗ trước bị chạm pad |
| Gộp chuỗi "dây – đoạn dưới – dây": router xuống mặt dưới một đoạn ngắn chỉ để một dây khác đi qua, nhưng dây có vỏ được bắt chéo nhau | 5 chuỗi gộp được. Cùng hai bước trên: 13 → 9 dây nhảy, 26 → 18 lỗ |
| Phạt via nặng hơn trong KiCadRoutingTools (`--via-cost` 1000–3000, mặt trên đắt gấp 100 lần) | Tệ hơn: các đường đi vòng xa chặn net route sau, còn 6–13 đường chưa nối, có dây dài 112 mm |
| Bỏ +5V khỏi J10 | Một lần route còn 4 đường chưa nối: router quá nhạy với thay đổi nhỏ để kết luận |
| Bỏ +5V khỏi J10 và vẽ tay trước bus mà không đổi chân: SCL qua khe giữa dải chia áp, SDA/SCL dưới module ESP32 sang IMU và J10, IR_CTRL dưới module sang J4 (8 cách phối hợp) | Bản tốt nhất có 23 cầu nối nhưng vẫn còn mảnh GND rời; nối các mảnh đó thì thành khoảng 25 |
| Thiết kế lại ở trên (đổi chân SDA/XSHUT, dời J4/J10, bus vẽ tay) | **6 cầu nối, 6 lỗ**: đang dùng |
| Vẽ tay thêm +3V3 (một dây nhảy phía bắc J10 xuống IMU) | 11 cầu nối: router đổi cách đưa +3V3 ra phía sau và bắt chéo encoder; không dùng |
| Hoán đổi chân GPIO bằng target swap của KiCadRoutingTools | Không bớt được cầu nối nào, lại đổi bảng chân; không dùng |
| Đặt lại J10, J4, J9, nút, D1, R1/R2/C2 bằng placer của KiCadRoutingTools (12 phương án, 6 phương án tốt nhất được route đầy đủ) | Các phương án ít cầu nối hơn đều còn 3–16 đường chưa nối, mà mỗi đường hở lại cần thêm cầu nối; không phương án nào hơn |

Router đặt từng net một, theo đường ngắn nhất lúc đó, nên không tìm cách sắp xếp ít chỗ bắt chéo cho cả board. Vì vậy bus được vẽ tay, router chỉ làm phần còn lại.

## Giới hạn

- Chưa làm thử.
- Không có lớp phủ xanh: hàn cẩn thận chỗ đường mạch chạy dưới điện trở 0 Ω (khe 0,25 mm).
- Pad giắc JST ZH chỉ có vành 0,15 mm, như hai bản kia.
- Mặt dưới còn 2 góc 90° và 2 đoạn lệch góc ngắn (đo bằng [geometry_report.py](../gen/geometry_report.py)).
- Firmware chưa có bảng chân cho bản này (SDA = GPIO15, XSHUT = GPIO17).
