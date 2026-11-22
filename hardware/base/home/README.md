# BASE-HOME: bản làm mạch tại nhà của PCB base

Cùng linh kiện, cùng vị trí, cùng mạch điện với [PCB base](../README.md) bản 0.7, nhưng route lại để tự làm trên phíp đồng 2 mặt mà **không cần mạ lỗ**. Bản gửi xưởng nằm ở `../fab`; bản này chỉ dùng khi làm ở nhà.

**Trạng thái: bản 0.7, file làm mạch đã xuất, chưa làm thử.**
- DRC theo luật làm tay của [home.kicad_pro](home.kicad_pro): 0 lỗi, 0 cảnh báo, 0 chân hở, khớp schematic.
- `check_weird.py` của KiCadRoutingTools: không có đầu dây treo, mối nối chồng mép, đồng trùng hay via nổi.
- [gen/home_check.py](gen/home_check.py): không chân nào nằm dưới thân linh kiện mà có đồng mặt trên; 2 via nằm dưới thân linh kiện (một dưới DRV, cắm sau nên không sao; một dưới bi cầu, giũa phẳng); mỗi mặt GND liền một mảng.

## Khác bản xưởng ở đâu

Làm ở nhà thì lỗ không được mạ, nên một chân chỉ nối được đồng mặt trên khi nó được hàn ở mặt trên. Chân nằm dưới thân linh kiện áp sát board thì không hàn được, nên bản này đổi như sau:

| | Bản xưởng (`../fab`) | Bản làm tại nhà |
|---|---|---|
| Đồng mặt trên ở chân nằm dưới socket, giắc, nút, IMU, buck | Có | **Cấm** (121 chân); các chân này chỉ nối ở mặt dưới |
| Via dưới thân linh kiện áp sát board (nút, giắc, header, buck) | Được | **Cấm**, vì sau khi lắp không hàn được mặt trên của via |
| 16 điện trở chia áp IR, R1, R2, C2 | Mặt trên | **Mặt dưới** (mặt đồng). D1 vẫn ở trên vì ở dưới nó đụng bi cầu |
| Dây / khe hở nhỏ nhất | 0,3 / 0,2 mm | 0,3 / 0,25 mm |
| Via | 36 via 0,8/0,4 mm, mạ lỗ | **53 via 1,4/0,7 mm, xỏ dây đồng hàn hai mặt** |
| Mối hàn bắt buộc ở mặt trên | — | **6**, đều với tới được (xem bên dưới) |
| Lớp phủ xanh, chữ in | Có | Không có; chữ in thì ủi bằng giấy in laser (tuỳ chọn) |

Giữ nguyên: vị trí linh kiện và lỗ bắt, gán GPIO ([wiring.md](../../wiring.md)), firmware.

## File để làm ([release/v0.7](release/v0.7/))

Mọi trang 1:1 có **vạch 100 mm**. In xong đo vạch bằng thước: lệch là máy in đã co giãn, phải in lại với tuỳ chọn "Actual size / 100%".

| File | Dùng làm gì |
|---|---|
| `phim/1_dong_mat_tren_lat_guong.pdf` | Phim mặt trên (đã lật gương), đồng màu đen |
| `phim/2_dong_mat_duoi.pdf` | Phim mặt dưới, đồng màu đen |
| `phim/*_AM-BAN.pdf` | Hai phim trên ở dạng âm bản: đồng trong suốt, nền đen |
| `phim/3_chu_mat_tren_ui_lat_guong.pdf` | Chữ in mặt trên để ủi (tuỳ chọn) |
| `so_do_lap_mat_tren.pdf`, `so_do_lap_mat_duoi.pdf` | Sơ đồ lắp có tên linh kiện; mặt dưới vẽ như nhìn từ dưới lên |
| `khoan/home-PTH.drl`, `home-NPTH.drl`, `*-drl_map.pdf` | File khoan và bản đồ lỗ kèm bảng cỡ lỗ |
| `LF-Bot_base-home_v0.7_gerber.zip` | Nếu tìm được chỗ phay mạch CNC: đưa file này (Gerber và file khoan) |

**Chọn phim theo loại keo:**
- **Phim khô hoặc keo UV loại âm** (chỗ chiếu UV thì cứng lại và giữ đồng): dùng bản `AM-BAN`.
- **Tấm phíp phủ sẵn cảm quang dương** (chỗ chiếu UV thì bị rửa trôi): dùng bản thường.
- **Ủi mực máy in laser** (toner transfer): dùng bản thường. Hướng lật gương đã đúng cho cả hai mặt.

## Cần thêm ngoài BOM của bản xưởng

- **Dây via:** dây đồng đơn lõi khoảng 0,5 mm (ví dụ lõi dây mạng), khoảng 1 m.
- **Khoan:** máy khoan mini và mũi 0,7, 0,8, 1,0, 1,2, 1,3, 1,8, 2,2, 3,2 mm.
- **Khác:** giấy in phim trong cho máy laser, băng kapton, dung dịch rửa hiện hình theo loại keo, FeCl₃.

## Các bước

1. **In phim** bằng máy laser lên giấy trong. In 2 tờ mỗi mặt rồi chồng khít lên nhau cho đậm. Đo vạch 100 mm.
2. **Căn hai mặt:**
   - Úp hai phim vào nhau, mặt in quay vào trong, soi đèn cho viền board và các pad tròn trùng khít.
   - Dán băng keo hai mép để thành một cái túi.
   - Cắt phíp lớn hơn viền board vài mm và luồn vào túi.
3. **Tạo mạch:**
   - Phủ keo, chiếu UV lần lượt từng mặt, rửa hiện hình.
   - Ăn mòn FeCl₃ ấm (khoảng 40–50 °C), lắc đều.
   - Tẩy lớp keo.
4. **Ủi chữ mặt trên** (tuỳ chọn), trước khi khoan để mực không lấp lỗ.
5. **Khoan** theo chấm tâm trên pad. Bảng mũi (202 lỗ):

   | Mũi | Lỗ trên board | Số lỗ |
   |---|---|---|
   | 0,7 mm | via (53); chân giắc JST ZH 0,73 mm (12) | 65 |
   | 0,8 mm | chân C1 | 2 |
   | 1,0 mm | header, socket, IMU (93); chân F1 1,01 mm (2) | 95 |
   | 1,2 mm | nút 1,1 mm (8); pad dây công tắc 1,15 mm (2); pad dây DRV 1,2 mm (8) | 18 |
   | 1,3 mm | chân buck | 8 |
   | 1,8 mm | pad dây pin 1,75 mm | 2 |
   | 2,2 mm | lỗ gá motor (4); 2 khe bi cầu 8,2 × 2,2 mm | 4 + 2 khe |
   | 3,2 mm | lỗ khay pin (4); 2 khe thanh IR 9,2 × 3,2 mm | 4 + 2 khe |

   Khe thì khoan một dãy lỗ sát nhau theo khe rồi giũa thông. Pad giắc JST ZH chỉ có vành 0,15 mm: khoan chậm, đúng tâm.
6. **Làm 53 via trước mọi linh kiện:** xỏ dây, hàn cả hai mặt, cắt sát. Đo thông mạch vài via. Via ở (−6,3; −40,8) nằm dưới bi cầu: giũa mối hàn mặt dưới cho phẳng.
7. **Linh kiện dán ở mặt dưới** (19 cái): 16 điện trở chia áp IR thành 8 cột sau J3 (10k ở hàng gần J3, 47k ở hàng sau; nhìn từ mặt dưới thì cột D1 ở **bên phải**), R1 47k, R2 10k, C2. Linh kiện dán ở mặt trên: D1, vạch quay về phía +5V.
8. **Dán kapton** lên mặt trên dưới chỗ IMU và buck sẽ nằm (chúng đè lên đầu via), rồi hàn IMU phẳng và buck.
9. **Linh kiện xuyên lỗ còn lại:** đều hàn ở mặt dưới như bình thường.
10. **6 mối bắt buộc hàn thêm ở mặt trên.** Hai mối VBAT_SW nằm trên đường nguồn ở mặt trên; bốn mối GND là chỗ duy nhất nối vùng GND mặt trên xuống mặt dưới (board không có via GND):

    | Pad | Net | Cách làm |
    |---|---|---|
    | U2 pad V+ (7) | VBAT_SW | Dây từ cầu đấu POWER của DRV trái, hàn cả hai mặt |
    | U2 pad V− (8), U3 pad V− (8) | GND | Dây từ cầu đấu POWER, hàn cả hai mặt |
    | J1 pad GND (2) | GND | Dây pin, hàn cả hai mặt |
    | C1 hai chân | VBAT_SW, GND | **Lắp tụ kênh lên 2–3 mm** để hàn được chân ở mặt trên |
11. **Kiểm trước khi cắm module:**
    - đo thông GND giữa J1, các pad GND của U2/U3, chân GND của ESP32 và các lỗ khay pin;
    - đo không chập VBAT_SW–GND, +5V–GND, +3V3–GND;
    - cấp nguồn khi **chưa cắm ESP32**, chỉnh buck lên khoảng 5,2–5,3 V.

    Phần còn lại giống bản xưởng ([release v0.7](../fab/release/v0.7/README.md)).

## Sinh lại

Đọc vị trí từ `../fab/base.kicad_pcb`, nên sửa vị trí ở bản xưởng rồi sinh lại bản này:

- [gen/make_home.sh](gen/make_home.sh), khoảng 2 phút:
  - gỡ đồng;
  - [gen/prepare.py](gen/prepare.py): chuyển linh kiện dán xuống dưới, cấm đồng mặt trên ở chân không hàn được, cấm via dưới thân linh kiện áp sát board;
  - route bằng KiCadRoutingTools theo luật trong `home.kicad_pro`, theo thứ tự: nguồn (ưu tiên mặt dưới), rồi ba dây tới ToF, rồi phần còn lại;
  - làm gọn bằng `tidy_tracks.py`;
  - [gen/center_vias.py](gen/center_vias.py): bỏ đồng trùng, bỏ mảng đồng trơ trọi, nối mảng GND bị cô lập bằng via, đưa đầu dây vào tâm via;
  - sắp nhãn, chạy DRC.
- [gen/export.sh](gen/export.sh): xuất toàn bộ thư mục `release/<bản>`.
- Schematic `home.kicad_sch` do `../gen/run_all.sh` sinh cùng lúc với schematic bản xưởng, từ cùng `design.py`.
- Kiểm: [gen/home_check.py](gen/home_check.py) (mối hàn mặt trên, via dưới thân linh kiện, vành pad) và [../gen/geometry_report.py](../gen/geometry_report.py) (góc gãy, đoạn lệch). Đường vòng và via của từng net, kèm vùng dây dày đặc: `route_review.py` của KiCadRoutingTools.

## Chất lượng đường mạch

Đo bằng `route_review.py` của KiCadRoutingTools (đường vòng, via) và [geometry_report.py](../gen/geometry_report.py) (góc). "Dây dư" là phần dài hơn cây nối ngắn nhất giữa các pad của mỗi net.

| | Bản xưởng | Bản tại nhà |
|---|---|---|
| Tổng dây / dây dư | 1209 / 119 mm | 1340 / 242 mm |
| Via | 36 (13 via khâu GND) | 53 (0 via GND) |
| Đoạn lệch khỏi 0/45/90° | 0 | 2 |
| Góc gãy lạ (không phải 45° hay 90°) | 0 | 3 |
| Góc 90° ngoài pad | 0 | 3 |

Bản tại nhà dài hơn vì 121 chân chỉ được nối ở mặt dưới, nên dây phải vòng quanh các hàng chân. Đã so 5 thứ tự route và 2 cách đặt vùng cấm via. Thứ tự hiện tại cho dây dư thấp nhất trong các bản qua DRC sạch: 242 mm, so với 282 mm ở bản đầu. Các góc và đoạn lệch còn lại là chỗ `tidy_tracks.py` sửa thì phạm luật khe hở, nên phải giữ nguyên.

## Giới hạn

- Chưa làm thử: kích thước chữ, độ chính xác căn hai mặt và cỡ vành pad mới là con số trên giấy.
- Không có lớp phủ xanh, nên hàn cẩn thận ở các pad gần vùng GND (khe hở 0,25 mm).
- Pad giắc JST ZH (bước 1,5 mm) là chỗ khó nhất. Nếu bong pad, hàn thẳng dây encoder vào lỗ.
