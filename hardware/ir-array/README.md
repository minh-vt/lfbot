# IR array — tám mắt, dải cong IR-03

**IR-03 placement được người dùng duyệt G1 ngày 2026-10-19; chưa phát hành gia công.** Hai vai bo tròn, không tai đầu/bậc giữa; header đực 1×12 gần mép trong; giữ8 TCRT5000 và mạch headroom IR-02. Schematic giữ nguyên. Outline option02 đã được khớp lại vào footprint native qua KiCad IPC; placement sensor/header không đổi. Board nguồn chưa route. Một recovery candidate bị đổi thành bản có218 cung tròn và35 lỗi DRC; đã tái tạo bản sạch rồi thử reroute riêng +3V3, tăng margin +3V3 từ0.592 lên0.9005mm. Nhưng bản thử thêm3 via-in-pad SMD và EMC analyzer báo GP-001 regression cho /IR1; dù native DRC0 lỗi/0 unconnected, bản này **không được chọn làm candidate tốt hơn hoặc nhập vào nguồn**. Track /IR7 vẫn0.700mm; còn26 board warnings/42 parity warnings. Xem [review và log quyết định](OPTIMIZATION_REVIEW.md), [lịch sử outline/routing](REVIEW.md). G2/fabrication chưa duyệt.

Yêu cầu hiện tại: tám TCRT5000 trên một cung trong mặt phẳng PCB, các mặt đọc
cùng hướng xuống sàn và cùng độ cao; tối ưu diện tích PCB. Người dùng đã chốt
hai PCB tự thiết kế, **rời vật lý**: thân và đầu IR cong. Không còn ưu tiên
một PCB liền của M02. Cách gá theo [spec](../SPEC.md) và [tickets](../TICKETS.md),
chưa freeze trong bản CAD này. Gá caster in 3D do người dùng thiết kế.
BOM mua hàng vẫn là [HARDWARE_BOM.csv](../../docs/HARDWARE_BOM.csv).

## Giao diện tám mắt đã chọn

J1=`PinHeader_1x12_P2.54mm_Vertical` (header đực thẳng, 1×12). Số chân dưới đây là số pad,
không phải thứ tự nhìn từ đầu dây. Header không có khóa; đánh dấu pad1 ở cả hai
đầu và continuity-test harness trước cấp nguồn, không đoán theo màu dây. Phân bổ DevKit theo guide chính hãng v1.1,
chưa chứng minh clone V1 có cùng header/nguồn. Carrier CAD/firmware chưa migrate
sang phân bổ này và vẫn tạm dừng; không coi bảng là dây robot đã được đấu thử.

| J1 IR | Net | GPIO / ADC1 | Header DevKit chuẩn |
|---|---|---|---|
| 1 | +5V | Nguồn LED từ buck; không nối ADC | J1.21 (rail5V; phải kiểm tra backfeed hệ thống) |
| 2 | +3V3 | Nguồn pull-up từ DevKit | J1.1 hoặc J1.2 |
| 3 | GND | Ground chung | J1.22 hoặc J3.1 |
| 4 | LED_EN | GPIO14, active high | J1.20 |
| 5 | IR0 | GPIO1 / CH0 | J3.4 |
| 6 | IR1 | GPIO2 / CH1 | J3.5 |
| 7 | IR2 | GPIO4 / CH3 | J1.4 |
| 8 | IR3 | GPIO5 / CH4 | J1.5 |
| 9 | IR4 | GPIO6 / CH5 | J1.6 |
| 10 | IR5 | GPIO7 / CH6 | J1.7 |
| 11 | IR6 | GPIO8 / CH7 | J1.12 |
| 12 | IR7 | GPIO9 / CH8 | J1.15 |

BAT_ADC dành GPIO10/ADC1_CH9, headerJ1.16, **không đi qua cáp IR**. Đưa nó
vào cùng pattern continuous gồm9 kênh; không chạy oneshot trên ADC1 đang
continuous. Không dùng ADC2 DMA hoặc GPIO3 cho IR/BAT. Giữ GPIO19/20 cho USB,
0/3/45/46 ngoài phân bổ này,35…37 cho ràng buộc bộ nhớ Octal và38/48 để tránh
RGB LED khác nhau giữa các bản board. Dành11/12 cho I²C,13 cho một ToF XSHUT,
15…18 encoder;39…42 còn đủ cho bốn đầu điều khiển motor, nhưng sẽ không giữ
JTAG ngoài đồng thời. Đây là phân bổ thiết kế, không sửa pin map firmware cũ.

### Headroom ADC và nguồn

R9…R16=10k/1% kéo lên3V3; **R19…R26=33k/1% kéo từng IRi xuống GND**.
Không dùng bộ chia chung cho tám mắt vì tải mỗi mắt sẽ làm lệch các mắt khác.
Ở dòng thu bằng0: `V_IR = V3V3 × 33/(10+33)`; danh định2.533 V.
Với rail thiết kế không vượt3.6 V và cả hai R±1%, góc xấu nhất nhỏ hơn2.776 V,
so với biên2.9 V trong vùng ADC được Espressif đặc tả. Đây không phải bảo vệ
khi đấu nhầm5V hoặc chứng minh nhiễu motor đã được kiểm soát.

Trở Thevenin danh định7.674k thay vì10k: gain dòng–áp giảm khoảng23.3%.
Giá trị này sửa headroom, **không chứng minh gain tối ưu hoặc hết bão hòa
phototransistor**. Dòng thu mạnh vẫn có thể ép IR xuống sát0. Phải kiểm tra
hai pha off/on không clipping; không lấy phép trừ để cứu mẫu đã bão hòa.

Không thêm100nF vào mỗi IR: RC với7.674k đã0.767 ms, không phù hợp ngân sách
settling250µs dưới đây. C1/C2/C3 chỉ là tụ rail. Chưa chốt tụ/filter ADC hoặc
buffer khi chưa có dữ liệu harness/SAR; nếu settling không đạt phải sửa thiết
kế/timing, không tự coi nhiều mẫu liên tiếp là đã ổn định.

R1…R8=220Ω, Q1 vẫn AO3400A. Ước tính Vf1.25 V cho khoảng17mA/mắt và136mA
cả tám; đây không phải dòng đo. Buck/harness phải cấp được dòng LED cùng
logic trong khi giữ rail; tụ10µF không tự cấp toàn bộ xung LED. R18 kéo gate
về GND lúc reset; bắt đầu capture với LED_EN thấp.

### Ngân sách capture để triển khai sau

Mục tiêu thiết kế, **chưa triển khai hoặc đo**: ADC1 continuous72kSa/s tổng,
9 kênh theo bảng + BAT, ATTEN3/12bit và calibration từng chip. Một scan125µs;
mỗi kênh8kSa/s. Hai pha trong chu kỳ2ms:

| Pha | Bỏ sau cạnh LED | Thu | Thời gian conversion |
|---|---|---|---|
| Off | 2 scan =250µs | 4 scan đầy đủ =500µs | 750µs |
| On | 2 scan =250µs | 4 scan đầy đủ =500µs | 750µs |

Còn500µs trong chu kỳ2ms cho lịch/biên DMA/thu thập/xử lý. Chuyển pha theo
mốc thời gian, không giả định LED đổi đúng cạnh DMA. Bỏ mọi scan chạm cạnh,
chỉ ghép off/on đủ8 mắt trong cùng chu kỳ và đúng channel ID. BAT đọc trong
pattern, lọc riêng; không đổi pattern hoặc dừng ADC để đọc pin.

72k nằm trong giới hạn header IDFv5.5.1 tham chiếu; phải kiểm tra version và
nhịp hiệu dụng thật khi triển khai.250µs là budget, không phải thời gian đáp
ứng TCRT đã đo. Bảo toàn timestamp/age, clipping, overflow và deadline; nếu
không đạt phải báo mẫu không hợp lệ/đi an toàn, không tái dùng frame cũ như mới.
Không block hoặc xử lý điều khiển trong ISR. ESP32/HIL vẫn là công việc riêng.

Nguồn pin/ADC: [DevKitC-1 v1.1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html),
[ADC continuous](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/adc/adc_continuous.html),
ESP32-S3 Datasheet Table2-8/5-5/5-6; bản tải và phép tính ở recovery của review.

## Căn cứ thiết kế

- [Luật thi đấu](../../docs/manual.md), Điều 3.2 và 4.2: robot tối đa
  160 × 160 mm; line rộng 20 mm. Hai ràng buộc này không xác định bán kính array,
  vùng bắt lại line hoặc lookahead tối ưu.
- [Vishay TCRT5000, document 83760](https://www.vishay.com/docs/83760/tcrt5000.pdf),
  bản tham chiếu [lưu trong project](../libs/datasheets/TCRT5000_Vishay.pdf):
  hình kích thước/footprint xác định thân khoảng 10,2 × 5,8 mm, vị trí bốn chân
  và các lỗ cho ngàm. Fig. 9 cho thấy tín hiệu phụ thuộc khoảng cách tới bề mặt
  trong điều kiện thử của hãng. Không có bán kính array tối ưu hoặc đường kính
  vùng đọc có thể mặc định dùng cho lô hàng mua tại shop.
- [Pololu QTRSensors Methods & Usage Notes](https://www.pololu.com/docs/0J19/3):
  khoảng cách cảm biến phải phù hợp bề rộng line để ước lượng vị trí có tính đơn
  điệu; hiệu chuẩn từng kênh và đọc LED on/off giúp xử lý sai khác/ánh sáng nền.
  Đây là tài liệu QTR, không phải thông số quang học của TCRT5000. Công thức
  trung bình chỉ số trong ví dụ không tự trở thành tọa độ hình học cho array cong.
- [Pololu Additional Considerations](https://www.pololu.com/docs/0J12/7): độ cao
  tăng và IR môi trường làm giảm tương phản; che sáng có thể cần thiết. Không
  chuyển giới hạn độ cao của QTR sang TCRT5000. Phần che sáng không được trở
  thành chân trượt chạm sàn, theo luật thi đấu.
- [Coulter, CMU-RI-TR-92-01 (1992)](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf),
  §2 và §4.1–4.3: hình học lookahead và đánh đổi trong pure pursuit. Ngay cả
  độ cong đường đi cũng không xác định một lookahead tối ưu duy nhất. Báo cáo
  không thử nghiệm TCRT5000, không chứng minh array cong nhanh hơn array thẳng,
  và không có nghĩa firmware LF-Bot đã dùng pure pursuit.

## Tính cung thay vì chọn bán kính tùy ý

Một phương án có ý nghĩa hình học là cung có tâm tại trung điểm trục hai bánh.
Đó là **lựa chọn thiết kế** để các vị trí danh định có cùng khoảng cách tới trục,
không phải kết luận rằng phương án này tối ưu thời gian thi đấu.

Với trục x hướng trước, y hướng ngang, bán kính R và góc bố trí θᵢ:

```
xᵢ = R cos(θᵢ)
yᵢ = R sin(θᵢ)
θᵢ = (i − 3,5) Δθ, i = 0…7   [nếu chọn chia đều góc]
```

Nếu p là khoảng cách dây cung giữa hai vị trí kề nhau:

```
Δθ = 2 asin(p / (2R))
W = 2R sin(3,5 Δθ)
```

Nếu xác định trước bề rộng dây cung W và độ võng h tính từ đỉnh cung danh định
đến dây nối hai vị trí ngoài cùng (cung trước không vượt nửa vòng: 0 < h ≤ W/2):

```
R = W² / (8h) + h/2
h = R − sqrt(R² − (W/2)²)
```

Đây là hệ thức hình học, không phải công thức tối ưu điều khiển. Với tám mắt
chia đối xứng sẽ không có mắt nằm đúng tại đỉnh cung; hai mắt giữa ở ±Δθ/2.
Tâm bố trí footprint cũng không mặc nhiên là tâm vùng nhạy quang học thực tế.
Pitch 12 mm của layout thẳng cũ chưa phải yêu cầu đã chứng minh cho array cong.

Không lấy bán kính đường đua làm bán kính array. Không suy R từ riêng bề rộng
line 20 mm, kích thước tối đa robot hoặc một PCB mẫu trên mạng.

## Tối ưu diện tích

Dùng outline lưỡi liềm liền mạch theo vùng cảm biến và giao diện, không giữ nguyên tấm chữ nhật 100 × 30 mm rồi chỉ dời cảm biến. IR-03 giữ LED ở r74.5; pull-up/shunt chia hàng phía ngoài, Q1/gate gần mép trái; header đực gần mép trong. Không xung đột ngàm, vùng hàn,
lỗ gá hoặc hy sinh đường hồi LED/GND để giảm diện tích.

So sánh cả diện tích vật liệu và bounding box; không giảm diện tích bằng cách
bỏ khoảng hở chế tạo, độ bền vùng lỗ gá hoặc chỗ đi dây. Phải kiểm tra các thân
linh kiện đã xoay và toàn bộ footprint/lỗ ngàm, không chỉ khoảng cách tâm.
Đổi hướng thân TCRT5000 còn đổi hướng cặp phát/thu; không mặc định nó chỉ là
một thao tác đóng gói cơ khí.

## Đầu IR-03 theo ảnh tham khảo

[Ảnh người dùng](../mechanical/references/ir-strip-reference.png) là mẫu bố
cục, không phải nguồn kích thước/pinout. [M03-IR03](../MECHANICAL.md) giữ đầu
**tách rời vật lý** khỏi thân bằng hai liên kết G10 dogleg không đồng.
[Bản vẽ](../mechanical/layout.svg) và [tọa độ nguồn](../mechanical/layout.json)
đã cập nhật outline lưỡi liềm, giữ tọa độ lỗ/slot, liên kết và giao diện header; không đổi carrier CAD.

Nhìn từ F.Cu/mặt trên, phía trước hướng lên: `boardX=100−vehicleY`,
`boardY=137.5−vehicleX`, góc TCRT B.Cu=`90+θ`. U1/IR0 ở phía phải robot
(y−28), U8/IR7 ở phía trái(y+28). IR-02 đã sửa phép đổi dấu Y trước đây;
không mirror kênh thêm lần nữa chỉ vì đang xem mặt B.Cu từ dưới.

- R80/span56; bước góc5.85352°, pitch dây cung8.16950. R tăng để dành khe
  giữa hai PCB, không phải kết luận bán kính tối ưu điều khiển.
- Cung ngoài R88/±24°, biên sau cong nông, không tai/bậc giữa. Bao ngoài
  **85×24.5 mm**; khe thân21.5 danh định/18.5 khi lùi3. Diện tích outline
  danh định khoảng1703 mm² trước trừ lỗ; không suy giá từ diện tích. Gá vẫn cho dịch−3…+2.
- PCB đầu dày mục tiêu1.6, đáy9.5. d2.5 danh định, d2…3.5 bằng shim1.4…2.9;
  tính theo mốc7 mm, phù hợp working distance d trong Fig.3 Vishay nếu mặt chuẩn A
  sát PCB. Khe tới mốc vòm6.3 mm là3.2 mm, **không phải lý do tự hạ PCB0.7**.
  Ghi rõ datum, kiểm tra cách ngồi/clip và dung sai trước freeze shim; xem R03.
- Tám TCRT ở B.Cu, R/C và header đực J1 ở F.Cu; hai lỗ M2 tại **(70.5,±38.5)**. Các tọa độ/góc là
  mốc thân10.2×5.8, chưa phải spot quang hoặc gốc pad1. Đã đọc lại pad qua IPC
  sau phép lật B.Cu, đối chiếu đủ105 pad điện/39 linh kiện với netlist IR-03;
  chưa có đo quang.
- Dùng footprint stock `OptoDevice:Vishay_TCRT5000`; symbol local đánh số
  1=C,2=E,3=K,4=A. Không đổi trực tiếp sang CNY70 có thứ tự chân khác.

Biên native hiện gồm **226 đoạn thẳng lấy mẫu** trong `IR_Array:Outline_IR03`, khớp contour SVG option02; điểm giữa mép sau là vehicle x64.75 mm. MP1 được refresh và lưu qua IPC khi board đang mở. DRC placement hiện có83 unconnected,26 cảnh báo board (8 library mismatch,16 silk-over-copper,2 silk-overlap) và42 parity warnings; chưa có invalid outline, copper-edge clearance, pad hoặc hole conflict mới. Board nguồn còn0 track/via/zone; chưa áp dụng routing.
Sau chốt placement: routing sau G1 → ERC/netlist/DRC → review pad/mask/drill/
cơ khí → bộ xuất.
Đọc LED-off/on không phục hồi dữ liệu đã bão hòa vì ánh sáng môi trường.
