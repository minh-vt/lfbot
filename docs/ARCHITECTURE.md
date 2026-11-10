# Kiến trúc firmware LF-Bot

**Trạng thái:** thiết kế đã duyệt ngày 2026-11-10 ([ADR-021](decisions/ADR-021-field-adaptive-two-run-design.md)). Mới triển khai phần đọc sensor và hiệu chuẩn tĩnh qua USB ([IMPLEMENTATION](IMPLEMENTATION.md)). Thứ tự làm và tiêu chí qua bước ở [PLAN](PLAN.md). Tài liệu này không chứng minh robot đã chạy.

Nguồn khác: pin map ở [wiring](../hardware/wiring.md), luật ở [manual](manual.md) và [FAQ](fqa.csv), quy ước tên ở [NAMING](NAMING.md). Thiết kế thời simulator nằm ở [history](history/README.md).

## 1. Mục tiêu và ràng buộc

- Hai lượt khác nhau. **Lượt 1:** bám line phản hồi trực tiếp từ IR, vòng gạch, ghi map. **Lượt 2:** định vị trên map, chạy kế hoạch path + tốc độ toàn tuyến, vẫn có feedback cảm biến.
- **Chạy ở sân thật:** ngoài trời, nắng thay đổi (Q-16), pin tụt, mặt sàn lạ. Không dựa vào hằng số vật lý cứng. Chỉ hai số đo của chủ dự án là giá trị khởi đầu: bánh 43 mm, bề ngang mép ngoài hai bánh 110 mm (**không phải wheelbase**).
- **Ngày thi không có laptop.** Sau check-in BTC thu xe và cấm nạp code hoặc điều chỉnh (Điều 4.7). Mọi thao tác tại sân chỉ qua **một nút** và chính robot.
- Không heap, không chặn và không ghi flash trong vòng điều khiển. Không framework, plugin hoặc interface ảo cho thiết bị chỉ có một implementation.

## 2. Vận hành bằng một nút

GPIO40 là nút duy nhất. Nhận diện ở `mission`, xử lý chống dội và chỉ phát sự kiện sau khi nhả tay.

| Thao tác | Trạng thái | Kết quả |
|---|---|---|
| Nhấn ngắn | Sẵn sàng | **Start khi nhả tay.** Từ chối nếu IR không thấy line hợp lệ, profile thiếu, pin yếu hoặc xe bị nhấc |
| Nhấn giữ | Sẵn sàng | Tự hiệu chuẩn tại sân (mục 5) |
| Nhấn giữ rất lâu | Sẵn sàng | Xóa map (đổi sa bàn) |
| Nhấn bất kỳ | Đang chạy | **Hủy:** dừng ngay, không chốt map là hoàn chỉnh |

Ngưỡng thời gian giữ nút là tham số giao diện, không phải tham số vật lý; chốt khi thử với người vận hành.

**Phản hồi trạng thái:** trước hết dùng motor phát âm (PWM tần số âm thanh, duty nhỏ, xe không được dịch chuyển; phải đo xác nhận). Nếu không đạt thì lắp LED trên GPIO39. Mỗi trạng thái và mã lỗi có một mẫu âm/nháy riêng; mẫu cụ thể chốt khi triển khai.

### Vòng đời

```text
Boot ─► Tự kiểm tra ─► Sẵn sàng ◄─────────────────────────────┐
                        │  ├─ giữ ──► Tự hiệu chuẩn ─► (đạt/không) ┤
                        │  └─ giữ rất lâu ──► Xóa map ────────────┤
                        └─ nhấn ─► Lượt N ─┬─► Đích ─► lưu map ───┤
                                           ├─► Hủy (nút / nhấc xe)┤
                                           └─► Lỗi ─► chỉ báo lỗi ┘
```

- Lỗi khởi tạo thiết bị **không abort, không reboot lặp**: xe ở trạng thái “không cho Start” và báo mã lỗi.
- **Chọn lượt dựa trên dữ liệu:** có map hoàn chỉnh thì lần Start kế tiếp là lượt 2. Map nằm trong flash nên tồn tại qua mất nguồn.
- **Đầu lượt 2, follower kiểm tra quan sát có khớp đoạn đầu của map.** Không khớp (quên xóa map, đặt sai chỗ) thì tự quay về hành vi lượt 1 và ghi map mới. Không bao giờ chạy mù theo map.
- Map lượt 1 bị Hủy giữa chừng được giữ là **prefix**, có đánh dấu. Lượt 2 dùng phần prefix đã biết, sau đó quay về reactive.

Đây là FSM vòng đời hợp lệ. Né vật cản **không** dùng FSM chuỗi động tác.

## 3. Đích: 8 mắt trắng có điều kiện

Ô đích nằm ở đầu cụt của line; thời gian do cổng của BTC đo. Robot chỉ cần nhận đích để **dừng an toàn và chốt map**. Báo đích sớm là DNF; báo muộn chỉ tốn thời gian sau khi đồng hồ đã dừng. Vì vậy bộ nhận đích phải bảo thủ.

**Ứng viên đích** khi đồng thời:
1. 8 mắt dưới ngưỡng trắng (có hysteresis), mẫu còn tươi.
2. Chế độ bám line, không phải vòng gạch hoặc tìm line.
3. Ngay trước khi mất, line là một cụm hẹp ở giữa thanh IR. Mất ở mép là cua, không phải đầu cụt.
4. Gyro cho thấy xe đang đi thẳng; ToF không thấy vật cản phía trước.

**Xác nhận:** đi thẳng thêm một đoạn suy từ hình học xe (offset IR–trục bánh và chiều dài thân) để thân chắc chắn vào ô đích, rồi quét tại chỗ trong biên góc giới hạn. Không thấy line thì là **Đích**: phanh, đứng yên, ghi map. Thấy line thì xử lý như mất line tạm thời.

**Lượt 2** cần thêm điều kiện vị trí: ứng viên chỉ hợp lệ trong vùng sai số quanh đầu cụt đã ghi trên map.

**Bắt buộc thử ngược:** cua gắt, góc 90°, vòng gạch, nhấc xe, nắng chiếu ngang đều không được báo đích.

## 4. Ba lớp tham số

| Lớp | Nội dung | Nguồn | Lưu |
|---|---|---|---|
| **Profile robot** | Bước mắt IR, offset IR và ToF so với trục bánh, tỉ lệ chia áp pin | Đo một lần ở nhà (thước, đồng hồ vạn năng), nhập qua USB **trước check-in** | Flash, có schema/version/checksum |
| **Profile sân** | Mức trắng/đen từng mắt, thứ tự mắt, chiều motor/encoder/gyro, wheelbase hiệu dụng, bias gyro | Robot tự đo (mục 5) | Flash, có thời điểm và chất lượng |
| **Thích nghi khi chạy** | Mức IR theo ánh sáng, bias gyro, hệ số motor theo áp pin, trượt bánh, tỉ lệ encoder | Liên tục, có biên | RAM; ghi kết quả đáng giữ khi xe đứng yên |

- Profile **chỉ dựng được qua bước kiểm định**. Type không cho phép “cờ sẵn sàng + dữ liệu sai”. Thiếu profile bắt buộc thì không cho Start.
- Serialize có schema; không ghi nguyên struct vào flash.
- **Đơn vị nội tại:** map và lượt 2 dùng số đếm encoder và radian gyro, chỉ cần nhất quán với chính robot trên chính sân đó. Đường kính 43 mm chỉ là giá trị ban đầu. Khi tiến về phía gạch, độ thay đổi khoảng cách ToF (mm tuyệt đối) so với encoder cho phép tinh chỉnh tỉ lệ encoder.
- Thích nghi phải có biên, chỉ cập nhật khi bằng chứng rõ. Ví dụ: chỉ học mức trắng của một mắt khi các mắt khác xác nhận line ở xa nó. Không thích nghi theo dữ liệu đang mơ hồ.

## 5. Tự hiệu chuẩn tại sân

Đặt xe lên line, nhấn giữ nút:

1. **Đứng yên:** bias và nhiễu gyro, kiểm tra accelerometer (không bị nhấc). Bias cũng được tự đo lại mỗi khi xe đứng yên ở trạng thái Sẵn sàng.
2. **Xung nhỏ từng bánh:** dấu encoder so với gyro, phát hiện đảo dây motor hoặc encoder.
3. **Lắc tại chỗ qua line:** mức trắng/đen của cả 8 mắt; thứ tự và chiều thanh IR (so với dấu gyro). Góc gyro, độ rộng line 20 mm (Điều 4.2) và bước mắt IR cho thước đo chiều dài.
4. **Quay tại chỗ:** góc gyro so với chênh lệch encoder cho ra wheelbase hiệu dụng (đơn vị encoder).

Mỗi bước có tiêu chí đạt suy từ nhiễu đo được, không dùng số cố định tùy ý. Không đạt thì báo mã lỗi theo bước, không lưu profile, không cho Start. Biên độ chuyển động có giới hạn và luôn chịu supervisor an toàn. Robot cần lớp actuator an toàn trước khi có bước 2–4.

Lệnh USB `white` / `sweep` / `gyro` / `roll` hiện có vẫn giữ để chẩn đoán ở nhà, không phải quy trình ngày thi.

## 6. Hành vi hai lượt

### Lượt 1
- Đọc IR và phân loại mẫu: không có line / một cụm / nhiều cụm / toàn đen / toàn trắng, kèm độ tin cậy. Line rộng hoặc hai cụm đối xứng **không** được coi là line nhỏ ở giữa.
- Steering phản hồi trực tiếp từ sai lệch IR (khởi đầu là PD có đạo hàm lọc), giảm tốc khi mơ hồ hoặc khi yêu cầu quay lớn. Không pure pursuit, không dựng điểm XY giả từ IR.
- Hai bánh có vòng tốc độ riêng (`wheel_control`).
- Vòng gạch phía phải thân (body-right) bằng lựa chọn chuyển động horizon ngắn, cập nhật theo sensor mới. Không chuỗi “quay N ms, đi X mm”. Bắt lại line phải kiểm tra đúng continuation, không lấy lần thấy đen đầu tiên.
- Map lưu riêng ba loại: **line đã quan sát**, **quỹ đạo xe**, **bằng chứng vật cản**, kèm sai số và thứ tự. Line bị che hoặc mơ hồ là unknown, không tự nối thẳng.

### Giữa hai lượt
Khi xe đứng yên sau Đích: chốt map và đánh giá chất lượng, lập kế hoạch lượt 2, ghi flash. Không lập kế hoạch trong lúc chạy.

### Lượt 2
- Định vị bằng encoder + gyro, sửa sai bằng IR đối chiếu với map **theo thứ tự**, không snap vào điểm XY gần nhất.
- Lập kế hoạch path chỉ trong miền có bằng chứng; không cắt qua line khác, không bắc cầu qua phần bị gạch che.
- Lập kế hoạch tốc độ toàn tuyến: forward/backward pass theo giới hạn cua, tốc độ bánh, gia tốc đo được ở lượt 1 và sai số. Phanh đón trước.
- Follower riêng: feedforward từ độ cong kế hoạch, feedback vị trí/hướng và IR sống. Mất tin cậy map thì giảm về reactive hoặc dừng.

## 7. Module

Core `firmware/lib/robot/`: độc lập ESP-IDF, chạy test trên host, trao đổi bằng struct dữ liệu.

| Module | Một trách nhiệm |
|---|---|
| `robot_data` | Struct mẫu đo (thời gian, sequence, trạng thái), lệnh, mã lỗi. Không có logic |
| `profiles` | Profile robot/sân: kiểm định, serialize có schema |
| `line_sensing` | Chuẩn hóa IR, thích nghi có biên, phân loại mẫu |
| `motion_estimate` | Encoder + gyro thành pose/vận tốc; phát hiện trượt |
| `wheel_control` | Bám tốc độ bánh: feedforward theo áp pin, feedback, giới hạn gia tốc |
| `self_calibration` | Quy trình mục 5: sinh lệnh chuyển động và profile sân |
| `run_one` | Bám line, vòng gạch, nhận đích, ghi map |
| `course_map` | Map có giới hạn dung lượng, đối chiếu vị trí, kiểm tra khớp đầu lượt |
| `run_two` | Lập kế hoạch khi đứng yên; follower lượt 2 |
| `mission` | Vòng đời, cử chỉ nút, chọn lượt |
| `safety` | Cổng an toàn: tươi mẫu, nhấc xe, pin yếu, Hủy |

Platform `firmware/src/esp32/`: mỗi thiết bị một lớp nhỏ, cụ thể (IR + pin trên ADC1 DMA, encoder PCNT, IMU, ToF, motor MCPWM, nút, phản hồi), cùng scheduler, storage và telemetry. Core không gọi driver; platform không chứa quyết định chuyển động. Tách module này khỏi `Board` hiện tại là việc của bước 1 trong PLAN.

Chính sách freshness có **một nơi định nghĩa** trong core. Ngưỡng nào cũng phải có tên, đơn vị, và nguồn: đo được, spec sensor, hoặc luật.

## 8. Luồng thực thi và nhịp

| Task | Nhịp | Việc |
|---|---|---|
| Control | 500 Hz (GPTimer đánh thức) | Mẫu IR/encoder/áp pin mới nhất + snapshot IMU/ToF → ước lượng → hành vi → bám bánh → PWM |
| I2C | Riêng | IMU nhanh, ToF khoảng 50 Hz. Không bao giờ chặn Control |
| Supervisor | Timer độc lập | Lệnh motor quá hạn thì ép dừng; nút khi đang chạy thì Hủy |
| Service | Ưu tiên thấp | Telemetry qua ring buffer cố định; flash chỉ ghi khi đứng yên; USB chẩn đoán |

500 Hz là mục tiêu ban đầu; nhịp thật, jitter và thời gian thực thi phải đo trên board. Tick bị lỡ không được phát lại thành nhiều lệnh.

## 9. An toàn

- **Một chủ sở hữu PWM.** Mọi lệnh motor có hạn dùng; supervisor ép dừng khi quá hạn hoặc khi task treo.
- Phân biệt coast (`IN1=IN2=0`) và brake (`IN1=IN2=1`) theo DRV8871. Mỗi tình huống dừng chọn rõ một chế độ.
- Boot và reset không phát lệnh motor. Chỉ cho Start khi tự kiểm tra, profile và pin đạt.
- Nhấc xe (accelerometer, bánh quay không tải), pin dưới giới hạn LiPo, sensor bắt buộc mất tươi đều dẫn tới dừng.
- Nút là Start/Hủy phần mềm, **không** thay công tắc nguồn vật lý.

## 10. Đo pin

Module voltage sensor (chia áp danh định khoảng 1:5) vào GPIO10, đấu theo [wiring](../hardware/wiring.md#nút-và-đo-pin). Tỉ lệ chia thật thuộc profile robot, đo một lần bằng đồng hồ vạn năng. Kênh pin nằm trong cùng chuỗi quét ADC1 DMA với IR.

Công dụng: chuẩn hóa feedforward motor, chặn Start khi áp mỗi cell dưới giới hạn LiPo, ghi sụt áp khi tăng tốc. Đo tổng pack không thay bảo vệ từng cell.
