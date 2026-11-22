# Viết lại firmware: ESP32 trước, hai lượt chạy

Trạng thái: **đề xuất trước khi thay code**, theo yêu cầu chủ dự án ngày 2026-11-10. Đã cho phép bỏ code firmware cũ; tài liệu này chốt phương án và các dữ kiện cần xác nhận, chưa tuyên bố đã triển khai. Không tiếp tục phát triển simulator. Không tự đổi các ADR hiện hành thành thiết kế đã được nghiệm thu.

## 1. Căn cứ và phạm vi reset

Nguồn phần cứng là [wiring](../../hardware/wiring.md), không phải geometry/calibration của simulator. Cấu hình mục tiêu: ESP32-S3 N16R8, IR analog 8 mắt, hai DRV8871, hai encoder, một VL53L1X và GY-9250 trên I2C, nút GPIO40. Chủ dự án xác nhận đã có phần cứng; wiring vẫn ghi chưa xác nhận đã nhận hai DRV8871, nên cần đối chiếu đúng driver trước actuation.

Đọc adapter hiện tại cho thấy `src/esp32/` dùng core đã bỏ, pin map khác, ba VL53L0X, GPIO finish giả định, chưa có producer IMU. Đây không phải nền thích hợp để vá tiếp.

Phạm vi thực hiện sau khi chốt dữ kiện:

- Thay toàn bộ implementation core, adapter, simulator và tests firmware cũ bằng firmware ESP32 và tests cho contract mới; không giữ controller cũ dưới tên mới.
- Giữ dữ liệu nguồn: track PNG/JSON được chuyển nguyên vẹn ra khỏi cây firmware nếu bỏ simulator, không xóa để coi là cleanup. Không đụng CAD, BOM, thư viện phần cứng hoặc ghi chú ngoài phạm vi.
- Cập nhật đồng thời `justfile`, CI, clangd, ARCHITECTURE, IMPLEMENTATION và AGENTS khi thực sự chuyển code. Tạo ADR thay các điều khoản software cũ; giữ lịch sử, không sửa các báo cáo DNF thành kết quả mới.
- PlatformIO CLI vẫn là frontend duy nhất. ESP-IDF vẫn là framework; CMake nội bộ ESP-IDF do PIO gọi, không mở frontend thứ hai. Không thêm custom build hook.

Recovery đã tạo ngoài repository: `/home/minhvt/.local/state/lfbot/recovery/esp32-restart-20261110-h2sQW3/firmware-before-reset.tar.gz`. Gồm toàn bộ `firmware/` tại thời điểm lưu, kể cả file ignored/untracked trong đó, cùng các cấu hình format/lint/clangd và justfile. Có SHA256, HEAD, status và working-tree patch bên cạnh. `tar --compare` không phát hiện khác biệt ngay sau khi lưu. Recovery này không thay backup toàn repository.

## 2. Lựa chọn kiến trúc

Chọn **pipeline định kỳ, ghép module bằng dữ liệu typed và ownership tĩnh**, không dùng ROS, behavior-tree framework, plugin/controller registry, event bus hay task cho từng class.

```text
ADC / PCNT / I2C
        ↓ mẫu có thời gian, sequence, trạng thái
Drive: chuẩn hóa, kiểm tra freshness, ước lượng chuyển động
        ↓
Race: lượt 1 dò line + ghi map / lượt 2 định vị + bám plan
        ↓ mục tiêu chuyển động, không phải PWM
Drive: bám tốc độ bánh + giới hạn + kiểm tra deadline
        ↓ lệnh motor có drive / brake / coast rõ nghĩa
Adapter DRV8871
```

Đây là luồng dữ liệu, không bắt buộc tạo một class cho mỗi dòng. Đề xuất giữ một package `lib/robot/` phẳng:

| Phần | Trách nhiệm |
|---|---|
| `robot_data` | Mẫu đo, calibration, trạng thái, lệnh và lỗi; không include driver/owner |
| `Drive` | Chuẩn hóa, lịch sử mẫu, wheel/gyro estimation, điều khiển bánh và output limits |
| `Race` | Hai lượt, chọn chuyển động, bám line/path, quyết định giảm tốc hoặc dừng |
| `Route` | Bằng chứng line/vật cản, correspondence và kế hoạch toàn tuyến |
| `Robot` | Scoped owner, ghép các phần trên; không service locator |
| `src/esp32/` | GPIO, ADC, PCNT, I2C, PWM, scheduling, lưu trữ và telemetry |

Không kế thừa implementation hay interface lỗi của các class cũ chỉ vì giữ tên. Ranh giới raw measurement / normalized readings / estimate / map / plan / motor output vẫn tách biệt. Calibration vật lý ở biên; core không-thứ-nguyên giữ một nguồn quy đổi từ các giá trị đã đo, không dựng thêm bộ đơn vị riêng cho mỗi module.

“Scalable” ở đây là đổi driver không sửa luật chạy, đổi follower không sửa ADC, mở rộng map trong ngân sách rõ ràng; không phải nhiều layer hoặc hỗ trợ nhiều board chưa có.

## 3. Runtime và driver

- **Điều khiển:** khởi đầu với mục tiêu 500 Hz, GPTimer chỉ đánh thức task. Đo dt, thời gian thực thi và trễ đầu-cuối thật; đây chưa phải chứng nhận hard real-time. Missed ticks không được phát lại thành nhiều lệnh liên tiếp như thể xe đã đi qua từng tick. [E1]
- **IR:** ADC1 continuous/DMA, đúng tám GPIO trong wiring; thứ tự trái/phải và polarity qua calibration. Ghép một sweep đầy đủ, giữ khoảng thời gian acquisition và skew, không đóng dấu thời điểm đọc cache thành thời điểm đo. Overflow/mất đồng bộ phá continuity của dữ liệu, không âm thầm coi buffer còn nguyên. [E2]
- **Pin:** nếu lắp GPIO10, đưa phép đo vào lịch ADC1 chung. Không chạy oneshot và continuous đồng thời trên cùng ADC unit. RC 100 nF làm phép đo pack chậm; không dùng nó làm bảo vệ dòng tức thời. [E2]
- **Encoder:** PCNT quadrature có dấu; chốt x2/x4 và ticks/vòng qua đo. Bù overflow với accumulator và watch points đúng tài liệu; giới hạn tổng count và rebase khi đứng yên được xác định rõ. [E3]
- **IMU/ToF:** một worker sở hữu I2C, ưu tiên và giới hạn thời gian giao dịch; control chỉ lấy snapshot nhất quán. Xác minh identity trước cấu hình, không suy chip từ tên GY-9250. Timestamp mẫu có khoảng bất định nếu không nối INT/FIFO timestamp; không giả định đọc thanh ghi là acquisition tức thời.
- **VL53L1X:** dùng ST ULD phiên bản được pin, giữ license; không viết lại chuỗi thanh ghi từ adapter L0X. Chọn short/long và timing budget bằng phép thử thật. ULD công bố 15/20/33/50/100/200/500 ms, 15 ms chỉ short mode; inter-measurement phải hợp budget. Mỗi kết quả giữ range status và khoảng acquisition, lỗi không đổi thành “đường trống”. [S1]
- **DRV8871:** MCPWM, ứng viên PWM 20 kHz cần đo nhiệt/nhiễu. `00=coast/sleep`, `11=brake`, `10/01=drive` theo datasheet. Điều khiển hai IN thật, không interface PWM+DIR giả định; cập nhật chuyển chiều phải tránh xung duty ngoài ý muốn. Phân biệt motor inversion và encoder inversion trong calibration. [T1, E4]
- **Telemetry:** task ưu tiên thấp, buffer cố định và bộ đếm drop. Mất USB không chặn control. Tắt Wi-Fi/Bluetooth trong cấu hình đua ban đầu.

Ba nhóm công việc là đủ ban đầu: control, I2C acquisition, service/telemetry. ADC/PCNT/PWM dùng peripheral; không tạo task riêng cho từng mắt IR hoặc từng bộ điều khiển. Không làm I2C, flash/NVS, cấp phát heap hay log blocking trong control tick.

Toolchain đang cài cục bộ là `espressif32 7.1.3`, package `framework-espidf 4.60100.0`, header IDF `6.1.0`. Đây là kết quả kiểm tra máy hiện tại, chưa phải build firmware mới. Khi viết lại phải pin phiên bản và cấu hình N16R8 theo board thật, không chỉ tin tên generic DevKitC.

## 4. Lượt 1: chạy hợp lệ, đồng thời xây map

1. Hiệu chuẩn đen/trắng từng mắt; kiểm tra saturation, contrast, thứ tự kênh và freshness. IR coverage rộng hoặc hai cụm đối xứng không được đồng nhất với line nhỏ đang ở giữa.
2. Điều khiển trực tiếp sai lệch IR: ứng viên đầu là PD có đạo hàm lọc theo thời gian mẫu mới, giảm tốc theo ambiguity và mức quay yêu cầu. Không dựng điểm XY giả từ một coverage moment rồi gọi pure pursuit là reactive.
3. Cả hai bánh có vòng tốc độ encoder, feedforward nhận dạng và chống windup khi saturation. Gains, deadband, acceleration và tốc độ xuất phát phải đo/tune trên robot; không lấy từ sim.
4. Encoder + gyro z ước lượng pose trong hệ tọa độ chung của lượt 1. Ước lượng bias khi đứng yên; không mặc định dùng magnetometer gần motor để sửa yaw. Gyro giảm một nguồn sai số hướng, không loại bỏ trượt bánh hay drift vị trí.
5. Ghi line **đã quan sát**, pose/trajectory của xe và bằng chứng vật cản vào các tập riêng. Line mơ hồ, bị che hoặc không đủ geometry là unknown/ambiguous, không nối thẳng. Ghi theo thay đổi không gian/hướng với giới hạn dung lượng, không theo mỗi control tick.
6. Map toàn cục là các mảnh line và quan hệ có bằng chứng trong cùng một frame, kèm uncertainty và thứ tự đã đi. Không phải lưới occupancy khổng lồ, cũng không phải trace xe đổi tên thành line.

Mục tiêu lượt 1 vẫn là có một kết quả hoàn thành hợp lệ, không hy sinh lượt này chỉ để thu log. Không gọi partial map hoặc odometry trace là bản đồ hoàn chỉnh.

## 5. Né vật cản: feedback liên tục, không chuỗi động tác đếm giờ

Ứng viên cần kiểm chứng trên phần cứng là lựa chọn chuyển động trong horizon ngắn với **số candidate và số bước kiểm tra cố định**, cập nhật lại theo sensor và pose mới. Chọn body-right làm phía mặc định; có hysteresis để không đổi phía khi nhiễu, không ép dấu curvature giống nhau suốt đường vòng.

Chi phí gồm tiến dọc tuyến, clearance, độ lệch hướng, tính quan sát được và thay đổi lệnh. Ràng buộc gồm footprint thật, uncertainty, wheel limits, độ trễ và quãng hãm/coast đã đo. Không chỉ kiểm tra cung lý tưởng của lệnh mới. Mặt sau viên gạch và vùng ngoài FOV vẫn unknown; cần quan sát thêm khi di chuyển, không cấp quyền đi xuyên unknown chỉ vì không có range return.

Một ToF phía trước cho bằng chứng khoảng cách trong FOV, không cho ngay hình chữ nhật gạch hoặc bản đồ hai bên. Kết hợp các lần đo với hướng gyro/odometry để cập nhật bằng chứng; không kết luận trước rằng bộ sensor này chắc chắn đủ hoặc chắc chắn không đủ.

Reacquisition kiểm tra chuỗi mẫu IR mới, hướng tiếp cận, vùng pose khả dĩ, thứ tự/correspondence và bằng chứng trước khi mất line. Không lấy lần thấy đen đầu tiên làm chứng nhận đúng nhánh; ở lượt 1 có thể cần thu thêm quan sát hoặc dừng nếu chưa phân biệt được. Phải có thử nghiệm gạch che góc và line khác ở gần, không chỉ gạch trên đoạn thẳng.

Không dùng avoidance FSM kiểu “quay phải N ms → đi X mm → quay trái”. Lifecycle/fault state và bộ nhớ bằng chứng vẫn cần thiết. Nếu chưa tìm được lệnh vừa có tiến triển vừa đáp ứng ràng buộc, giảm tốc/quan sát thêm hoặc dừng có lý do. **Dừng không phải né thành công.** Đây là phần rủi ro kỹ thuật lớn nhất, không gọi planner là đã giải xong vì kiến trúc đẹp.

## 6. Giữa hai lượt và lượt 2

Sau khi kết thúc lượt 1: ngừng motor, chốt map và chất lượng; dựng kế hoạch khi xe đứng yên. Không tự chạy lượt 2 ngay sau khi thấy đích. Người dùng đặt lại xe, nhả rồi bấm Start cho lượt kế; reset counters/lịch sử sống không xóa map. Hai lượt thi khác với các pass xử lý map.

Lượt 2 có controller khác lượt 1:

- **Định vị:** encoder/gyro dự đoán, IR so với phần map có thứ tự để sửa sai. Không snap vào nearest XY bất kể nhánh; giữ số giả thuyết giới hạn khi quan sát lặp lại. Không xác nhận correspondence thì không tăng tốc theo map.
- **Tối ưu geometry toàn tuyến:** dựng path trong vùng có đủ bằng chứng và đúng thứ tự line; thay đổi geometry chỉ trong miền hợp lệ đã xác định, không cắt line khác hoặc bắc cầu qua phần line bị gạch che. Giữ các góc/đoạn chưa đủ căn cứ thay vì smoothing cho đẹp.
- **Tối ưu tốc độ toàn tuyến:** cap theo curvature, tốc độ từng bánh, khả năng tăng/giảm tốc và uncertainty. Forward/backward passes truyền giới hạn phanh từ cua/vật cản về trước, không phải 64 speed buckets hoặc max duty đã yêu cầu ở lượt 1.
- **Bám path:** feedforward từ curvature của plan, feedback sai lệch vị trí/hướng và IR còn sống; không phát lại PWM theo thời gian. Dùng cùng vòng tốc độ bánh và safety envelope như lượt 1. Khi map mất tin cậy, giảm về reactive trong miền line còn quan sát được hoặc dừng; không phát lại blind.

Đây là tối ưu trên map và giới hạn ước lượng, không lời hứa nghiệm tối ưu toàn cục tuyệt đối. Cần cả spatial path planning và speed planning; một bảng tốc độ đơn thuần không đáp ứng mục tiêu này. Geometry chỉ tối ưu được tới mức phép đo và luật cho phép; không mặc định có hành lang hợp lệ chỉ từ tim line ước lượng.

## 7. Liên tục và an toàn

- Tính liên tục áp dụng cho mẫu có lịch sử thời gian, estimation, mục tiêu vận tốc và chuyển tiếp actuator; không phải xóa mọi state.
- Giới hạn gia tốc bánh/thay đổi yaw, tránh giật khi đổi nguồn reactive/map; không cộng mù hai controller vào cùng motor.
- Góc không thể đi ở tốc độ hữu hạn phải hạ tốc, có thể về zero rồi quay có feedback. Không ép spline chạy nhanh qua mọi góc vuông.
- Boot/reset mặc định không drive; chỉ arm khi calibration và sensor bắt buộc đủ điều kiện. Không tạo sample zero hợp lệ cho IMU/ToF chưa triển khai.
- Output có expiry. Thiết kế đường cưỡng bức PWM về trạng thái đã định khi control không gia hạn, độc lập với task bị treo, rồi kiểm chứng bằng fault injection. Watchdog reset đơn thuần không chứng minh motor dừng đúng hạn. MCPWM có cơ chế fault/brake, nhưng chân wiring hiện không có safety interlock phần cứng riêng. [E4]
- Nút GPIO40 là Start/Stop vận hành, không phải E-stop cắt năng lượng. Phải giữ công tắc nguồn vật lý. Với dây hiện tại, phần mềm không chứng minh được an toàn khi CPU treo toàn bộ hoặc cầu H lỗi.
- `00` để motor coast không bằng active brake; boot, stale-output, fault điện và dừng thông thường cần policy riêng. Không coi `11` bảo đảm xe đứng ngay hoặc luôn là lựa chọn an toàn cho mọi lỗi.
- NVS/calibration/map chỉ ghi khi đứng yên, có version, checksum và kiểm tra tương thích. Dữ liệu lỗi không cho arm; không ghi flash trong lượt chạy. Dung lượng map, hàng đợi và log có overflow contract, không wrap thành map có vẻ hợp lệ.

## 8. Trình tự triển khai và điều kiện qua bước

| Bước | Sản phẩm | Bằng chứng cần có |
|---|---|---|
| 0 | Recovery, reset nguồn, PIO ESP32 + native tests mới, docs/build nhất quán | Build ESP32 không simulator; host tests mới, format/lint phù hợp; không dùng kết quả cũ |
| 1 | Firmware đọc sensor khi motor chưa cấp nguồn | Tám IR thay đổi đúng; hai encoder có dấu; IMU identity/data thật; L1X status/distance; nút và pin nếu lắp |
| 2 | Điều khiển bánh trên giá kê, công suất giới hạn | Chiều đúng; encoder scale; phản ứng duty; coast/brake/stop; timeout khi treo producer/control; log timing |
| 3 | Lượt 1 bám line trên sàn | Thẳng, S, góc vuông, rộng/mất line; completion tự hành với quy tắc đích có thể quan sát; không điều khiển qua USB |
| 4 | Né gạch và thu map | Đi qua khỏi gạch, bám đúng continuation; kiểm tra che góc/line gần; log không nhập nhằng stop với bypass |
| 5 | Lượt 2 định vị, tối ưu path + speed | Hai lượt trên cùng cấu hình, đặt lại xe; lượt 2 dùng map thật, có phanh đón trước, giữ hợp lệ khi sai pose/giảm pin |

Native tests là tests toán học, replay dữ liệu và lỗi contract, **không dựng world/GUI mới**. Native tests không chứng minh xe đã chạy. Bench/sàn kiểm tra hardware và hành vi: lưu log acquisition/age, raw/normalized IR, gyro, encoder, tốc độ đặt/đo, duty/stop mode, pose uncertainty, map index/quality, faults và timing. Kiểm thử nhiễu I2C, IR stale, mất encoder, map overflow, USB rút, reset/stop/rearm. Ngưỡng pass về phanh, tốc độ và timing phải được chốt từ bring-up, không điền số sim cho đủ bảng.

Đích và giao cắt chưa có hình thức quan sát được xác nhận. Wiring không có finish sensor; [manual](../manual.md) và [FAQ](../fqa.csv) nói thiết bị BTC bấm giờ, không nói thiết bị đó gửi tín hiệu cho robot. Không thêm GPIO finish giả hoặc mặc định all-black = đích. Chế độ bench có thể Stop thủ công, nhưng không được ghi nhận là autonomous race completion. Không yêu cầu liên hệ BTC; trước hết cần ảnh/mô tả start/finish và đường người dùng đang có để chốt giả định kỹ thuật rõ ràng.

## 9. Dữ kiện cần chủ dự án cung cấp

1. Driver thực nhận có đúng **hai DRV8871** không? Nếu khác, tên/ảnh mặt module. Đây là điều kiện trước mọi test motor, không phải yêu cầu mua thêm sensor.
2. Xe đã lắp cơ khí hay mới có module? Nếu đã lắp: đường kính bánh, khoảng cách hai bánh, bố trí/offset thanh IR và ToF; encoder scale được đo qua bring-up, không đoán theo tên motor. Chưa có số thì firmware đầu phải ở chế độ đo/calibration, chưa chạy tự hành.
3. Ảnh/mô tả vạch xuất phát và ô đích trên đường đang dùng, gồm các vạch rộng/giao cắt tương tự. Cần biết dấu nào robot thực sự quan sát được để chốt kết thúc và giữ map giữa hai lượt.

## Nguồn đã đối chiếu

Đọc ngày 2026-11-10; tài liệu ESP-IDF pin `v6.1` khớp header local. Các lựa chọn controller/planner ở trên là đề xuất kỹ thuật, không kết quả do tài liệu driver chứng minh.

- [E1 — GPTimer, alarm callback và task notification](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/api-reference/peripherals/gptimer.html).
- [E2 — ADC continuous, callback, overflow, hardware limitations và IRAM](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/api-reference/peripherals/adc/adc_continuous.html). ADC unit không dùng oneshot và continuous đồng thời; IRAM ISR không làm toàn bộ API IRAM-safe.
- [E3 — PCNT, quadrature và Compensate Overflow Loss](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/api-reference/peripherals/pcnt.html).
- [E4 — MCPWM, generator updates và fault/brake](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/api-reference/peripherals/mcpwm.html).
- [T1 — TI DRV8871, §7.3.1 Table 1 và §7.3.2](https://www.ti.com/lit/ds/symlink/drv8871.pdf). Không chuyển rating peak 3,6 A thành dòng liên tục của module.
- [S1 — ST UM2510, §§3.3, 3.6.3 và 5](https://www.st.com/resource/en/user_manual/um2510-a-guide-to-using-the-vl53l1x-ultra-lite-driver-stmicroelectronics.pdf). Fetch trực tiếp bị timeout; nội dung primary PDF đã đọc qua bản nội dung do web search fetch được. Không trộn giới hạn/status của ULD với L0X hoặc full API khác phiên bản.
