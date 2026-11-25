# Trạng thái triển khai

Cập nhật 2026-11-22. Thiết kế ở [ARCHITECTURE](ARCHITECTURE.md), thứ tự việc ở [PLAN](PLAN.md). Tài liệu này chỉ ghi cái code đang có, cái đã kiểm, và các sự thật đo trên xe còn đúng.

## Code đang có

**Core** ([firmware/core](../firmware/core/)):
- `line`: tự quét học mức nền/line và cực tính từng mắt; vị trí line theo bước mắt; hình dạng (`None`, `Line`, `Wide`, `Split`).
- `motion`: encoder + gyro → tốc độ bánh, quãng đường `s`, hướng `ψ`, vị trí `x, y`; bias gyro tìm trong lúc đứng yên.
- `drive`: (tiến, lái) → duty hai bánh, ưu tiên lái, giới hạn đổi lệnh tiến, bù pin, phát hiện kẹt bánh.
- `pilot`: tự quét IR; mỗi tick chọn `e` từ line, nhớ, dò, thẳng hoặc line ảo ([ADR-026](decisions/ADR-026-one-follower-virtual-line.md)); một luật lái cho mọi trường hợp; né gạch bằng line ảo bên phải.
- `course`: lượt 1 ghi công sức lái và mốc mỗi 20 mm; lượt 2 nhìn trước, khớp mốc, ra nhịp ([ADR-024](decisions/ADR-024-dimensionless-control.md)).
- `robot`: nút và chế độ (chờ, ổn định, quét, sẵn sàng, đua, dừng, về đích), an toàn, LED, nhật ký.

**ESP32** ([firmware/esp32](../firmware/esp32/)): driver ADC (IR + pin, có chế độ bật/tắt LED của thanh), encoder (PCNT), IMU, ToF (VL53L1X ULD), motor (MCPWM), LED (RMT), nút; vùng flash hộp đen và bản ghi lượt 1; console chỉ đọc (`status`, `monitor`, `slots`, `log`, `irctl`). Bảng chân theo board 1 mặt (SDA GPIO15, XSHUT GPIO17).

## Đã kiểm

- **Host (xe giả, không phải bằng chứng trên xe):** `just check` xanh.
  - Line thẳng trên 4 kiểu nền, góc trái/phải, giao cắt, chữ T, nút tô đen, mắt bão hoà.
  - Gạch trên đoạn thẳng, có line khác cạnh đường vòng, line bẻ 20° sau gạch; xe không chạm gạch.
  - Lượt 2 nhanh hơn lượt 1 trên cùng đường và vẫn vòng được gạch mới; khớp mốc đúng/sai cửa sổ.
  - Dừng an toàn: nút, gyro cũ hoặc sai dấu, kẹt bánh, xe bị nhấc, pin yếu; ToF cũ được ghi là thiếu.
- **Kiểm đột biến:** bỏ cổng nhận lại line thì xe đâm gạch; bỏ hành lang thì bám nhầm line cạnh bên; bỏ sửa theo mốc, công sức hoặc tăng tốc thì đúng test tương ứng hỏng.
- **Trên xe:** chưa. Code hiện tại chưa chạy trên xe; board 1 mặt chưa làm.

## Sự thật đo trên xe (khung dây rời, 2026-11-11)

Còn dùng vì cùng motor, bánh và cảm biến; kiểm lại khi lên board mới.

| Đại lượng | Số đo |
|---|---|
| `kMmPerCount` | 0,0313 (ToF so encoder, hai chiều 0,0331 và 0,0295; ToF nhiễu ±10 mm) |
| `kTrackCountsPerRad` | 3040; × mm/xung = 95 mm, khớp khoảng cách hai bánh |
| Motor trên sàn | gain 22480 / 21009 counts/s/duty, τ 0,62 / 0,55 s, vùng chết 0,21 / 0,22. Đo lúc treo bánh sai gần gấp đôi gain và bốn lần τ: không dùng |
| Dấu gyro | `+1` |
| Tick điều khiển | tối đa 0,98 ms trong chu kỳ 2 ms |
| IR (chưa có chia áp) | nền 3750–4025, line 4095: tương phản chỉ ~250 xung, sát trần ADC. Board mới có chia áp 47/57 |

## Bài học phần cứng còn hiệu lực

- **Ghi flash tắt cache hàng chục ms.** Ngắt của ADC, I2C, PCNT, RMT phải nằm trong IRAM (`sdkconfig.defaults`), và ADC tự khởi động lại khi 5 ms không có lượt quét mới. Thiếu một trong hai thì ADC chết sau lần ghi hộp đen đầu tiên.
- **Không rút pin khi LED tím sau khi xe dừng:** đang ghi flash.
- **Bảng phân vùng** chỉ được nạp khi có cả `board_build.partitions` (platformio.ini) và `CONFIG_PARTITION_TABLE_CUSTOM` (sdkconfig.defaults).
- **PSRAM tắt:** bật lên thì FreeRTOS cấp mutex vào PSRAM và panic.
- **IMU** có thể đọc sai `WHO_AM_I` ở lần boot đầu; driver thử lại 5 lần.
- **MCPWM:** force mức và nhả force phải cùng một cơ chế (continue / non-continue), nếu không chân bị khoá ở 0.
- **Console:** mở `/dev/ttyACM0` bằng `dtr=False, rts=False`; `rts=True` giữ chip ở reset và board im lặng như treo. Mở cổng làm board khởi động lại: nhật ký RAM mất, nhật ký flash còn.

## Số tạm phải đo hoặc chỉnh trên xe

| Số | Giá trị | Ghi chú |
|---|---|---|
| `kBarAheadMm` | 90 | Theo ADR-025; line ảo dùng trực tiếp |
| `kEyePitchMm` | 7 | Chưa đo; line ảo dùng trực tiếp |
| `kNodeCrawlDuty` | 0,30 | Có thể dưới vùng chết thật (~0,32) |
| `kForwardRisePerTick`, `kForwardFallPerTick` | 0,004 / 0,02 | Bánh không trượt, xe không chúi |
| `kBrickSeeMm` | 300 | |
| `kFastPace`, `kHardEffort`, `kLookaheadTicks`, `kLandmarkWindowCounts` | 1,25 / 0,3 / 150 / 300 mm | Cột `pace_pct` trong `log` cho thấy lượt 2 tăng tốc ở đâu |
| `pins::kIrCtrlWiring` | `kNotUsed` | Chân IR của thanh: đo bằng `irctl` (xem [wiring](../hardware/wiring.md#thanh-ir-8-mắt)) |
| `kSettlingSweeps` (ADC) | 2 | Chỉ khi dùng bật/tắt LED |

## Chưa có hoặc chưa làm

- Luật "ToF thấy vật gần thì không tăng tốc" (`g_ToF` của ADR-024): giả lập không cho thấy cần; thêm ở `Pilot::forward_for` nếu xe thật tới gạch quá nhanh.
- Giới hạn đã biết: gạch cách góc 90° chưa tới một thân xe thì line rẽ ngang có thể nằm ngoài hành lang nhận lại line.
- BTN_2 và IMU INT có trên board, firmware chưa dùng.
