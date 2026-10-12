# LF-Bot 2026 — Kiến trúc tái thiết: Kernel không-thứ-nguyên + Adapters

> Quyết định ghi nhận: `docs/decisions/ADR-*.md` · Việc còn dở: `docs/ROADMAP.md`.

## Problem Statement

Robot LF-Bot thi đấu F-Maze-Solving: sa bàn **bí mật** (không biết trước tới ngày thi), 2 run (Run 1 dò, Run 2 sprint), né vật cản, 500Hz hard real-time trên ESP32-S3, phải **tự map + dựng profile on-chip trong 3 phút**. Kiến trúc hiện tại không đáp ứng vì:

1. **God-class SimEngine** (8 trách nhiệm) + **3 struct state trùng lặp** (`RobotState`/`SimRobotState`/`RigidBodyState`) chép tay ~15 trường mỗi tick.
2. **Segment map rời rạc** (3 bin, debounce 30 tick) thay vì profile liên tục.
3. **Trộn vật lý** (mm, m/s, μ, g) vào trong trí tuệ — robot thật không quan sát được các đại lượng này.
4. **`just sim-test` hỏng** (alias `lf::RobotMode`) — không verify tự động, vi phạm hợp đồng kiểm thử.

## Solution

Kiến trúc **2 tầng**: drive không-thứ-nguyên (thuần, không Raylib/OS) + imperative-shell adapters (sim, esp32). **Một seam ngoài duy nhất** `step(state, frame, plan, cfg) → command`. Profile liên tục 1-D (2 mảng theo tick) + **backward-pass tick-space**. Chiến lược 2-run thống nhất (Direction A): Run 1 ghi, Run 2 phát lại — không có controller riêng cho từng run.

## User Stories

1. Là kỹ sư firmware, tôi muốn toàn bộ trí tuệ robot nằm trong một module drive thuần, để hiểu và sửa toàn bộ engine quyết định trong dưới 5 phút.
2. Là kỹ sư firmware, tôi muốn drive chỉ thấy đại lượng không-thứ-nguyên (`line_offset ∈ [-1,1]`, tick thô, `dt`), để cùng một code chạy nguyên vẹn trên sim và ESP32.
3. Là AI agent, tôi muốn một seam ngoài duy nhất `step()`, để đọc code theo đường `main → adapter → drive` mà không lạc trong 24 file.
4. Là thí sinh (robot), tôi muốn tự dò sa bàn bí mật ở Run 1 và dựng profile trên chip, để Run 2 sprint nhanh hơn mà không cần map cho trước.
5. Là thí sinh, tôi muốn profile liên tục (không segment map), để không mất thời gian debounce/lookup khi vào cua.
6. Là thí sinh, tôi muốn backward-pass tick-space giảm ga *trước* khúc cua (ràng buộc phanh phi-cục bộ), để không lao quá tốc độ vào cua và mất vạch.
7. Là thí sinh, tôi muốn né vật cản ở cả 2 run — Run 2 biết trước vị trí từ Run 1 — để không va chạm ở tốc độ sprint.
8. Là kỹ sư CI, tôi muốn `just test` + `just sim-test` xanh trong < 1s không cần GUI, để mọi thay đổi được verify tự động.
9. Là người dùng simulation, tôi muốn `just sim` hiển thị cuộc đua 2 run với HUD, để quan sát hành vi bằng mắt.
10. Là trọng tài (arbiter), tôi muốn phán quyết ground-truth (finish box, clearance, wrong-way) độc lập với niềm tin của robot, để kết quả khách quan.
11. Là kỹ sư firmware, tôi muốn adapter ESP32 đổi ADC/PCNT/ToF → frame chuẩn hóa và `DriveCommand` → MCPWM, để phần cứng và mô phỏng dùng chung một drive.
12. Là nhà phát triển, tôi muốn mission FSM (Idle→Run1→Run2→Done) là một switch ~10 dòng mỗi shell, để không tạo module orchestration thừa.

## Implementation Decisions

- **D1 — Functional core + imperative shell:** drive = free functions thuần (DOD); class chỉ cho adapter sở hữu tài nguyên (Raylib `Image`, peripheral ESP32).
- **D2 — Ranh giới drive:** drive = normalized-in → command-out + toàn bộ chiến thuật (mất vạch, né, finish latch, profile). Quy hoạch quỹ đạo 2D loại bỏ.
- **D3 — Một facade:** `step(state, frame, plan, cfg) → command` là seam ngoài + bề mặt test; internal seams (`resolve_line_offset`, `steer_to_line`, `avoid_strength`, `choose_speed`, `drive_motors`) test trực tiếp.
- **D4 — Profile liên tục 1-D:** hai mảng `planned_steer[tick]` + `speed_limit[tick]`, nội suy O(1). Không SegmentType/debounce.
- **D5 — Dimensionless contract:** drive chỉ thấy `[-1,1]`, tick thô, `dt`. Không mm/m/s/μ/g/wheelbase.
- **Direction A — Unified controller:** `steer = planned_steer[tick] + PD(e)`; `throttle = speed_limit[tick] · obstacle_factor`. Run 1 = plan rỗng (feedback-dominant, đang ghi), Run 2 = plan đầy (feedforward-dominant, phát lại). `step` **không biết mode** — "plan rỗng vs đầy" chính là mode.
- **Backward-pass tick-space (plan_speed):** quét ngược O(N), dimensionless:
  `speed_limit[tick] = min(grip_throttle(tick), speed_limit[tick+1] + brake_limit)`,
  với `grip_throttle(tick) = 1 − corner_k·|planned_steer[tick]|`.
  Lý do tồn tại: ràng buộc phanh là **phi-cục bộ** (khớp `tick` với `tick+1`), grip limit chỉ là cục bộ. `brake_limit` tuyến tính là xấp xỉ bậc nhất của ràng buộc `v²`, hấp thụ vào tuning.
- **μ·g·R là oracle trong sim, không vào drive:** mô hình grip vật lý cần `R` (hình học 2D) + đơn vị vật lý → vi phạm D4/D5 và robot thật không tính được. Sim dùng nó để **kiểm chứng** profile của robot (profile-feasibility test).
- **Né vật cản (F1):** mảng thứ 3 `obstacle_side[tick]` (±1) ghi ở Run 1, làm mượt thành cửa sổ ±W tick (chịu trôi odometry), dùng làm bias feedforward ở Run 2 + ToF phản ứng làm safety net.
- **Tuning (F2) — auto-tune 3 tầng:** Run 1 chính là calibration — profile feasible-by-construction (ghi cái đã *thực thi*, không phải kế hoạch mơ). `Tuning` tách làm 2 lớp: **(1) derive** — `d_filter_alpha` (từ `dt`+fc), `obstacle_window` (từ lookahead+odometry), `brake_limit` (max drop quan sát Run 1); **(2) measure-once (hằng số robot)** — `kp`/`kd` qua pole-placement (ζ=0.7, ωn đo 1 lần), `speed_limit(κ)` (hàm theo ĐỘ CONG, circle-test on-robot, không cần μ), `avoid_strength/slowdown` fix theo hình học; **(3) trim-on-track (2 núm theo sa bàn)** — `base_throttle` + `sprint_scale` tự chỉnh trong Run 1 theo quy tắc line-loss monotonic. Kết quả: 10 tham số "giả định" → ~2 tham số "đo được". Không adaptive online, không μ·g·R trong lib/drive. (ticket 17, 23)
- **Mission FSM (F3):** enum `RunMode` trong drive types; switch ~10 dòng mỗi shell; không `MissionController`.
- **Chiến lược 2-pass (F4) — chạy đúng giới hạn bám:** Run 1 dò map loại bỏ uncertainty — reactive phải chạy DƯỚI √(μgR) (để biên an toàn), còn ta chạy ĐÚNG √(μgR) nhờ map. Bốn phần mềm để đạt: **(1)** `speed_limit(κ)` đo theo độ cong thay grip-theo-steer cũ (ticket 23); **(2)** `brake_limit` đo thật thay `decel_per_tick` giả định (ticket 23); **(3)** map `line_curve[t]` thuần tách khỏi `planned_steer` (ticket 24); **(4)** offset profile `d[t]` racing-line 1.5-D (ticket 25 — luật đã rõ: cắt line = "đi tắt qua đoạn line khác"; offset trên CÙNG đoạn line hợp lệ). Track có thẳng + S-curve + góc 90° (FAQ Q-12) → đoạn thẳng làm phanh "đắt", nên offset/racing-line càng có giá trị.
- **Phân rã module (as-built):** `lib/drive/` (control thuần, dimensionless: `step`/`plan`/types) · `lib/sensing/` (chân lý cảm biến + chuẩn hóa vật lý→dimensionless: `kIrOffsetsMm`, `kTofRangeMm`, `normalize_ir`, `normalize_tof`) · `src/sim/` · `src/esp32/`. Kernel không phụ thuộc sensing; cả 2 adapter tham chiếu sensing.
- **Phân rã sim (5 module):** `TrackAsset` (sở hữu Image) · `WorldPhysics` (physical↔normalized) · `RaceArbiter` (trọng tài) · `Renderer` (view thuần) · `main` (loop + FSM ~40 dòng). Không SimEngine, không `get_*_mut()`, không raylib rò vào physics.
- **State SSoT:** drive = `DriveState` (nhỏ, value-returned) + `RacePlan` (lớn, const-ref); sim = một physical state (WorldPhysics) + view const; xóa `RobotState`/`SimRobotState` trùng.
- **Seam:** `Readings`/`DriveCommand` là hợp đồng duy nhất giữa drive và 2 adapter (adapter thật: 2 implementation, không phải seam giả định).

## Quyết định ghi nhận (Audit & As-built)

Các quyết định phát sinh trong quá trình triển khai + audit clean-code:

- **AS-1 — Thư viện JSON: nlohmann-json.** Manifest track (`assets/*.json`) parse bằng `nlohmann::json` (`#include <nlohmann/json.hpp>`, hệ thống tại `/usr/include`). **Chỉ dùng trong sim** (`TrackAsset`); drive và esp32 không phụ thuộc. Thay cho parser tay ~70 dòng.
- **AS-2 — Module `lib/sensing/`:** shared sensing model — chân lý cảm biến dùng chung (`kIrOffsetsMm`, `kIrHalfSpanMm`, `kTofRangeMm`) + chuẩn hóa thuần vật lý→dimensionless (`normalize_ir`, `normalize_tof`). Cả sim lẫn esp32 tham chiếu cùng một nguồn; layer base, không phụ thuộc gì. (Tên cũ `layout` đặt theo nội dung — đổi sang vai trò.)
- **AS-3 — Hạ tầng lint:** thêm env `lint` trong `platformio.ini` (gộp mọi include path `lib`/`src/sim`/`src/esp32`/`test`) để `compiledb` sinh `compile_commands.json` đầy đủ cho clang-tidy (khắc phục lỗi file test không có compile command).
- **AS-4 — Chính sách clang-tidy:** 0 warning là bắt buộc. Tắt `cppcoreguidelines-pro-bounds-avoid-unchecked-container-access`, nhất quán với 3 check pro-bounds đã tắt sẵn (dự án chấp nhận `vec[i]` không kiểm biên vì hiệu năng realtime).
- **AS-5 — sentrux layers:** `sim`(1) · `esp32`(1) · `drive`(2) · `sensing`(3); số nhỏ = tầng trên (được phụ thuộc tầng dưới). Thay config cũ tham chiếu `lib/{system,control,track,math,config}` đã xóa.
- **AS-6 — Tích lũy encoder không-thứ-nguyên:** adapter sim tích lũy quãng đường bánh bằng `float` (mm) rồi mới cast sang tick cho `Readings`, tránh cắt cụt về 0 khi increment < 1 tick (500Hz) làm encoder đóng băng.
- **AS-7 — Tinh giản (code-simplification):** `std::clamp` thay `clampf` tự viết; hằng có tên cho magic number (`kSafetyTrim`, `kRobotRadiusMm` suy từ half-len/width, `kPi` dùng chung); tách `steer_evasion`/`smooth_obstacle_side`/`bake_throttle`/`clip_slab`; bỏ nested ternary và vòng `i-- > 0`; hàm `check()` thay macro `do-while` trong test.
- **AS-8 — Comment convention:** câu nối tiếng Việt, thuật ngữ SSoT giữ nguyên tiếng Anh; cấm decoration (`// ---` banner, `// namespace` đóng) và restate-name; giữ comment why/đơn vị/range. (ticket 13)
- **AS-9 — Xóa `TofResult`:** `TofResult.{danger,side}` trùng `Readings.obstacle_danger/obstacle_side`; `normalize_tof` ghi thẳng vào `Readings&`, gộp 3 hằng `300.0f` về `sensing::kTofRangeMm`. (ticket 15; vị trí normalize chốt ở `sensing` — ticket 19)
- **AS-10 — Mission FSM ở shell (REVERT ticket 16):** không `mission.hpp`/`MissionController`; FSM ~10 dòng mỗi shell theo D3. (ticket 20)
- **AS-11 — Layout một nguồn chân lý:** `kIrHalfSpanMm = kIrOffsetsMm.back()`; xóa `RacePlan::empty()`, dead-clamp trong `drive_motors`, `deg2rad` 1-caller, 7 handle global rời. (ticket 14)
- **AS-12 — Tooling (REVERT ticket 18):** compile DB **tách riêng** — `compiledb` native-only; esp32 để clangd mờ (chấp nhận blurry). Một DB không phục vụ 2 ABI (host glibc vs xtensa newlib). esp32 lint = cppcheck. (ticket 22)
- **AS-13 — `Tuning` tách 2 lớp:** giữ `Tuning { RobotTuning robot; TrackTuning track; }` trong `types.hpp`; KHÔNG giữ `drive/tuning.hpp` (derive/trim speculative — viết lại khi có calibration thật). (ticket 17, 21)
- **AS-14 — speed_limit(κ) + brake_limit không-thứ-nguyên (bỏ μ khỏi mọi thứ trừ oracle):** grip theo steer (`corner_limit` cũ) → hàm `speed_limit(κ)` (bảng/đường trơn theo độ cong, nội suy); `decel_per_tick` giả định → `brake_limit` đo thật, trong `tuning.robot`/`tuning.track`; `plan_speed()` lookup cả hai. Index là ĐỘ CONG (geometry), không phải steer (control). Đo on-robot bằng một mode calibration (circle test + phanh gấp + phát hiện trượt qua gyro/encoder/line-loss) trên sàn thi, lưu NVS. IMU (MPU6050) thành peripheral, gyro dùng làm sensor trượt. μ/g/R không vào lib/drive. (ticket 23)
- **AS-15 — Geometry tách khỏi control signal:** `planned_steer[t]` (tín hiệu lái, gồm nhiễu PD) không phải geometry. Ghi `line_curve[t]` = độ cong thuần (low-pass) để Run 2 feedforward sạch + PD nhẹ. (ticket 24)
- **AS-16 — Offset profile d[t] (racing line 1.5-D):** lệch ngang `d[t]` ≤ sensor span (line vẫn dưới robot), suy trong `plan_speed` từ `line_curve` + `speed_limit` + `brake_limit`; Run 2 chạy `line_offset` setpoint = `d[t]`. KHÔNG cần pose 2D. Luật (FAQ Q-15): "cắt line = đi tắt qua đoạn line KHÁC" → DNF; offset trên CÙNG đoạn line không phải cắt, hợp lệ. Làm sau 23+24. (ticket 25)
- **AS-17 — Thực tế cuộc thi (docs/fqa.csv):** track = thẳng + S-curve + góc vuông 90° (Q-12); vật cản = gạch 205×95×95mm đặt **trên tim line** (Q-13) → né vật cản là detour bắt buộc off-line rồi bám lại CÙNG line (Q-15: hợp lệ); cắt line = "đi tắt qua đoạn line khác" = DNF, KHÔNG còn giới hạn 5s off-line; sân ngoài trời có gió/ánh sáng (Q-16) → IR cần chống nhiễu ambient; đích = chạm ô đích, cảm biến tự dừng giờ (Q-17/18).
- **AS-18 — Continuous line belief:** line-following là niềm tin liên tục (`line_certainty` + `line_estimate`), không phải if/else + ±1. Mất vạch → giữ `line_estimate` (đi thẳng theo tiếp tuyến), không slam — fix bug xoay vòng. Heading ψ (IMU) để sau. (ticket 26)
- **AS-19 — Naming (Linux-style):** tên = việc robot làm, không phải lý thuyết; lý thuyết trong comment/implementation. Bỏ `docs/CONTEXT.md` + 8 thuật ngữ cũ (RawTrace/...). Quy ước ở `docs/NAMING.md`; đổi tên code ở ticket 27.
- **AS-20 — Thiết kế UI/UX mô phỏng: Neutral Telemetry + Video Playback Scrubber (ticket 05):** Single-focus Smooth Chase Camera (~1.5× scale) + phím `[C]` đổi Macro Overview; thanh tiến độ 1D đáy màn hình tích hợp video playback scrubber tua frame-by-frame (`Space` pause, phím mũi tên tua, kéo chuột, `[,]`/`[.]` tốc độ); 8 micro-LEDs IR cản trước + 3 nón ToF FoV 25° (bán kính 300mm); Heatmap Ribbon Run 1 (tốc độ backward-pass) + Ghost Car Run 1 ở Run 2; hot-swap sa bàn qua phím `[Tab]` (Vòng Loại 6m ↔ Chung Kết 10m); hot-reload `tuning.json`; bảng màu Neutral công nghiệp (Matte Charcoal `#0e1013`, Zinc `#fafafa`, Amber `#f59e0b`), cấm drawer/sidebar để tối ưu không gian ngang sa bàn 10m.

- **AS-21 — First-principles tuning (zero hand-tuned constants):** Mọi tham số `Tuning` chỉ thuộc 1 trong 3 loại: SUY (hình học+dt) · ĐO (calibration once) · XÓA (thay bằng luật vật lý). Plant line-following `ë = (v²/(L·s))·u` → `steer_kp/steer_kd` KHÔNG là hằng số: pole-placement ζ=0.7 cho `kp = ωn²·L·s/v²`, `kd = 2ζωn·L·s/v²`, **schedule theo v** (encoder). Hai trần tốc độ độc lập: **tracking** `v ≤ ωn·√(s·L·budget/|κ|)` + **friction** `v ≤ √(μg/|κ|)`, chạy `min` (mất vạch ở cua = vượt trần tracking, không phải trượt lốp). `speed_limit(κ)` = bảng đo slip-boundary (circle-test đến trượt); `brake_limit` = a_brake·dt/v_max; `avoid_slowdown` → luật phanh `v²≤2·a_brake·d`; `avoid_strength` → detour side-commit; `offset_gain` → `argmin ∫ds/v`; `sprint_scale` → 1.0; `base_throttle` → error-budget speed (`|line_offset|≤0.5`); `obstacle_window` = ceil((ToF_range+half_len)/tick_len). Còn 3 lựa chọn hợp pháp: ζ=0.7, margin 0.9, ωn (chặn bởi motor bandwidth). IMU optional: gyro → ψ sạch + slip detection; không gyro → ψ̂=de/ds; chiến thuật không phân nhánh theo IMU. Xem tickets 33–37.

## Quyết định tối ưu hóa (cpp-pro audit) — Ticket `issues/07..12`

- **OPT-1 — ESP32 = ESP-IDF + FreeRTOS, bỏ Arduino:** `framework = espidf`; vòng điều khiển là FreeRTOS task pin core 1, `vTaskDelayUntil` chu kỳ **cố định 2ms** (500Hz thật, `dt` hằng). Arduino `delay()` + `micros()` (jitter, thực tế ~350–400Hz) bị loại.
- **OPT-2 — ADC-DMA:** 8 kênh IR đọc qua ADC continuous DMA (nền, ring buffer) thay `analogRead()` blocking (~0.5ms/tick).
- **OPT-3 — PCNT:** encoder đếm quadrature phần cứng (chiều + ×4) thay `attachInterrupt` SW ISR (rớt xung, không biết chiều).
- **OPT-4 — MCPWM:** điều khiển cầu H bằng MCPWM (2 đôi complementary + dead-time) thay LEDC.
- **OPT-5 — ToF + cảm biến đích:** I2C master driver + VL53L0X (XSHUT sequencing GPIO 13/14/21 cho 3 cảm biến); cảm biến đích thật thay `finish_detected = false`.
- **OPT-6 — Kernel static allocation:** `RacePlan` dùng fixed-capacity (không `std::vector::resize` trong vòng 500Hz); dung lượng tối đa theo chiều dài sa bàn (≤ 10000 mm → ≤ ~12k tick × 3 mảng).

**As-built (build đã verify):** `framework = espidf` — **ESP-IDF 6.1**, bỏ hẳn Arduino.
- OPT-1 ✅ FreeRTOS `vTaskDelayUntil` 2ms, task pin core 1 (entry `app_main`)
- OPT-3 ✅ PCNT quadrature (`driver/pulse_cnt.h`, API mới)
- OPT-4 ✅ MCPWM 2 cầu H 20kHz (`driver/mcpwm_prelude.h`)
- OPT-5 ✅ I2C master mới (`driver/i2c_master.h`) + XSHUT + VL53L0X single-shot (init đầy đủ = TODO phần cứng; finish pin giả định GPIO22)
- OPT-2 ⚠️ `esp_adc/adc_oneshot.h` — `adc_continuous` (DMA) có sẵn trong IDF 6.1 nhưng chưa dùng (oneshot đơn giản trước)
- OPT-6 ✅ Kernel fixed-capacity

`src/CMakeLists.txt` riêng để espidf chỉ biên dịch `esp32/*` (không `src/sim`). Build ESP32-S3 cần venv `uv` (`.venv`) + `pio` từ venv; env esp32s3 cần `build_unflags = -std=gnu++11` + `-std=gnu++17` (framework ép C++11 → thiếu `std::clamp`).

## Testing Decisions

- Test qua facade `step` (tích hợp hành vi) + trực tiếp internal seams (math thuần). Không mock, không ODE solver tổng hợp.
- **Bất biến:** line centering (zero error → thẳng, cân bằng); deflection (error dương → lệch lái ngược chiều kim); anti-saturation (không clip duty); proximity deflection (danger dịch setpoint mượt); finish latch; backward-pass (giảm ga trước cua); replay (thẳng tăng tốc, cua giảm).
- **Profile-feasibility oracle test:** sim tính `v_max[s]` vật lý (μ·g·R) → chuẩn hóa → assert `speed_limit[tick]` của drive ≤ `v_max` tương ứng. Chứng minh drive không đòi vượt giới hạn vật lý.
- `just test` (drive unit) + `just sim-test` (headless 2 run) đều < 1s, không GUI.
- **Chống regression:** một namespace `lf::drive`, không alias rải rác (lỗi `lf::RobotMode` cũ).

## Out of Scope

- Quy hoạch quỹ đạo 2D / QP / replan đường vòng vật cản.
- μ·g·R trong drive (chỉ oracle trong sim).
- Telemetry/UDP/wireless dashboard.
- Track editor, toast manager, input controller.
- Calibration nghi thức (fit μ, a_brake).
- Adaptive control online.
- Hardware PCB/schematic.

## Further Notes

- **Tái thiết từ con số 0:** code cũ (~4600 dòng, 24 file) bị thay thế, không di trú từng phần.
- **As-built:** 23 file / ~1637 dòng — drive 5 · sensing 1 · sim 10 · esp32 5 (main + sensors + actuators) · test 2.
- **Trạng thái verify:** `just format-check` ✅ · `just lint` **0 warning** ✅ · `just test` ✅ · `sentrux check` ✅ · `just sim` build ✅ · **`just upload`/ESP32-S3 build ✅** (`framework=espidf`, ESP-IDF 6.1; cần venv `.venv` có pip).
- **Issue mở (→ `docs/ROADMAP.md` mục 4):** `just sim-test` — Run 1 PASS (40.46s), Run 2 DQ (LINE LOST) do feedforward/speed-envelope.
- **Đợt audit clean-code (tickets 13–18): DONE — sau đó refactor đính chính (tickets 19–22).** Giữ lại: comment hygiene (13), delete dead-code + micro-DRY (14), xóa `TofResult` (15), `Tuning` tách 2 lớp (17-struct). Đính chính: `layout`→`sensing` + normalize về đúng lớp (19), bỏ `mission.hpp` (20), bỏ `tuning.hpp` (21), tách compile DB chấp nhận blurry (22). Chi tiết: `docs/decisions/`.
- **Tickets 23–27: DONE (software).** (23) `speed_limit(κ)` + `brake_limit` — cấu trúc xong, calibration on-chip + IMU chờ phần cứng; (24) ghi `line_curve[t]`; (25) offset profile `d[t]` (placeholder); (26) continuous line belief (bỏ if/else ±1); (27) rename Linux-style. `just check` + build 2 env xanh; `sim-test` vẫn DQ (chấp nhận, robot đi xa hơn hẳn). Chi tiết: `docs/decisions/`.
- **Tickets 28–30: DONE (Sim DRY/KISS Audit & SSoT).** (28) `TrackRules` thay thế `RaceArbiter` làm SSoT duy nhất giám sát luật & chốt vòng đua (khi dừng hẳn trong đích, dừng sớm có lý do khi DQ); (29) Khử 100% nested if bằng guard-clauses & early returns; (30) Phân rã God Renderer (`draw_hud` → 3 sub-docks, tách ghost chassis, đổi tên Linux-style `poll_*`). `just check` xanh 100% (0 errors, 0 warnings); `just sim` build sạch; `test_sim` dừng chính xác tại bước va chạm (10180 steps). Chi tiết: `docs/decisions/`.
- **Tickets 31–37: DONE (HUD + first-principles tuning).** (31) HUD rem-scale; (32) curvature viz (Predictive Arc + Apex ribbon); (33) kp/kd pole-placement + gain-scheduling (xoá `steer_kp/kd`); (34) `speed_limit` bảng đo (xoá `corner_limit`) — calibration on-robot chờ phần cứng; (35) Run 1 speed = envelope đo + error-budget (xoá `base_throttle/sprint_scale`); (36) obstacle state machine `kIdle→kBrake→kCommit→kRejoin` (xoá `avoid_strength/slowdown`) — sim obstacle còn va chạm; (37) `heading_rate` optional — **phát hiện: gyro tích ψ là positive feedback**, nên ψ từ `de/ds`, gyro dành slip detection. `just check` 0 warning + build 2 env + sentrux xanh. **`sim-test`: Run 1 PASS 40.46s, Run 2 DQ (LINE LOST)** — nợ điều khiển feedforward/speed envelope. Việc còn dở: `docs/ROADMAP.md`.
