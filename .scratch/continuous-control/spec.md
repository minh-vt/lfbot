# Spec: LF-Bot F-Maze-Solving — bám line liên tục qua góc vuông

> Trạng thái: chốt từ hội thoại (first-principles, không số bịa).

## Problem Statement

Robot ≤16×16cm theo MỘT line đen 2cm liên tục từ start → finish, vẽ kiểu mê cung (tự cắt: ngã ba T, ngã tư +, ô vuông lồng nhau, vòng tròn Olympic, cua vuông 90°), track BÍ MẬT. Trên tim line có gạch 205×95×95mm đặt ngẫu nhiên. 2 lượt, lấy lượt tốt nhất, ngoài trời có gió. Ra khỏi line được phép; "cắt line" (đi tắt qua đoạn line KHÁC) = DQ.

Không có ngõ cụt, không nhánh sai — line liên tục nên chạy liên tục là tới đích. Robot KHÔNG giải mê cung, KHÔNG chọn nhánh, KHÔNG backtrack.

## First principles

- **Vật lý quyết định:** tốc độ qua cua `v ≤ √(μgR)` — không phụ thuộc động cơ, không phụ thuộc code. Người thắng = lốp (μ) + nhẹ + phanh đúng lúc + không mất line.
- **"Tối ưu số lần rẽ" = cực đại thời gian `L/R = 1` ở `v_max`.** Suy ra: mọi cua < ~412° thì nới càng rộng càng nhanh → racing line = nới mỗi cua tới bán kính hợp lệ tối đa (biên = đoạn line lân cận, lấy từ map Run 1).

## Phần cứng (quyết định thiết kế — KHÔNG phải tham số controller)

- Thanh IR **CONG** theo cung rẽ của xe: `R ≈ wheelbase/2`, độ lệch lookahead trước/sau ≈ `s²/wheelbase`. Chọn một lần lúc chế tạo.
- **Vì sao cong:** ở góc vuông 90°, đoạn line sau cua SONG SONG với hàng thẳng → smear rồi mù. Cong → mỗi sensor ở lookahead khác → luôn đo sạch, và tự sinh tín hiệu "line phía trước" vs "line bên".
- Encoder 2 motor (odometry + tỉ số L/R), IMU (hướng + bắt trượt), ToF (gạch).

## Điều khiển (tầm thường — không tracking, không FSM, không lọc)

Curved sensor làm controller trở nên tầm thường:

- **Có line phía trước** → tự cân bằng với 2 sensor bên → **đi thẳng**.
- **Không có line phía trước** → **tự rẽ** (về phía sensor bên còn thấy line).

Hệ quả tự nhiên:
- Điểm tự cắt (có line phía trước + line ngang) → đi thẳng, bỏ line ngang (đây là "branch prioritize": ưu tiên line phía trước).
- Cua 90° (line phía trước biến mất, line ở bên) → tự rẽ.

Hình học cong làm hết việc; controller chỉ "cân bằng line".

## Chiến lược 2-pass

- **Run 1:** theo line + ghi đường bằng odometry + ghi vị trí gạch bằng ToF. Không quyết định, không backtrack.
- **Run 2:** chạy lại racing line từ map: nới cua tới biên corridor, cực đại `L/R=1` ở `v_max`, phanh theo `√(μgR)` trước cua, IMU bắt trượt.

## Né vật cản (gạch 205mm)

Line "uốn quanh" gạch: ToF thấy gạch chặn line → tạo **line ảo** uốn quanh gạch (hình học gạch + clearance = nửa bề rộng xe) rồi nối lại line thật. Cùng cơ chế "bám line" — không FSM, không magic number.

## Nguyên tắc xuyên suốt

Mọi thứ suy từ sự thật (góc 90°, line 2cm, xe ≤16cm, luật cắt line) — không số bịa. Phần cứng là quyết định thiết kế; controller là bộ cân bằng đơn giản; 2-pass để Run 2 chạy racing line.

## Out of scope (đã bác trong hội thoại)

- Trường thế liên tục / potential field.
- Temporal tracking / lọc / dự đoán state.
- State machine né vật cản (`AvoidPhase`).
- Pole-placement gain scheduling (`kp ∝ 1/v²`).
- Backward-pass speed envelope / `SpeedLimitTable` đo.
- κ-planning / đo độ cong từ steer.
- Multi-pass pipeline (stage-pass-manager).
- Đo "corridor" bằng cảm biến mở rộng (corridor lấy từ map Run 1).
