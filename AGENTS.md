# AGENTS.md - LF-Bot 2026 Autonomous Racing Platform

Tài liệu định nghĩa ngữ cảnh, quy tắc kiến trúc và hợp đồng phát triển cho AI Agents & Codex trong dự án LF-Bot.

---

## 1. Tổng quan Dự án & Kiến trúc (Context Hierarchy Level 1)
Robot tự hành dò line tốc độ cao và vượt chướng ngại vật thi đấu "F-Maze-Solving" (Đại học FPT).
Mã nguồn được phân tách theo mô hình kiến trúc sâu (Deep Modules) với ranh giới phân tầng nghiêm ngặt:
- **`firmware/lib/`**: Lõi C++17 độc lập phần cứng/OS — `kernel/` (thuật toán thuần, không-thứ-nguyên: `step`, `plan`) + `layout/` (chân lý phần cứng dùng chung: vị trí 8 cảm biến IR). Không phụ thuộc Arduino, FreeRTOS hay Raylib.
- **`firmware/src/sim/`**: Môi trường mô phỏng máy tính để bàn (Raylib 2D), nhận diện vạch quang học (8 cảm biến IR) và ToF trên sa bàn 6000×1500 mm (vòng loại) / 10000×1500 mm (chung kết), 1 px = 1 mm.
- **`firmware/src/esp32/`**: Firmware nhúng chạy trên vi điều khiển ESP32-S3 FreeRTOS (dual-core 240MHz, ADC-DMA, PCNT, MCPWM).
- **`firmware/test/`**: Bộ kiểm thử tự động không đồ họa (Headless Native Tests).
- **`.scratch/architecture-redesign/spec.md`**: Spec kiến trúc duy nhất (SSoT) — kernel không-thứ-nguyên + adapters (sim, esp32). Ticket triển khai tại `.scratch/architecture-redesign/issues/`.

---

## 2. Tech Stack & Công Cụ
- **Ngôn ngữ:** C++17 (Google C++ Style, 2-space indentation).
- **Build System & Toolchain:** PlatformIO Core (Native GCC cho Desktop/Test, Espressif32 toolchain cho ESP32-S3).
- **Task Runner:** `just` (quản trị toàn bộ lệnh thực thi).
- **Linter & Formatter:** `clang-format` cho toàn bộ C++ (gồm `src/esp32`); `clang-tidy` cho code build được trên host (kernel/sim/test/header thuần) và `cppcheck` cho firmware ESP32-S3 (IDF không parse được bằng clang-tidy host). Policy 0 errors, 0 warnings.
- **Đồ họa & Mô phỏng:** Raylib Desktop (hỗ trợ cả GUI và chế độ xử lý ảnh Headless).
- **Thư viện bên thứ ba:** `nlohmann-json` — chỉ dùng trong sim để parse manifest track; kernel và esp32 không phụ thuộc.
- **ESP32:** `framework = espidf` (ESP-IDF 6.1, bỏ Arduino). FreeRTOS task 500Hz (`vTaskDelayUntil` 2ms, core 1, entry `app_main`) + API IDF mới: `esp_adc/adc_oneshot.h`, `driver/pulse_cnt.h`, `driver/mcpwm_prelude.h`, `driver/i2c_master.h`. `src/CMakeLists.txt` chỉ biên dịch `esp32/*` (không `src/sim`). Build cần venv `uv` (`.venv`) có pip; env esp32s3 dùng `build_unflags=-std=gnu++11`.

---

## 3. Danh Mục Lệnh Tiêu Chuẩn (Command Center)
Mọi agent BẮT BUỘC sử dụng các lệnh qua `just`:

```bash
just test         # Chay toan bo unit tests headless (< 0.3s)
just sim-test     # Chay kiem thu mo phong headless 2 luot (Map & Sprint) (< 0.5s)
just check        # Kiem tra toan dien: format + lint (native + esp32) + test
just sim          # Chay mo phong GUI Raylib Desktop (danh cho nguoi dung quan sat)
just format       # Tu dong format toan bo ma nguon C++ bang clang-format
just format-check # Kiem tra format (dung cho CI/CD)
just lint         # Kiem tra tinh: clang-tidy (native) + cppcheck (esp32)
just lint-native  # clang-tidy voi firmware/compile_commands.json (kernel/sim/test)
just lint-esp32   # cppcheck cho firmware ESP32-S3 (pio check -e esp32s3)
just lint-fix     # Tu dong sua loi clang-tidy va format lai code
just compiledb    # Cap nhat database bien dich compile_commands.json
just track        # Sinh sa ban procedural ngau nhien (generate_track.py)
```

---

## 4. Các Ranh Giới Bất Biến (Strict Boundaries & Non-negotiables)
1. **Không bao giờ trộn mã GUI vào Core Library hoặc Physics:**
   - Thư viện trong `firmware/lib/` KHÔNG BAO GIỜ include `raylib.h`, không bao giờ dùng `printf`, không dùng biến toàn cục.
   - Vòng lặp vật lý mô phỏng (`SimEngine`) phải luôn chạy được ở chế độ **Headless (không mở cửa sổ)**.
2. **Quy tắc Kiểm thử Tự động (Verification Contract):**
   - Tuyệt đối KHÔNG yêu cầu người dùng phải tự bật `just sim` để làm "smoke-test" bằng mắt.
   - Mọi thay đổi liên quan đến thuật toán điều khiển, bám line, né vật cản, quy hoạch $C^2$ phải được kiểm chứng tự động bằng `just test` và `just sim-test`.
3. **Chuẩn mã nguồn sạch:**
   - 100% mã nguồn C++ tuân thủ quy tắc 2-space của `.clang-format`.
   - Biên dịch với cờ `-Wall -Wextra -Wpedantic` không phát sinh bất kỳ cảnh báo nào.
4. **Bảo tồn thuật ngữ SSoT:**
   - Tuân thủ nghiêm ngặt các khái niệm định nghĩa trong `docs/CONTEXT.md`: `RawTrace`, `TrackCorridor`, `RacingTrajectory`, `LateApex`, `FrictionEnvelope`, `Centroid`, `TrajectoryFollower`, `RobotCoordinator`.

---

## 5. Quy Trình Làm Việc Của Agent (Thinking Framework)
1. **Trước khi sửa code:** Khảo sát các seam liên quan, xác định tiêu chí thành công cụ thể (ví dụ: test nào sẽ pass).
2. **Sửa code tinh gọn (Surgical Changes):** Chỉ chạm đúng những dòng cần sửa, không tái cấu trúc mã xung quanh không liên quan.
3. **Xác minh độc lập (Independent Verification):**
   - Chạy `just test`
   - Chạy `just format-check`
   - Báo cáo kết quả định lượng kèm theo số liệu đo đạc, không phán đoán mơ hồ.
