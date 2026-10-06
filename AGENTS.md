# AGENTS.md - LF-Bot 2026 Autonomous Racing Platform

Tài liệu định nghĩa ngữ cảnh, quy tắc kiến trúc và hợp đồng phát triển cho AI Agents & Codex trong dự án LF-Bot.

---

## 1. Tổng quan Dự án & Kiến trúc (Context Hierarchy Level 1)
Robot tự hành dò line tốc độ cao và vượt chướng ngại vật thi đấu "F-Maze-Solving" (Đại học FPT).
Mã nguồn được phân tách theo mô hình kiến trúc sâu (Deep Modules) với ranh giới phân tầng nghiêm ngặt:
- **`firmware/lib/`**: Lõi giải thuật C++17 độc lập hoàn toàn với phần cứng và hệ điều hành (không phụ thuộc Arduino, FreeRTOS hay Raylib).
- **`firmware/src/sim/`**: Môi trường mô phỏng máy tính để bàn (Raylib 2D SSoT), nhận diện vạch quang học và Raycast laser ToF trên sa bàn 2400x1600 mm.
- **`firmware/src/esp32/`**: Firmware nhúng chạy trên vi điều khiển ESP32-S3 FreeRTOS (dual-core 240MHz, ADC-DMA, PCNT, MCPWM).
- **`firmware/test/`**: Bộ kiểm thử tự động không đồ họa (Headless Native Tests).
- **`docs/`**: Nguồn chân lý duy nhất (SSoT) về thuật ngữ nghiệp vụ ([CONTEXT.md](file:///home/minhvt/Projects/lfbot/docs/CONTEXT.md)) và đặc tả kỹ thuật ([SPEC.md](file:///home/minhvt/Projects/lfbot/docs/SPEC.md)).

---

## 2. Tech Stack & Công Cụ
- **Ngôn ngữ:** C++17 (Google C++ Style, 2-space indentation).
- **Build System & Toolchain:** PlatformIO Core (Native GCC cho Desktop/Test, Espressif32 toolchain cho ESP32-S3).
- **Task Runner:** `just` (quản trị toàn bộ lệnh thực thi).
- **Linter & Formatter:** `clang-format`, `clang-tidy` (0 errors, 0 warnings policy).
- **Đồ họa & Mô phỏng:** Raylib Desktop (hỗ trợ cả GUI và chế độ xử lý ảnh Headless).

---

## 3. Danh Mục Lệnh Tiêu Chuẩn (Command Center)
Mọi agent BẮT BUỘC sử dụng các lệnh qua `just`:

```bash
just test         # Chay toan bo unit tests headless (< 0.3s)
just sim-test     # Chay kiem thu mo phong headless 2 luot (Map & Sprint) (< 0.5s)
just check        # Kiem tra toan dien: format + lint + test + sim-test
just sim          # Chay mo phong GUI Raylib Desktop (danh cho nguoi dung quan sat)
just format       # Tu dong format toan bo ma nguon C++ bang clang-format
just format-check # Kiem tra format (dung cho CI/CD)
just lint         # Kiem tra tinh bang clang-tidy voi firmware/compile_commands.json
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
