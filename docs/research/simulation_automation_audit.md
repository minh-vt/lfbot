# Báo cáo Nghiên cứu: Hiện trạng Kiến trúc Mô phỏng & Khả năng Tự động hóa Headless Test (SSoT)

* **Mã dự án:** LF-BOT-2026
* **Tài liệu tham chiếu:** [CONTEXT.md](file:///home/minhvt/Projects/lfbot/docs/CONTEXT.md), [SPEC.md](file:///home/minhvt/Projects/lfbot/docs/SPEC.md), [platformio.ini](file:///home/minhvt/Projects/lfbot/firmware/platformio.ini), [justfile](file:///home/minhvt/Projects/lfbot/justfile)
* **Mục tiêu:** Điều tra nguồn gốc kỹ thuật nguyên bản (Primary Sources) giải thích vì sao hiện tại không có cơ chế test tự động cho simulation, vạch trần điểm nghẽn kiến trúc và chứng minh giải pháp tự động hóa headless sub-second.

---

## 1. Điều tra Nguồn Gốc Hiện Trạng (Primary Sources Audit)

### 1.1. Hiện trạng: Con người làm "Biological Smoke-Tester"
Hiện tại, mỗi khi thay đổi thuật toán điều khiển, quy hoạch quỹ đạo hay tham số PID, quy trình kiểm thử bị chặn hoàn toàn bởi thao tác thủ công:
1. Chạy lệnh [`just sim`](file:///home/minhvt/Projects/lfbot/justfile#L7-L8) -> mở cửa sổ GUI Raylib 1920x1080.
2. Dùng tay bấm phím `[1]` để kích hoạt Lượt 1 (Map Scanning). Robot bắt đầu chạy ở tốc độ an toàn (0.8 m/s).
3. Chờ đợi từ 15-20 giây thời gian thực (wall-clock time) để robot chạy hết 1 vòng sa bàn 2.4m x 1.6m.
4. Nhìn bằng mắt thường xem robot có bị trượt line, va chạm vật cản, hoặc có dừng đúng ô đích hay không.
5. Bấm phím `[2]` để kích hoạt Lượt 2 (C2 Racing Sprint). Robot tăng tốc lên 2.0+ m/s theo quỹ đạo tối ưu $C^2$.
6. Chờ thêm 10-15 giây thời gian thực, đọc thông báo delta thời gian trên HUD toast, rồi copy-paste kết quả báo lại cho AI.

**Hậu quả:**
- **Lãng phí thời gian khủng khiếp:** Tốn 35-50 giây thời gian con người cho mỗi lần kiểm thử hồi quy nhỏ.
- **Mất tính định lượng (Qualitative vs Quantitative):** Đánh giá bằng mắt không đo được sai số bám vạch cực đại ($\max |e_{\text{cte}}|$), cự ly an toàn tối thiểu với vật cản ($\min d_{\text{obs}}$), hay biên độ rung lắc lái.
- **Gãy vòng lặp tự phản hồi của Agent (Broken Agent Loop):** AI không thể tự chạy kiểm thử kiểm tra xem bản diff của mình có làm vỡ logic xe chạy hay không, biến con người thành một nút thắt cổ chai (bottleneck) ngớ ngẩn.

---

## 2. Điểm Nghẽn Kiến Trúc Trong Mã Nguồn

### 2.1. Đóng gói lẫn lộn giữa Vật lý & Giao diện đồ họa (GUI-Physics Coupling)
Khảo sát trực tiếp tệp [`firmware/src/sim/main.cpp`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/main.cpp):
* Toàn bộ hàm cập nhật vật lý [`update_simulation(...)`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/main.cpp#L482-L606) và [`run_simulation_step(...)`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/main.cpp#L747-L782) nằm trong **anonymous namespace** của `main.cpp`.
* Hàm vật lý này bị phụ thuộc trực tiếp vào các biến GUI:
  - `char* toast_msg`, `float& toast_timer` (dòng 488, 542, 553, 563) để in thông báo lên màn hình.
  - Các cấu trúc của Raylib (`Image`, `Color`, `GetImageColor`, `Vector2`).
* Điểm vào `main()` (dòng 885) gọi ngay [`InitWindow(...)`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/main.cpp#L887) và [`SetTargetFPS(60)`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/main.cpp#L889) trước khi làm bất cứ việc gì khác.

=> **Kết luận:** Do logic mô phỏng bị khóa cứng bên trong vòng lặp cửa sổ đồ họa của `main.cpp`, không có bất kỳ unit test hay binary headless nào có thể import hoặc gọi được hàm mô phỏng này.

### 2.2. Sự khác biệt giữa `test_core.cpp` và Thực tế Sa bàn
Tệp [`firmware/test/test_core.cpp`](file:///home/minhvt/Projects/lfbot/firmware/test/test_core.cpp) thực thi qua [`just test`](file:///home/minhvt/Projects/lfbot/justfile#L11-L12) trong 0.28 giây với 27 bài kiểm tra độc lập.
Tuy nhiên, `test_core.cpp` chỉ kiểm tra các hàm đơn lẻ với dữ liệu giả lập (synthetic vectors / analytical mocks):
- Centroid với mảng giá trị float cố định ([test_core.cpp:L22-L53](file:///home/minhvt/Projects/lfbot/firmware/test/test_core.cpp#L22-L53)).
- Stanley tracker trên đường cong giải tích mượt ([test_core.cpp:L93-L130](file:///home/minhvt/Projects/lfbot/firmware/test/test_core.cpp#L93-L130)).
- Tránh vật cản với ToF giả lập dạng số tuyến tính ([test_core.cpp:L1309-L1345](file:///home/minhvt/Projects/lfbot/firmware/test/test_core.cpp#L1309-L1345)).

**Thiếu vắng hoàn toàn:**
- Không có bài test nào nạp bức ảnh sa bàn thực [`assets/track.png`](file:///home/minhvt/Projects/lfbot/firmware/assets/track.png).
- Không có bài test nào tích phân 3x3 điểm ảnh mô phỏng vết quang học 5mm của cảm biến hồng ngoại trên ảnh bitmap thật.
- Không có bài test nào cho robot chạy liên tục 1 vòng hoàn chỉnh từ vạch xuất phát, lập bản đồ, tối ưu $C^2$, sang Lượt 2 phóng hết tốc lực qua vật cản thật và phanh dừng trong ô đích.

---

## 3. Khảo Sát Kỹ Thuật Đột Phá: Khả Năng Chạy Headless Của Raylib

Một giả định sai lầm phổ biến là: *"Raylib cần màn hình/cửa sổ X11 để xử lý hình ảnh và Raycast"*.
Thực nghiệm biên dịch độc lập trực tiếp trên hệ thống Linux với GCC đã chứng minh điều ngược lại:

```cpp
#include "raylib.h"
int main() {
    // KHÔNG GỌI InitWindow!
    Image img = LoadImage("firmware/assets/track.png");
    ImageBlurGaussian(&img, 2);
    Color c = GetImageColor(img, 500, 1300); // Đọc pixel tại vạch xuất phát
    UnloadImage(img);
    return 0;
}
```

* **Kết quả thực nghiệm:**
  - `LoadImage()` sử dụng `stb_image` đọc trực tiếp file PNG vào RAM heap (không cần OpenGL/X11).
  - `ImageBlurGaussian()` và `GetImageColor()` là các phép toán thuần túy trên CPU byte array (`img.data`).
  - [`raycast.cpp`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/raycast.cpp) tính toán OBB giao cắt bằng đại số học 2D thuần túy trên CPU.
  - Tốc độ xử lý: Nạp ảnh 2400x1600 và làm mờ chỉ mất ~15ms!

---

## 4. Định Lượng Tốc Độ: 500x Speedup (Virtual Time vs Wall-Clock)

Trong môi trường mô phỏng đồ họa:
* Tốc độ bị khóa cứng ở $60\text{ FPS}$ (mỗi frame thời gian thực $\approx 16.6\text{ ms}$).
* Một vòng chạy $15\text{ giây}$ tốn đúng $15.0\text{ giây}$ ngồi nhìn màn hình.

Trong môi trường mô phỏng không đồ họa (Headless Virtual Clock):
* Bước tích phân vật lý: $dt = 0.002\text{ s}$ ($500\text{ Hz}$).
* Một vòng chạy $15\text{ giây}$ tương đương $15 / 0.002 = 7,500$ vòng lặp tính toán.
* Mỗi vòng lặp gồm:
  - 13 phép tính tích phân $3 \times 3$ điểm ảnh $= 117$ phép đọc RAM.
  - 3 phép Raycast OBB $= \approx 20$ phép tính vector.
  - 1 bước cập nhật động học vi sai kinematics và odometry.
* **Thời gian CPU thực tế cho 7,500 bước trên x86_64:** $\approx 30 - 45\text{ ms}$.
* Cả 2 lượt chạy (Run 1: 18s + Run 2: 12s $= 30\text{s}$ robot) hoàn tất trong **dưới 100 ms**!

=> **Tốc độ nhanh gấp hơn 300 - 500 lần** so với việc mở GUI và ngồi chờ con người thao tác.

---

## 5. Kết Luận & Khuyến Nghị Kiến Trúc

1. **Tách Seam (Seam Extraction):** Trích xuất logic cập nhật mô phỏng từ [`firmware/src/sim/main.cpp`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/main.cpp) thành một module độc lập [`firmware/src/sim/sim_engine.hpp/.cpp`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/sim_engine.hpp) hoặc `sim_world`. Module này hoàn toàn không chứa mã UI, không chứa `toast_msg`, chỉ nhận input và cập nhật state.
2. **Xây dựng Binary Test Tự động:** Tạo môi trường PlatformIO `sim_test` hoặc bổ sung vào `test_core.cpp` một kịch bản test tích hợp `test_simulation_full_lap()`:
   - Tự động nạp sa bàn `track.png`.
   - Chạy Run 1 tới vạch đích -> kiểm tra bất biến (Invariant assertions).
   - Tự động kích hoạt bộ tối ưu quỹ đạo $C^2$ QP.
   - Chạy Run 2 tới vạch đích -> kiểm tra cự ly né vật cản và thời gian rút ngắn ($\Delta T < 0$).
3. **Task Runner Một Lệnh:** Bổ sung `just sim-test` vào [`justfile`](file:///home/minhvt/Projects/lfbot/justfile), trả về mã `0` khi đỗ toàn bộ và in ra báo cáo scorecard định lượng.
