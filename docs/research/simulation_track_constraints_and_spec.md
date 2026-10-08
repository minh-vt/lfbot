# Báo cáo Nghiên cứu & Đặc tả Ràng buộc Sa bàn Mô phỏng (SSoT)

* **Mã dự án:** LF-BOT-2026
* **Tài liệu tham chiếu:** [generate_track.py](file:///home/minhvt/Projects/lfbot/firmware/assets/generate_track.py), [types.hpp](file:///home/minhvt/Projects/lfbot/firmware/src/sim/types.hpp), [sim_engine.cpp](file:///home/minhvt/Projects/lfbot/firmware/src/sim/sim_engine.cpp), [race_arbiter.cpp](file:///home/minhvt/Projects/lfbot/firmware/src/sim/race_arbiter.cpp), [manual.md](file:///home/minhvt/Projects/lfbot/docs/manual.md), [SPEC.md](file:///home/minhvt/Projects/lfbot/docs/SPEC.md), [CONTEXT.md](file:///home/minhvt/Projects/lfbot/docs/CONTEXT.md)
* **Bản vẽ gốc đối soát:** [vongloai.jpg](file:///home/minhvt/Projects/lfbot/docs/vongloai.jpg) (Sân 6000x3000 mm), [chungket.jpg](file:///home/minhvt/Projects/lfbot/docs/chungket.jpg) (Sân 10000x6000 mm)

---

## 1. Nguồn Gốc Săn Tin & Nguyên Lý Vật Lý Mô Phỏng Quang Học

### 1.1. Mô hình Cảm nhận Quang học (Optical Reflectivity Physics)
Trong [`firmware/src/sim/sim_engine.cpp:L130-L159`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/sim_engine.cpp#L130-L159), 8 cảm biến quang analog đọc độ phản xạ mặt đường thông qua phép tích phân không gian hạt nhân $3 \times 3$ trên ảnh mờ Gaussian 2px:

$$\text{brightness} = \frac{R + G + B}{3 \times 255.0}$$
$$\text{reflectivity} = (1.0 - \text{brightness})$$

* **Nền trắng chuẩn:** $R=G=B=255 \implies \text{brightness} = 1.0 \implies \text{reflectivity} = 0.0$ (không có vạch).
* **Vạch đen chuẩn:** $R=G=B=0 \implies \text{brightness} = 0.0 \implies \text{reflectivity} = 1.0$ (bắt vạch trọn vẹn).
* **Mũi tên định hướng xanh dương:** $R=0, G=102, B=204 \implies \text{brightness} \approx 0.40 \implies \text{reflectivity} \approx 0.60$. Cảm biến quang sẽ nhầm mũi tên thành vạch line dày lệch tâm!
* **Vạch đỏ ô đích / Checkpoint đỏ:** $R=220, G=20, B=20 \implies \text{reflectivity} \approx 0.65$.
* **Chữ viết, số thứ tự, đường gióng kích thước:** Bất kỳ pixel màu/xám nào trên ảnh đều biến thành "vết nhiễu quang học giả" (optical false-positives), khiến thuật toán Centroid tính sai lệch trọng tâm hàng centimet, làm rung lắc bộ lái Stanley.

**=> Kết luận bất biến 1:** Ảnh sa bàn nạp vào `SimEngine` BẮT BUỘC là nhị phân quang học (Pure White `RGB(255,255,255)` và Pure Black `RGB(0,0,0)`). Tuyệt đối **KHÔNG** chứa bất kỳ annotation, mũi tên chỉ hướng, chữ viết hay đường kích thước nào.

---

## 2. Hệ Thống Ràng Buộc Kỹ Thuật (Simulation Hard Invariants)

| Tham số | Giá trị chuẩn | Cơ sở mã nguồn (Source Citation) | Giải thích & Ràng buộc |
| :--- | :--- | :--- | :--- |
| **Tỷ lệ ảnh (Scale)** | $1\text{ px} = 1\text{ mm}$ | `firmware/assets/generate_track.py:L17` | $1.0\text{ mm/px}$ đảm bảo phép tính động học vi sai, odometry và raycast laser ToF không cần hệ số scale phức tạp. |
| **Bề rộng line** | $20.0\text{ mm}$ ($20\text{ px}$) | `manual.md:L27`, `SPEC.md:L9`, `generate_track.py:L28` | Độ rộng tiêu chuẩn băng dính đen dán trên sân thi đấu FPT. |
| **Độ mờ quang học** | Gaussian $\sigma = 2\text{ px}$ | `sim_engine.cpp:L48` | `SimEngine::load_track` tự động làm mờ ảnh khi nạp để mô phỏng sự khuếch tán ánh sáng. Ảnh gốc đầu vào phải sắc nét, mép vạch chuẩn. |
| **Kích thước vật cản** | $105 \times 60\text{ mm}$ | `manual.md:L29`, `SPEC.md:L9`, `types.hpp:L31-L32` | Khối xốp/gỗ $105 \times 60\text{ mm}$. **KHÔNG ĐƯỢC VẼ VẬT CẢN LÊN ẢNH LINE!** Vật cản được nạp dạng hình học OBB qua vector `std::vector<Obstacle>` để ToF quét và tính va chạm. |
| **Bán kính an toàn vật cản** | $R_{\text{clearance}} = 155\text{ mm}$ | `SPEC.md:L40, L110-L114`, `CONTEXT.md:L15` | Bán kính ngoại tiếp vật cản ($60.5\text{ mm}$) + nửa thân xe ($69\text{ mm}$) + đệm an toàn ($25\text{ mm}$). |
| **Ô đích (Finish Box)** | $160 \times 160\text{ mm}$ | `CONTEXT.md:L52`, `SPEC.md:L43`, `race_arbiter.cpp:L61` | Trên sa bàn thi đấu thật, đích là vạch ngang màu đỏ $240\text{ mm}$ vuông góc vạch line. Trong mô phỏng, robot phải dừng trọn vẹn trong vùng $160 \times 160\text{ mm}$ ($v \le 0.05\text{ m/s}$). |
| **Giới hạn rời line** | $t_{\text{off-line}} < 5.0\text{ s}$ | `manual.md:L39, L64`, `race_arbiter.cpp:L117-L129` | Điều 5.5 thể lệ thi đấu: rời line quá 5.0 giây bị `RaceArbiter` truất quyền thi đấu (`kLineLostTimeout`). |

---

## 3. Phân Tích Thực Địa: 2 Sa Bàn Thi Đấu FPT 2026

### 3.1. Sa bàn Vòng loại (`docs/vongloai.jpg`)
* **Kích thước sân:** $6000 \times 3000\text{ mm}$ ($6.0\text{ m} \times 3.0\text{ m}$).
* **Cấu trúc làn:** 2 làn song song độc lập (Lane A ở $y \in [0, 1500]$, Lane B ở $y \in [1500, 3000]$).
* **Mô hình đường đua:** Point-to-Point (Sprint) gấp 3 khúc dạng chữ Z:
  - Khúc 1: Chạy từ Trái qua Phải ($y = 500\text{ mm}$, $x \in [200, 5700]$).
  - Cua gập $180^\circ$ xuống Khúc 2: Chạy từ Phải qua Trái ($y = 1000\text{ mm}$, $x \in [5700, 300]$).
  - Cua gập $180^\circ$ xuống Khúc 3: Chạy từ Trái qua Phải ($y = 1350\text{ mm}$, $x \in [300, 5700]$).
  - Đích tại $x \approx 5700\text{ mm}, y \approx 1350\text{ mm}$.
* **15 Phân đoạn kỹ thuật (Lane A):**
  1. Đường thẳng xuất phát: $600\text{ mm}$
  2. S-curve ($R120$): $420\text{ mm}$
  3. Cụm vuông chồng: $480\text{ mm}$ (3 hình vuông lồng so le)
  4. Sóng sin ($R80$): $500\text{ mm}$
  5. Vùng kỹ thuật vật cản A1: $600\text{ mm}$ (vật cản $105 \times 60\text{ mm}$)
  6. Răng cưa zích zắc: $420\text{ mm}$
  7. Cua quay đầu $180^\circ$ sang Khúc 2 ($R \approx 250\text{ mm}$)
  8. Đèn đen $120 \times 120\text{ mm}$ (marker vuông đen line đi xuyên tâm): $240\text{ mm}$
  9. Sóng lệch chặt ($R70$): $480\text{ mm}$
  10. Hộp lệch hướng: $500\text{ mm}$
  11. Chicane kép ($R80$): $380\text{ mm}$
  12. Bậc vuông $\times 4$: $480\text{ mm}$ (4 bậc vuông nhấp nhô)
  13. Cụm vòng tròn chồng (5 vòng tròn đan Olympic): $480\text{ mm}$
  14. Cua quay đầu $180^\circ$ sang Khúc 3 ($R \approx 175\text{ mm}$)
  15. Vùng kỹ thuật vật cản A2: $600\text{ mm}$
  16. Răng cưa dài biên đổi: $600\text{ mm}$
  17. S-curve kép ($R90$): $520\text{ mm}$
  18. Đường thẳng nước rút về đích: $500\text{ mm}$
  19. Vạch đích: Ngang $240\text{ mm}$.

### 3.2. Sa bàn Chung kết (`docs/chungket.jpg`)
* **Kích thước sân:** $10000 \times 6000\text{ mm}$ ($10.0\text{ m} \times 6.0\text{ m}$).
* **Cấu trúc làn:** 3 làn song song đối xứng (Lane A, B, C; mỗi làn rộng $2000\text{ mm}$).
* **Các bẫy hình học bổ sung (Cực kỳ nguy hiểm):**
  1. **Cụm lục giác chồng (Honeycomb):** Kích thước $420 \times 280\text{ mm}$, gồm 4 lục giác đều ghép tổ ong (3 nhịp $140\text{ mm}$).
  2. **Uốn cong liên hoàn (Loop-the-loop $R90$):** Đường uốn tròn $360^\circ$ tự cắt chính nó tạo giao cắt ngã tư phẳng.
  3. **Cổng răng lược:** Dài $500\text{ mm}$, gồm các răng hình chữ U vuông dựng đứng.
  4. **Cụm hình thoi:** Dài $480\text{ mm}$, gồm 3 hình thoi liên tiếp tiếp xúc đỉnh.
  5. **Serpentine U hẹp:** Dài $600\text{ mm}$, uốn khúc chữ U biên độ sâu liên tục.

---

## 4. Vạch Trần Điểm Mù Kiến Trúc Giữa Sa Bàn Thật & Mã Nguồn Mô Phỏng

Sau đây là những sự thật kỹ thuật mà nếu không xử lý, việc nạp ảnh mới sẽ khiến simulation gãy ngay lập tức:

### 4.1. Điểm mù 1: Kích thước sân bị ghim cứng (Hardcoded Canvas Mismatch)
* Trong mã nguồn hiện tại [`firmware/src/sim/types.hpp:L16-L17`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/types.hpp#L16-L17):
  ```cpp
  constexpr float kTrackWidthMm = 2400.0f;
  constexpr float kTrackHeightMm = 1600.0f;
  ```
* Trong [`firmware/src/sim/race_arbiter.cpp:L104-L115`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/race_arbiter.cpp#L104-L115):
  ```cpp
  constexpr float kArenaMaxX = 2450.0f;
  constexpr float kArenaMaxY = 1650.0f;
  ```
* **Hậu quả:** Sa bàn Vòng loại có $X$ lên tới $6000\text{ mm}$, Chung kết lên tới $10000\text{ mm}$. Nếu đưa ảnh $6000\times 3000$ vào `SimEngine`, khi robot chạy vượt qua $X = 2450\text{ mm}$, `RaceArbiter` sẽ lập tức truất quyền thi đấu với lỗi `kOutOfBounds`!

### 4.2. Điểm mù 2: Cấu trúc Vòng khép kín (Loop) vs Nước rút (Point-to-Point)
* Thuật toán hiện tại trong [`generate_track.py`](file:///home/minhvt/Projects/lfbot/firmware/assets/generate_track.py) và [`sim_engine.cpp:L100-L128`](file:///home/minhvt/Projects/lfbot/firmware/src/sim/sim_engine.cpp#L100-L128) được thiết kế cho **đường đua khép kín (closed circuit)**:
  - Robot xuất phát tại $(500, 1300)$, đi 1 vòng và phanh về ô đích $(400, 1300)$.
  - Run 1 quét quỹ đạo để nối điểm đầu với điểm cuối thành spline đóng tuần hoàn.
* Sa bàn thi đấu FPT thật là **đường đua nước rút thẳng (Point-to-Point)**: Xuất phát ở bên trái ($x \approx 200$), chạy ngoằn ngoèo qua 3 tầng sân, và cán đích ở bên phải ($x \approx 5700$).
* **Hậu quả:** Bộ tối ưu quỹ đạo `GlobalTrajectoryOptimizer` và `RaceArbiter` cần được nới lỏng điều kiện biên (hỗ trợ non-cyclic open trajectories).

### 4.3. Điểm mù 3: Bẫy phân nhánh quang học (Bifurcations & Loops)
* "Cụm vòng tròn chồng" (Olympic rings) và "Cụm lục giác chồng" (Honeycomb) tạo ra **nhiều ngã ba/ngã rẽ cùng lúc**:
  - Khi xe 8 mắt IR đi vào cụm 5 vòng tròn hoặc tổ ong, thanh cảm biến sẽ nhìn thấy 2 hoặc 3 vạch đen tách đôi.
  - Bộ điều khiển Centroid thuần túy sẽ bị nhiễu loạn: trọng tâm nhảy qua lại giữa 2 nhánh (centroid bifurcation jump), khiến xe giật lái hoặc xoay tròn luẩn quẩn trong một vòng khép kín.
  - Cần có chiến lược: (1) Đi theo nhánh biên tối ưu, hoặc (2) Giữ nguyên hướng la bàn (yaw hold) khi phát hiện phân nhánh.

### 4.4. Điểm mù 4: Góc vuông gấp khúc $90^\circ$ (Square Waves)
* "Bậc vuông x4" và "Cổng răng lược" chứa các góc vuông $90^\circ$ tuyệt đối với bán kính $R = 0\text{ mm}$.
* Về mặt toán học điều khiển xe đua:
  $$v_{\text{grip}} = \sqrt{\frac{\mu g}{|\kappa| + \epsilon}} \xrightarrow{\kappa \to \infty} 0$$
* Xe đua tốc độ cao quán tính lớn không thể bo cua góc vuông $90^\circ$ nếu không có cơ chế phát hiện vạch vuông ngang để chủ động phanh khẩn cấp (hard deceleration) hoặc cắt góc trong hành lang cho phép.

---

## 5. Chiến Lược Thực Thi Chuẩn Hóa

1. **Bộ sinh sa bàn Python chuẩn hóa (`generate_competition_track.py`):**
   - Viết script Python dựng hình học giải tích chuẩn (1 px = 1 mm) cho cả Sa bàn Vòng loại ($6000 \times 3000\text{ mm}$) và Chung kết ($10000 \times 6000\text{ mm}$).
   - Hỗ trợ xuất:
     - Toàn bộ sân thi đấu (Full Arena).
     - Từng làn riêng lẻ (Lane A: $6000 \times 1500\text{ mm}$, phù hợp nạp mô phỏng độc lập).
   - Xuất ảnh nhị phân: Nền trắng `#FFFFFF`, Line đen 20mm `#000000`, ô đích đen, **KHÔNG CÓ ANNOTATION**.
2. **Quản lý dữ liệu vật cản tách biệt:**
   - Tạo tệp JSON/C++ chứa tọa độ vật cản chuẩn $(x, y, w, h, \theta)$ tương ứng với các vùng kỹ thuật trên bản vẽ để nạp động vào `SimEngine::set_obstacles()`.
