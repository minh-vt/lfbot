# DAC TA KY THUAT: ROBOT TU HANH DO LINE VA NE VAT CAN

* Ma du an: LF-BOT-2026
* Cuoc thi: F-Maze-Solving (Dai hoc FPT)
* Tieu chuan tham chieu: The le cuoc thi (manual.md)

## 1. Bai toan ky thuat

Robot can tu hanh di chuyen tren sa ban nen trang, vach den rong 2.0 cm, vuot qua cac chuong ngai vat 10.5 x 6.0 cm va ve dich trong thoi gian ngan nhat.
Quy dinh cuoc thi dat ra cac gioi han:

1. Ranh gioi pham quy: Neu roi khoi vach qua 5 giay se bi huy luot chay (Dieu 5.5). Kich thuoc vat can lon doi hoi quy dao ne phai chinh xac.
2. Kich thuoc va dien ap: Khung bao khong vuot qua 16 x 16 cm. Dien ap nguon pin khong qua 12V tai thoi diem check-in (Dieu 3.2, 3.3).
3. Chien thuat 2 luot chay: Ket qua tinh theo luot nhanh nhat. Neu chi dung giai thuat phan xa thuan tuy, robot kho dat toc do toi da o doan thang vi can phong ngua truot cua.

Muc tieu la che tao robot co phan cung on dinh, tich hop thuat toan dieu khien bam quy dao Stanley Tracker, toi uu hoa loi QP toan cuc (Global Convex QP Trajectory Optimizer), va lap ban do sa ban o Luot 1 de toi uu toc do o Luot 2.

## 2. Giai phap tong the

1. Co khi: Khung xe 145 x 138 mm, trong tam tap trung 70% vao truc banh chu dong phia sau de tang ma sat, banh ty bi cau phia truoc. Thanh cam bien cach truc banh 55 mm (co khe truot tinh chinh len 80 mm) va co man chan sang ngoai vi.
2. Phan cung mach in: Bo mach chu dua tren ESP32-S3 WROOM-1, mach cau H kep A4950/DRV8871 chiu dong dinh 3.5A ho tro phanh chu dong, cum 3 cam bien laser ToF (truoc 0 do, trai -90 do, phai +90 do) voi re quat quang hoc 21 tia (FoV 25 do), thanh 8 mat quang analog doc qua ADC-DMA, va man hinh OLED 0.96 inch cho giao dien van hanh.
3. Nguon dien: Pin LiPo 2S 7.4V (day 8.4V). Nguon pin cap truc tiep cho dong co; mach buck ha ap xuong 5V va LDO on ap 3.3V cap rieng cho vi dieu khien va cam bien de han che nhieu sut ap.
4. Dieu khien va phan mem:
   * Stanley Tracker & Speed-Scaled PID: Luot 1 bam vach bang PID vi tri tam Centroid ket hop phanh quang hoc dong hoc (Optical Dynamic Braking) giam toc muot ma vao cua; Luot 2 bam quy dao dua toi uu bang bo dieu khien phi tuyen Stanley ket hop bu goc lai dong hoc Kinematic Feedforward va dao chieu banh xe (Counter-Rotation).
   * Quy hoach ne vat can QP & Cung ne dong: Luot 1 ne phan xa bang cung luon dong muot ma (TacticalBehaviorArbiter) khi ToF truoc phat hien vat can trong cu ly 35 cm; Luot 2 ne hoan toan bang quy dao toi uu toan cuc QP giai truoc dua tren ban kinh bao ngoai tiep 155 mm.
   * Lap ho so van toc 2-Pass (Velocity Profiler): Tinh toan gioi han gia toc tien va phanh lui dua tren vong tron ma sat Kamm cung phong bi bao hoa dong hoc vi sai lien tuc $v_{\text{diff\_limit}}$, toi uu hoa thoi gian chay toan sa ban ma khong dung if-else roi rac.

## 3. Quy trinh nguoi dung va yeu cau he thong

1. Check-in ky thuat: Robot co kich thuoc 145 x 138 mm va pin LiPo 2S 8.4V de vuot qua khau do kich thuoc (< 16 cm) va do dien ap (< 12V).
2. Thao tac tai vach xuat phat: Man hinh OLED va hai nut bam (SELECT, START) cho phep chon che do Calibrate, Run 1 hoac Run 2 duoi 5 giay ma khong can ket noi may tinh.
3. Giam sat dien ap: Man hinh OLED hien thi dien ap pin voi sai so duoi 0.05V de phat hien kip thoi khi pin yeu.
4. Khang nhiễu quang hoc: Man chan quang hoc cach mat san 2 mm bao quanh 8 mat cam bien giup ngan anh sang den chieu xiên lam lech gia tri do.
5. Thu thap du lieu mat line: Khoi GDMA lay mau 8 kenh analog o tan so 10 kHz tong the ma khong lam tieu ton thoi gian xu ly cua CPU.
6. Noi suy tam Centroid: Thuat toan tinh trong tam noi suy vi tri vach den voi do phan giai duoi 0.5 mm, tranh gay giat lai.
7. Dem xung encoder va Ty le quang duong: Khoi PCNT dem 4x xung quadrature cua 2 dong co truc tiep bang phan cung; quy doi quang duong bang he so hieu chuan thuc nghiem `meters_per_tick_scale = 0.00019074f` (ho tro `calibrate(distance, ticks)`), triet tieu gia dinh suy doan ve ti so truyen va duong kinh lop.
8. Vong lap van toc: Chay o chu ky 2 ms (500 Hz) tren Core 1 cua ESP32-S3 giup duy tri van toc banh xe on dinh khi dien ap pin thay doi tu 8.4V xuong 7.4V.
9. Quet vat can phia truoc: ToF phia truoc quet voi tan so 50 Hz, re quat quang hoc 21 tia (FoV 25 do) de nhan dien vat can o cu ly 35 cm va chu dong phanh/ne tranh.
10. Gioi han thoi gian ne: Cung ne dong Luot 1 va Quy dao toi uu Luot 2 luon dam bao bat lai line trong duoi 2.5 giay, khong vi pham gioi han 5.0 giay cua Dieu 5.5.
11. Vung an toan ngoai tiep: Duy tri ban kinh an toan ngoai tiep R_clearance ~ 155 mm bao ve xe truoc vat can o moi goc xoay [0, 360 do].
12. Ghi nhan ban do: Luot 1 luu vet toa do khong gian RawTrace (x, y, theta, s) va vi tri vat can phat hien vao RAM/NVS Flash.
13. Giam sat Trong tai doc lap (RaceArbiter): Vong dua duoc xac thuc boi cac cong kiem soat SectorGate co huong; bat loi di sai chieu, cat sa ban hoac mat line qua 5 giay.
14. Phanh dung trong o dich (FinishBox): Kich hoat phanh chu dong cua cau H de robot dung gon ben trong o dich 160 x 160 mm voi van toc duoi 0.05 m/s (Dieu 4.6).
15. Chay Luot 2: Robot dung quy dao toi uu loi QP toan cuc (GlobalTrajectoryOptimizer) voi hanh lang tuy chinh len toi 120 mm (`Max Dev`, mac dinh 80 mm) va Stanley Tracker de tang toc toi da tren doan thang va giam toc muot ma truoc cua.

## 4. Quyet dinh thiet ke chi tiet

### 4.1. Co khi

* Kich thuoc: Dai 145 mm, rong 138 mm, cao 55 mm.
* Khung tam day: In 3D bang PETG, day 3.0 mm, do dien 50% dang Gyroid. Cac lo oc dung insert dong ren nhiet M2 va M3.
* Vi tri trong tam: Dong co va pin dat lui ve sau, cach truc banh khoang 15 mm de 70% trong luong de len hai banh chu dong.
* Banh xe: Banh silicon duc hoac cao su bam duong kinh 34 mm, be rong 10 mm. Banh ty truoc la bi mat trau mini phi 12 mm.
* Cum cam bien: Tam thanh 8 mat cach truc banh chu dong 55 mm tren cau hinh dong hoc phan mem (`sensor_lookahead_m = 0.055f`), ranh truot co khi cho phep tinh chinh do vuon tu 50 mm den 80 mm va do cao tu 3.5 mm den 6.0 mm.
* Bo tri ToF: ToF truoc dat cao 25 mm so voi san; ToF trai va phai dat o hai mep suon goc 90 do.

### 4.2. Mach dien

* Vi dieu khien: Module ESP32-S3-WROOM-1, cong USB Type-C noi voi GPIO 19 (D-) va GPIO 20 (D+) de nap va debug truc tiep.
* Mach cau H: 2 IC A4950 hoac DRV8871 kem pad tich hop via tan nhiet xuong mat duoi. Moi kenh dieu khien bang 2 chan PWM ho tro che do chay va phanh chu dong.
* Thanh cam bien line: 8 cap LED phat 940 nm va phototransistor (pitch 9.0 mm, tong be rong 63 mm). Tin hieu analog noi vao ADC1 (GPIO 1 den GPIO 8).
* ToF: 3 module VL53L0X dung chung bus I2C (SDA GPIO 11, SCL GPIO 12). Cac chan XSHUT noi rieng biet de boot tuan tu va gan lai dia chi I2C.
* Phan phoi nguon: Pin LiPo 2S qua cau chi 5A cap truc tiep cho motor. Mach buck MP1584 ha xuong 5.0V / 2A, LDO AP2112K ha xuong 3.3V / 600mA cho MCU va cam bien.
* Giao dien: OLED 0.96 inch I2C, 2 nut bam chong rung phan cung kem tro keo len, va 1 buzzer mini.

### 4.3. Phan mem va phan bo tac vu

*(Ghi chu kien truc: Day la dac ta kien truc phan cung dich tren ESP32-S3. Hien tai logic giai thuat loi da hoan thien va kiem chung tren Native/SimEngine; phan firmware nhung ESP32-S3 dang duoc duy tri khung va tich hop theo tung giai doan).*

1. PCNT: Cau hinh 2 kenh dem phan cung cho banh trai (GPIO 15, 16) va banh phai (GPIO 17, 18), giai ma quadrature 4x.
2. MCPWM: Tan so phat xung 25 kHz tranh tieng rit am hoc, do phan giai 10-bit.
3. ADC-DMA: Quet lien tuc tren 8 kenh ADC1 luu vao bo dem ring thong qua GDMA.
4. Phan bo FreeRTOS:
   * Task dieu khien chuyen dong (Core 1, uu tien 5, chu ky 2 ms): Doc ADC, tinh Centroid, doc PCNT, chay Stanley Tracker / PID, xuat MCPWM.
   * Task may trang thai va ban do (Core 0, uu tien 3, chu ky 20 ms): Doc ToF, chay Dynamic Evasion / Planner, cap nhat odometry va doan duong.
   * Task giao dien (Core 0, uu tien 1, chu ky 100 ms): Doc dien ap pin, cap nhat OLED va dieu khien buzzer.

### 4.4. Thuat toan dieu khien

#### 4.4.1. Trong tam vach (Centroid)

Gia tri 8 mat duoc chuan hoa:
$$V_{norm}[i] = \frac{V_{raw}[i] - V_{min}[i]}{V_{max}[i] - V_{min}[i]} \quad (i = 0 \dots 7)$$

Voi buoc mat 9.0 mm, trong so vi tri milimet cua 8 mat la:
$$W = \{-31.5, -22.5, -13.5, -4.5, 4.5, 13.5, 22.5, 31.5\} \text{ mm}$$

Toa do tam vach:
$$Pos = \frac{\sum_{i=0}^{7} W[i] \cdot V_{norm}[i]}{\sum_{i=0}^{7} V_{norm}[i]} \quad (\text{mm})$$

Khi $Pos = 0$, vach den nam chinh xac o giua tam robot.

#### 4.4.2. Dieu khien bam vach va bam quy dao (Stanley Tracker & Speed-Scaled PID)

* **Luot 1 (Tham do & Do vach):** Su dung bo dieu khien PID vi tri tam Centroid ket hop phanh quang hoc dong hoc co dien (Ancient Optical Dynamic Braking theo tieu chuan All-Japan RoboTrace):
  $$e = Pos \quad (\text{mm})$$
  $$\Delta v = K_p(v) \cdot e + K_i \int e \, dt + K_d \frac{de}{dt}$$
  Van toc muc tieu $V_{\text{target}}$ khong co dinh ma tu dong thich nghi lien tuc theo do lech tam quang hoc:
  $$v_{\text{target}}(e) = \frac{V_{\text{map}}}{1.0 + \left(\frac{|e|}{20.0\text{ mm}}\right)^2}$$
  Khi chay doan thang ($e \approx 0\text{ mm}$), robot chay toi da toc do $V_{\text{map}}$ (chinh qua HUD); khi tiep can goc vuong $90^\circ$ ($|e| \to 25 - 31.5\text{ mm}$), van toc tu dong giam sau xuong $\le 0.38 V_{\text{map}}$ de bo cua an toan ma khong can bat ky lenh if-else roi rac nao.

* **Luot 2 (Dua toc do cao theo quy dao toi uu):** Su dung bo dieu khien phi tuyen **Stanley Tracker** ket hop bu dong hoc Kinematic Feedforward va co che quay dao chieu banh xe (Differential Counter-Rotation):
  $$\delta(t) = \theta_e(t) + \arctan\left(\frac{k_{ct} \cdot e_{fa}(t)}{v(t) + k_{soft}}\right) + \text{steer}_{ff}$$
  Trong do:
  * $\theta_e$: Sai so goc huong giua robot va tiep tuyen quy dao tai waypoint gan nhat.
  * $e_{fa}$: Sai so lech ngang (Cross-track error) tinh tai diem nhin truoc (lookahead distance $L = 55\text{ mm}$).
  * $k_{ct} = 6.5$, $k_{soft} = 0.25$, $k_{heading} = 1.3$: Cac he so bam phi tuyen triet tieu dao dong.
  * $\text{steer}_{ff} = \frac{1}{2} v \cdot \kappa \cdot W_{\text{base}}$: Bu goc lai tien nghiem dua tren do cong Menger $\kappa$ va chieu rong co so $W_{\text{base}} = 115\text{ mm}$.
  * **Counter-Rotation:** Van toc banh xe duoc kep bao hoa doi xung $[-v_{\max}, +v_{\max}]$, cho phep banh trong chu dong quay lui khi $\delta(t) > 1.0\text{ rad}$, thuc hien cu xoay tai tam truc (pivot turn) de vuot qua goc vuong ma khong bi truot vang vang khoi duong dua.

#### 4.4.3. Hinh hoc Vung an toan va Cung ne dong Luot 1 (TacticalBehaviorArbiter)

1. Vung an toan ngoai tiep (Circumscribed Clearance Radius):
   Vat can co kich thuoc $105 \times 60\text{ mm}$, ban kinh ngoai tiep $R_{obs} = \frac{1}{2}\sqrt{105^2 + 60^2} \approx 60.47\text{ mm}$.
   Voi nua be rong robot $W/2 = 69\text{ mm}$ va he so an toan $25\text{ mm}$:
   $$R_{clearance} = R_{obs} + \frac{W_{robot}}{2} + \text{margin} \approx 154.5\text{ mm} \approx 155\text{ mm}$$
   Dap ung tieu chi an toan tuyet doi o moi goc xoay $\theta \in [0, 360^\circ]$ cua chuong ngai vat.

2. Cung ne dong o Luot 1 (TacticalBehaviorArbiter):
   Khi ToF truoc phat hien vat can trong cu ly $d \le 0.35\text{ m}$:
   * Arbiter kich hoat che do ne dong (`is_evading`), thiet lap huong ne ve phia co khoang trong lon hon.
   * Tao cung chuyen tiep $C^2$ muot ma vong qua vat can voi ban kinh dem $R_{\text{clearance}} \ge 155\text{ mm}$.
   * Ghi nhan toa do tam vat can OBB $(x, y, w, h, \theta)$ vao danh sach `detected_obstacles_` de phuc vu toi uu hoa Luot 2.
   * Khi cam bien quang bat lai vach den, ket thuc chu trinh ne trong duoi 2.3 giay ma khong bi phat loi roi vach 5.0 giay (Dieu 5.5).

#### 4.4.4. Bo toi uu hoa Quy dao Toan cuc Loi (Global Convex QP Trajectory Optimizer)

O Luot 2, thay vi dung may trang thai phan xa gay giam toc dot ngot, he thong su dung **Global Convex QP Trajectory Optimizer** giai truoc tran dua:
1. Toi thieu hoa do cong Menger toan cuc (Minimum-Curvature Formulation):
   $$\min_{\mathbf{p}} \sum_{i=1}^{N} \kappa_i^2 + \lambda_{\text{reg}} \|\mathbf{p}_i - \mathbf{p}_i^{\text{center}}\|^2 + \lambda_{\text{jerk}} \sum_{i=1}^{N-1} (\kappa_{i+1} - \kappa_i)^2$$
2. Rang buoc bien hanh lang tuy chinh (Configurable Lateral Corridor):
   Moi waypoint $\mathbf{p}_i$ duoc phep lech ngang toi da:
   $$|n_i| \le w_{\text{corridor}} \in [0.0, 120.0]\text{ mm} \quad (\text{mac dinh } 80.0\text{ mm via HUD `Max Dev`})$$
   Cho phep bo giai QP tao ra duong dua cat goc Late-Apex ro net ma van dam bao khong vi pham ranh gioi sa ban.
3. Truong the luc day vat can (Obstacle Repulsive Potential Field):
   Doi voi moi vat can da duoc dinh vi o Luot 1, he thong them thanh phan phat phi tuyen day diem quy dao ra ngoai ban kinh an toan $R_{\text{clearance}} \ge 155\text{ mm}$.

Do cong tai moi diem duoc tinh bang cong thuc do cong Menger:
$$\kappa(P_1, P_2, P_3) = \frac{4 \cdot \text{Area}(\Delta P_1 P_2 P_3)}{a \cdot b \cdot c}$$

Hồ sơ vận tốc tối ưu được tính bằng **2-Pass Dynamic Programming** kết hợp bao hình động học vi sai liên tục:
1. **Giới hạn bão hòa động học vi sai (Continuous Kinodynamic Limit):**
   $$v_{\text{diff\_limit}} = \frac{v_{\text{top}}}{1.0 + \left(\frac{v_{\text{top}}}{v_{\text{safe}}}\right) |\kappa| W_{\text{base}}}$$
   $$v_{\text{limit}} = \min\left(v_{\text{top}},\, \sqrt{\frac{\mu \cdot g}{|\kappa| + \epsilon}},\, v_{\text{diff\_limit}}\right)$$
2. **Quét lùi (Backward pass):** Tính điểm phanh tới hạn từ đích về xuất phát theo gia tốc phanh $a_{brake} = 4.5\text{ m/s}^2$.
3. **Quét xuôi (Forward pass):** Tối đa hóa gia tốc động cơ theo $a_{accel} = 3.0\text{ m/s}^2$.
4. **Vận tốc xuất phát:** Ép ràng buộc $v_{entry}(0) \ge 0.80\text{ m/s}$ triệt tiêu hiện tượng stall tại vạch xuất phát.

## 5. Kiem thu va Xac minh Chat luong (Verification & QA)

1. **Kiem thu don vi toan dien (Headless Native Unit Tests - `just test`):**
   * Bo 41 bai kiem thu tu dong xac thuc toan ven toan bo cac seam kien truc: Centroid, Odometry, SAT OBB Collision, Global Convex QP Optimizer, 2-Pass Velocity Profiler, Continuous Kinodynamics, Stanley Counter-Rotation, va Ancient Optical Dynamic Braking trong duoi 1.5 giay.
2. **Tuan thu chat luong ma nguon (0 Warning Policy - `just format-check` & `just lint`):**
   * 100% ma nguon C++ tuan thu chuan Google C++ Style 2-space qua Clang-Format (`just format-check`).
   * 0 loi, 0 canh bao tren toan bo 22 translation units qua Clang-Tidy (`just lint`).
3. **Kiem thu tich hop phan cung thuc (ESP32-S3 Hardware Verification):**
   * Kiem tra do sut ap nguon khi dong co keo dong 3A, kiem tra lay mau DMA ADC1 khong rot mau, kiem tra dem xung quadrature PCNT va giao tiep I2C ToF.
