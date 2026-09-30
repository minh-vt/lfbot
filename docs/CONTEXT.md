# CONTEXT: He thong robot tu hanh do line va ne vat can (LF-BOT)

Tai lieu nay dinh nghia he thong thuat ngu thong nhat (Ubiquitous Language) va ranh gioi nghiep vu cua du an.

## 1. Domain glossary (Thuat ngu nghiep vu)

### Sa ban va moi truong

* Sa ban (Arena / Track): Mat phang thi dau nen trang, vach den. Chieu rong vach quy chuan la 2.0 cm.
* Vat can (Obstacle): Khoi hop chu nhat kich thuoc 10.5 x 6.0 cm dat tren sa ban.
* O dich (Finish Zone): Vung ket thuc sa ban duoc danh dau bang vach den toan phan.
* Ty le toa do (World Scale): Quy uoc 1 px = 1.0 mm (1000 px/m). Vach 2 cm tuong ung 20 px tren anh mo phong; vat can tuong ung 105 x 60 px.

### Thuc the robot

* Khung xe (Chassis): Kich thuoc co khi 145 x 138 mm, nam trong gioi han toi da 160 x 160 mm theo quy che thi dau.
* Thanh cam bien quang (Sensor Bar): Cum 8 mat hong ngoai analog bo tri doi xung qua truc tam, buoc mat (pitch) 9.0 mm, co man chan quang hoc giam anh sang ngoai vi.
* Trong tam vach (Centroid): Vi tri lech tam cua vach den tinh theo milimet trong khoang [-31.5, +31.5] mm dua tren trong so vi tri 8 mat. Gia tri 0 mm bieu thi tam robot trung tim vach den.
* Cum 3 cam bien laser ToF (ToF Trio):
  * Front ToF (0 do): Huong thang phia truoc de phat hien va phanh chu dong khi tiep can vat can.
  * Left ToF (-90 do): Huong sang suon trai de do cu ly mep vat can hoac vach bien.
  * Right ToF (+90 do): Huong sang suon phai de bam mep vat can trong chu trinh bypass.
* Khoang cach phat hien vat can (D_obs): Nguong cu ly phia truoc (mac dinh 0.15 m, tinh chinh tu 0.08 m den 0.60 m) kich hoat chu trinh giam toc tiep can.

### Dieu khien va chien thuat

* Bo dieu phoi trung tam (RobotCoordinator): Lop logic loi C++ dong vai tro trung tam dieu phoi trang thai, kinematics, odometry, PID va velocity profiler. Khong phu thuoc phan cung (Single Source of Truth).
* Vong lap kep (Cascaded PID):
  * Vong ngoai (Line PID): Chuyen sai so lech tam vach Centroid (mm) thanh do lech lai goc.
  * Vong trong (Speed PID): Doc xung Encoder tung banh xe de duy tri van toc thuc te bam sat van toc dat.
* May trang thai ne vat can (Obstacle FSM): Chuoi trang thai khong che thoi gian duoi 5 giay de khong vi pham Dieu 5.5: TRACKING -> BRAKE_APPROACH -> TURN_OUT -> BYPASS_EDGE -> TURN_IN -> REACQUIRE. Tich hop cung vung an toan ngoai tiep $R_{clearance} \approx 155\text{ mm}$ bao phu moi goc xoay vat the tu 0 den 360 do.
* Quy dao Virtual Chicane muot $C^2$ (Continuous Curvature): Duong cua tranh vat can tao boi da thuc bac 5 (Quintic polynomial) lien tuc do cong $C^2$, triet tieu do giat (jerk) tai hai dau bien tiep tuyen voi tim duong va tu dong dieu chinh van toc theo ban kinh cong Menger de khong bi vuot nguong ma sat.
* Ho so van toc 2 chieu (Two-Pass Velocity Profiler): Thuat toan toi uu van toc dua tren du lieu topo Luot 1:
  * Backward pass: Quet nguoc tu dich ve diem xuat phat de tim diem phanh tre nhat dua tren gia toc phanh cho phep.
  * Forward pass: Quet xuoi tu vach xuat phat den dich de toi da hoa van toc tren doan thang dua tren gia toc dong co, dong thoi ep van toc xuat phat $v_{entry}(0) \ge 0.8\text{ m/s}$ de triet tieu rung lac tai vach xuat phat.
* Che do chay nuoc rut phan xa (Reactive Sprint): Co che chay o Luot 2 khi chua nap ban do tu Luot 1. Robot tu dong dieu chinh toc do dua tren do lech tam Centroid thoi gian thuc de toi uu van toc doan thang ma van giu xe an toan tai cua.
* So sanh quy dao truc quan (Trajectory Visual Comparison): Render dong thoi vet chay thuc te Luot 1 (vet net dut xanh ngoc) va duong dua toi uu hoa $C^2$ Luot 2 (dai ribbon doi mau tu ho phach sang xanh luc bao theo dai toc do).
* Tu chinh thong so (RelayAutoTuner): Thuat toan tao dao dong toi han dua tren phuong phap relay feedback cua Astrom-Hagglund de xac dinh nhanh bo thong so {Kp, Ki, Kd} tren sa ban thuc te.
* Bo dieu khien Speed-Scaled PID: Thuat toan tu dong co gian do loi {Kp, Kd} ti le nghich voi van toc xe de triet tieu hien tuong rung lac (oscillation) o van toc cao tren doan thang trong khi van tang luc bam khi giam toc vao goc cua gat.
* Tram thu thap du lieu mat dat (Digital Twin Ground Station): Che do nhan stream du lieu UDP 50-100Hz tu robot that (hoac script gia lap), tai tao quy dao thoi gian thuc tu encoder (dead-reckoning) va tu dong can chinh ty le camera (Auto-Scale) theo vet chay thuc dia.

### Nen tang chay

* Target ESP32-S3: Firmware thuc thi tren chip ESP32-S3 voi FreeRTOS, ADC-DMA, PCNT va MCPWM.
* Target Desktop Simulation: Mo phong do hoa 2D chay tren PC bang Raylib voi bo dem Gaussian blur mo phong tinh hieu analog quang hoc, tich hop trạm thu Telemetry khong day.
