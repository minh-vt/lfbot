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

Muc tieu la che tao robot co phan cung on dinh, tich hop thuat toan dieu khien vong kin, may trang thai vuot vat can duoi 2.5 giay, va lap ban do sa ban o Luot 1 de toi uu toc do o Luot 2.

## 2. Giai phap tong the

1. Co khi: Khung xe 145 x 138 mm, trong tam tap trung 70% vao truc banh chu dong phia sau de tang ma sat, banh ty bi cau phia truoc. Thanh cam bien cach truc banh 80 mm va co man chan sang ngoai vi.
2. Phan cung mach in: Bo mach chu dua tren ESP32-S3 WROOM-1, mach cau H kep A4950/DRV8871 chiu dong dinh 3.5A ho tro phanh chu dong, cum 3 cam bien laser ToF (truoc 0 do, trai -90 do, phai +90 do), thanh 8 mat quang analog doc qua ADC-DMA, va man hinh OLED 0.96 inch cho giao dien van hanh.
3. Nguon dien: Pin LiPo 2S 7.4V (day 8.4V). Nguon pin cap truc tiep cho dong co; mach buck ha ap xuong 5V va LDO on ap 3.3V cap rieng cho vi dieu khien va cam bien de han che nhieu sut ap.
4. Dieu khien va phan mem:
   * Cascaded PID: Vong ngoai tinh sai so goc lai tu vi tri tam Centroid; vong trong dung khoi dem phan cung PCNT giam sat van toc tung banh xe.
   * May trang thai ne vat can (FSM): Phat hien vat can o cu ly 15 cm, be goc 90 do, bam suon vat can o khoang cach 5 cm va bat lai line trong duoi 2.5 giay.
   * Lap ho so van toc (Velocity Profiler): Luot 1 ghi nhan chieu dai va do cong tung phan doan vao Flash NVS; Luot 2 tra cuu ho so toc do hinh thang va bom goc lai tien nghiem truoc cua.

## 3. Quy trinh nguoi dung va yeu cau he thong

1. Check-in ky thuat: Robot co kich thuoc 145 x 138 mm va pin LiPo 2S 8.4V de vuot qua khau do kich thuoc (< 16 cm) va do dien ap (< 12V).
2. Thao tac tai vach xuat phat: Man hinh OLED va hai nut bam (SELECT, START) cho phep chon che do Calibrate, Run 1 hoac Run 2 duoi 5 giay ma khong can ket noi may tinh.
3. Giam sat dien ap: Man hinh OLED hien thi dien ap pin voi sai so duoi 0.05V de phat hien kip thoi khi pin yeu.
4. Khang nhiễu quang hoc: Man chan quang hoc cach mat san 2 mm bao quanh 8 mat cam bien giup ngan anh sang den chieu xiên lam lech gia tri do.
5. Thu thap du lieu mat line: Khoi GDMA lay mau 8 kenh analog o tan so 10 kHz tong the ma khong lam tieu ton thoi gian xu ly cua CPU.
6. Noi suy tam Centroid: Thuat toan tinh trong tam noi suy vi tri vach den voi do phan giai duoi 0.5 mm, tranh gay giat lai.
7. Dem xung encoder: Khoi PCNT dem 4x xung quadrature cua 2 dong co truc tiep bang phan cung de khong bi sot xung khi van toc tren 2.0 m/s.
8. Vong lap van toc: Chay o chu ky 2 ms (500 Hz) tren Core 1 cua ESP32-S3 giup duy tri van toc banh xe on dinh khi dien ap pin thay doi tu 8.4V xuong 7.4V.
9. Quet vat can phia truoc: ToF phia truoc quet voi tan so 50 Hz de nhan dien vat can o cu ly 15 cm va chu dong phanh truoc.
10. Gioi han thoi gian ne: FSM chuyen tu do line sang ne va bat lai line trong duoi 2.5 giay de khong vi pham gioi han 5 giay cua Dieu 5.5.
11. Giu khoang cach suon: ToF suon phai duy tri khoang cach 5 cm voi mep vat can trong suot qua trinh chay song song.
12. Ghi nhan ban do: Luot 1 luu cac phan doan gom chieu dai, ban kinh cong va vi tri vat can vao NVS Flash.
13. Nhan dien vach dich: Nhan dien vach dich khi ca 8 mat cam bien dong thoi phat hien vach tren quang duong toi thieu 3 cm de phan biet voi nga tu cat vuong goc.
14. Phanh dung trong dich: Kich hoat che do phanh chu dong cua cau H de robot dung gon ben trong o dich.
15. Chay Luot 2: Robot dung ho so van toc da tinh toan de tang toc toi da 2.5 m/s o doan thang va giam ve 0.8 m/s truoc cua 15 cm.

## 4. Quyet dinh thiet ke chi tiet

### 4.1. Co khi

* Kich thuoc: Dai 145 mm, rong 138 mm, cao 55 mm.
* Khung tam day: In 3D bang PETG, day 3.0 mm, do dien 50% dang Gyroid. Cac lo oc dung insert dong ren nhiet M2 va M3.
* Vi tri trong tam: Dong co va pin dat lui ve sau, cach truc banh khoang 15 mm de 70% trong luong de len hai banh chu dong.
* Banh xe: Banh silicon duc hoac cao su bam duong kinh 34 mm, be rong 10 mm. Banh ty truoc la bi mat trau mini phi 12 mm.
* Cum cam bien: Tam thanh 8 mat cach truc banh chu dong 80 mm. Khe truot cho phep tinh chinh do cao tu 3.5 mm den 6.0 mm.
* Bo tri ToF: ToF truoc dat cao 25 mm so voi san; ToF trai va phai dat o hai mep suon goc 90 do.

### 4.2. Mach dien

* Vi dieu khien: Module ESP32-S3-WROOM-1, cong USB Type-C noi voi GPIO 19 (D-) va GPIO 20 (D+) de nap va debug truc tiep.
* Mach cau H: 2 IC A4950 hoac DRV8871 kem pad tich hop via tan nhiet xuong mat duoi. Moi kenh dieu khien bang 2 chan PWM ho tro che do chay va phanh chu dong.
* Thanh cam bien line: 8 cap LED phat 940 nm va phototransistor (pitch 9.0 mm, tong be rong 63 mm). Tin hieu analog noi vao ADC1 (GPIO 1 den GPIO 8).
* ToF: 3 module VL53L0X dung chung bus I2C (SDA GPIO 11, SCL GPIO 12). Cac chan XSHUT noi rieng biet de boot tuan tu va gan lai dia chi I2C.
* Phan phoi nguon: Pin LiPo 2S qua cau chi 5A cap truc tiep cho motor. Mach buck MP1584 ha xuong 5.0V / 2A, LDO AP2112K ha xuong 3.3V / 600mA cho MCU va cam bien.
* Giao dien: OLED 0.96 inch I2C, 2 nut bam chong rung phan cung kem tro keo len, va 1 buzzer mini.

### 4.3. Phan mem va phan bo tac vu

1. PCNT: Cau hinh 2 kenh dem phan cung cho banh trai (GPIO 15, 16) va banh phai (GPIO 17, 18), giai ma quadrature 4x.
2. MCPWM: Tan so phat xung 25 kHz tranh tieng rit am hoc, do phan giai 10-bit.
3. ADC-DMA: Quet lien tuc tren 8 kenh ADC1 luu vao bo dem ring thong qua GDMA.
4. Phan bo FreeRTOS:
   * Task dieu khien chuyen dong (Core 1, uu tien 5, chu ky 2 ms): Doc ADC, tinh Centroid, doc PCNT, chay Cascaded PID, xuat MCPWM.
   * Task may trang thai va ban do (Core 0, uu tien 3, chu ky 20 ms): Doc ToF, chay FSM ne vat can, cap nhat odometry va doan duong.
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

#### 4.4.2. Vong lap Cascaded PID

* Vong ngoai: Sai so vi tri $Pos$ qua bo dieu khien PID tao ra gia tri do lech van toc goc $\Delta v$.
* Toc do dat cho hai banh: $v_L^* = V_{target} - \Delta v$, $v_R^* = V_{target} + \Delta v$.
* Vong trong: Tinh van toc thuc $v_L, v_R$ tu PCNT moi 2 ms, chay PID van toc cho tung banh va xuat gia tri PWM tuong ung.

#### 4.4.3. May trang thai FSM ne vat can va Hinh hoc Vung an toan

1. Vung an toan ngoai tiep (Circumscribed Clearance Radius):
   Vat can co kich thuoc $105 \times 60\text{ mm}$, ban kinh ngoai tiep $R_{obs} = \frac{1}{2}\sqrt{105^2 + 60^2} \approx 60.47\text{ mm}$.
   Voi nua be rong robot $W/2 = 69\text{ mm}$ va he so an toan $25\text{ mm}$:
   $$R_{clearance} = R_{obs} + \frac{W_{robot}}{2} + \text{margin} \approx 154.5\text{ mm}$$
   Dap ung tieu chi an toan tuyet doi o moi goc xoay $\theta \in [0, 360^\circ]$ cua chuong ngai vat.

2. Quy trinh may trang thai phan xa:
   * `TRACKING`: Do vach binh thuong.
   * `BRAKE_APPROACH`: Khi ToF truoc $< 0.15$ m, giam toc chu dong theo profile an toan ve $0.80$ m/s.
   * `TURN_OUT`: Be cung lai tron ne sang phai dua tren vi sai banh xe thay vi goc be dot ngot.
   * `BYPASS_EDGE`: Duy tri khoang cach $0.05$ m voi mep vat can bang ToF suon phai.
   * `TURN_IN`: Be lai tiep tuyen quay ve huong tim duong.
   * `REACQUIRE`: Chay cat vach voi co che bat tuc thi (`any_ir_on_line`) de tro ve `TRACKING` ma khong bi truot qua line.
   * `EMERGENCY_SEARCH`: Neu thoi gian ne vuot qua 4.2 giay ma chua bat duoc vach, xoay tai cho tim line truoc moc 5 giay.

#### 4.4.4. Bo lap quy dao Lien tuc Do cong (C^2 Virtual Chicane Planner)

O Luot 2, thay vi dung FSM phan xa gay giam toc dot ngot, he thong su dung duong cong da thuc bac 5 (Quintic Polynomial) de tao duong tranh Virtual Chicane lien tuc do cong $C^2$:
$$u = \frac{s - s_{start}}{L_{chicane}}, \quad u \in [0, 1]$$
$$y_{lat}(u) = Y_{\max} \cdot 64 u^3 (1-u)^3$$

Dac tinh hinh hoc:
* Tai $u=0$ va $u=1$: Do lech $y=0$, tiep tuyen $y'=0$, do cong $y''=0$, triet tieu hoan toan luc giat ngan (lateral jerk).
* Tai $u=0.5$: Do lech dat dinh $y = Y_{\max} = R_{clearance} \approx 155\text{ mm}$.

Do cong Menger tai moi diem duong cong duoc tinh toan de xac dinh van toc toi da khong bi mat ma sat:
$$\kappa(P_1, P_2, P_3) = \frac{4 \cdot \text{Area}(\Delta P_1 P_2 P_3)}{a \cdot b \cdot c}, \quad R = \frac{1}{\kappa}$$
$$v_{\max} = \min\left(V_{top}, \sqrt{\mu \cdot g \cdot R}\right)$$

Ket hop cung 2-Pass Velocity Profiling:
1. Quet lui (Backward pass): Tinh diem phanh toi han tu dich ve diem xuat phat theo gia toc phanh $a_{brake} = 4.5\text{ m/s}^2$.
2. Quet xuoi (Forward pass): Toi da hoa van toc tang toc theo $a_{accel} = 3.0\text{ m/s}^2$.
3. Triet tieu dao dong xuat phat: Ep rang buoc van toc vach xuat phat $v_{entry}(0) \ge 0.80\text{ m/s}$ de loai bo hien tuong stall/rung lac tai cho.

## 5. Kiem thu

1. Kiem thu phan cung: Kiem tra nguon on dinh duoi dong tai 3A, kiem tra 8 mat analog doc qua DMA khong roi mau, kiem tra dem xung quadrature khong bi bo sot xung, va kiem tra giao tiep I2C cua 3 ToF.
2. Tinh chinh PID do vach: Chay thu o cac dai toc do tu 0.6 m/s den 2.0 m/s tren sa ban, kiem tra do on dinh tren doan thang va goc vuong.
3. Kiem thu chu trinh ne: Ghi hinh de xac nhan toan bo chu trinh ne dien ra trong 1.8 den 2.3 giay, khong cham mep vat can va khong vuot qua gioi han 5 giay.
4. Tich hop 2 luot: Kiem tra ghi nhan ban do o Luot 1 va phat huy toc do o Luot 2.
