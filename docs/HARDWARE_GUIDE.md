# HUONG DAN THIET KE PHAN CUNG VA CO KHI (KICAD & 3D CAD)

Du an: LF-BOT-2026 (F-Maze-Solving)

## 1. Phan bo chan ESP32-S3

Luu y khi ve so do nguyen ly tren KiCAD:

* Tranh dung cac chan GPIO 26 den GPIO 38. Day la cum chan noi voi Flash SPI va PSRAM noi bo cua ESP32-S3.
* Cac chan strapping gom GPIO 0 (boot mode), GPIO 45 (dien ap VDD_SPI) va GPIO 46 (ROM log) can de tro keo tu nhien, khong keo cuong buc len nguon hoac xuong dat khi khoi dong.
* Cac cam bien quang analog phai noi vao cum ADC1 (GPIO 1 den GPIO 8) vi ADC2 bi xung dot khi su dung driver khong day.

### Bang phan bo chan

| Nhom ngoai vi | Tin hieu | Chan ESP32-S3 | Chuc nang va ghi chu |
| :--- | :--- | :--- | :--- |
| Cam bien line (ADC1-DMA) | IR_0 (Trai ngoai) | GPIO 1 | ADC1_CH0 (Quet qua GDMA) |
| | IR_1 | GPIO 2 | ADC1_CH1 |
| | IR_2 | GPIO 3 | ADC1_CH2 |
| | IR_3 (Tam trai) | GPIO 4 | ADC1_CH3 |
| | IR_4 (Tam phai) | GPIO 5 | ADC1_CH4 |
| | IR_5 | GPIO 6 | ADC1_CH5 |
| | IR_6 | GPIO 7 | ADC1_CH6 |
| | IR_7 (Phai ngoai) | GPIO 8 | ADC1_CH7 |
| Do dien ap pin | VBAT_SENSE | GPIO 9 | ADC1_CH8 (Cau phan ap 100k / 33k) |
| ToF va OLED | I2C_SDA | GPIO 11 | Bus I2C chung (Tro keo 4.7k len 3.3V) |
| | I2C_SCL | GPIO 12 | Bus I2C chung (Tro keo 4.7k len 3.3V) |
| | TOF_XSHUT_FRONT | GPIO 13 | Dieu khien khoi dong ToF truoc |
| | TOF_XSHUT_RIGHT | GPIO 14 | Dieu khien khoi dong ToF phai |
| | TOF_XSHUT_LEFT | GPIO 21 | Dieu khien khoi dong ToF trai |
| Encoder trai (PCNT 0) | ENC_L_A | GPIO 15 | Kenh A dem xung banh trai |
| | ENC_L_B | GPIO 16 | Kenh B dem xung banh trai |
| Encoder phai (PCNT 1) | ENC_R_A | GPIO 17 | Kenh A dem xung banh phai |
| | ENC_R_B | GPIO 18 | Kenh B dem xung banh phai |
| USB Native Debug | USB_D- | GPIO 19 | USB CDC / JTAG |
| | USB_D+ | GPIO 20 | USB CDC / JTAG |
| Dong co trai (MCPWM) | M_L_IN1 | GPIO 39 | PWM kenh 1 cau H trai (A4950) |
| | M_L_IN2 | GPIO 40 | PWM kenh 2 cau H trai (A4950) |
| Dong co phai (MCPWM) | M_R_IN1 | GPIO 41 | PWM kenh 1 cau H phai (A4950) |
| | M_R_IN2 | GPIO 42 | PWM kenh 2 cau H phai (A4950) |
| Nut nhan va chuong | BTN_MODE | GPIO 47 | Nut chon che do (Tro keo noi + tu 100nF) |
| | BTN_START | GPIO 48 | Nut chay / dung (Tro keo noi + tu 100nF) |
| | BUZZER | GPIO 10 | Dieu khien coi bao qua transistor NPN |

## 2. Thiet ke mach in (KiCAD layout)

### 2.1. Phan vung mat dat va duong nguon

1. Chia tach mat dat (Ground plane):
   * Power GND (PGND): Noi chan mass cua 2 IC cau H, tu hoa 220 uF va cuc am jack pin XT30.
   * Signal GND (DGND/AGND): Noi chan mass cua ESP32-S3, thanh cam bien quang, cum ToF va OLED.
   * PGND va DGND duoc noi tai duy nhat mot diem (Star ground) ngay tai chan am cua jack XT30 de dong dien cua dong co khong gay nhieu len duong do analog.
2. Do rong duong mach:
   * Duong nguon dong co (VBAT den IC A4950): Rong toi thieu 2.5 mm hoac do polygon rieng. Dong 1oz chiu duoc dong xung 5A.
   * Duong nguon 5V va 3.3V: Rong tu 0.6 mm den 0.8 mm.
   * Duong tin hieu logic: Rong 0.25 mm.
3. Giai nhiet cau H:
   * IC A4950 va DRV8871 co pad tan nhiet duoi bung.
   * Bo tri tu 6 den 9 lo via tan nhiet (Thermal Vias) phi 0.3 mm noi pad bung xuong mat dong phia duoi de giai phong nhiet khi robot phanh chu dong.

## 3. Thiet ke co khi (3D CAD)

### 3.1. Kich thuoc khung xe

* Kich thuoc bao hinh: 145 mm (dai) x 138 mm (rong), giu khoang an toan 15 mm so voi gioi han 160 x 160 mm cua cuoc thi.
* Tam day (Baseplate): In 3D bang PETG hoac ABS, do day 3.0 mm, do dien 50% dang Gyroid. Cac lo bat vit dung insert dong ren nhiet M2 va M3 thay vi bat vit truc tiep vao nhua.

### 3.2. Cum dong co va banh xe

* Ga dong co N20: Om sat than hop so kim loai (12 x 10 mm), co go chan sau de motor khong bi xo lech khi va cham.
* Chieu rong co so: Khoang cach giua hai banh chu dong tu 110 mm den 115 mm giup xe xoay goc 90 do gon gang ma khong bi vang duoi.
* Banh xe: Duong kinh 34 mm, be rong lop 10 mm. Truc motor D phi 3 mm can co vit tri vao mat vat de chong truot khi dao chieu phanh gap.

### 3.3. Cum cam bien truoc

* Khoang cach don bay: Tam cum 8 mat quang cach truc banh sau 80 mm.
* Do cao gam: Mat cam bien cach san 4.0 mm, co khe truot dung tren ga in 3D de tinh chinh do cao tu 3.5 mm den 6.0 mm.
* Man chan sang: In bang nhua den mo (Matte Black) bao quanh 4 phia cua thanh 8 mat, mep duoi cach san 2.0 mm de hap thu anh sang ngoai vi.

### 3.4. Phan bo trong tam

* Dong co va pin LiPo dat lui ve sau, cach truc banh khoang 15 mm.
* Trong luong toan bo robot (kem pin) muc tieu trong khoang 180g den 240g.
* Khoang 70% tong khoi luong de len truc hai banh chu dong de toi da hoa luc ma sat bam duong khi tang toc 3.0 m/s2.
