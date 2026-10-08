# LF-BOT 2026: Robot tu hanh do line va ne vat can

He thong robot tu hanh do line toc do cao va vuot chuong ngai vat tham gia cuoc thi "F-Maze-Solving" (Line the Future) tai Dai hoc FPT.

Du an duoc thiet ke tach biet giua thu vien giai thuat (firmware/lib) khong phu thuoc phan cung va phan thuc thi tren tung nen tang (mo phong 2D Raylib tren Desktop va firmware nhung ESP32-S3).

## 1. Cau truc du an

```
lfbot/
├── .clang-format              # Dinh dang ma nguon 2-space (Google C++ Style)
├── .clang-tidy                # Cau hinh kiem tra tinh linter C++
├── .gitignore
├── README.md                  # Huong dan tong quan va van hanh
├── docs/                      # Tai lieu ky thuat va the le
│   ├── CONTEXT.md             # Thuat ngu nghiep vu he thong (SSoT)
│   ├── SPEC.md                # Ban dac ta ky thuat chi tiet
│   ├── HARDWARE_GUIDE.md      # Huong dan mach in KiCAD va thiet ke co khi 3D
│   ├── manual.md              # The le cuoc thi F-Maze-Solving
│   └── decisions/             # Ho so quyet dinh kien truc (ADR-0001 den ADR-0008)
├── hardware/                  # Ho so thiet ke phan cung
│   ├── pcb/                   # File mach in KiCAD
│   └── mech/                  # Mo hinh in 3D (.step, .stl)
└── firmware/                  # Ma nguon he thong (PlatformIO)
    ├── platformio.ini         # Cau hinh bien dich cho sim, test va esp32s3
    ├── assets/                # Du lieu sa ban
    │   ├── track.png          # Sa ban chuan 2400x1600 (1 px = 1 mm)
    │   └── generate_track.py  # Script sinh sa ban procedural generation
    ├── lib/                   # Thu vien logic loi doc lap phan cung (C++17)
    │   ├── config/            # Cau hinh tap trung va kieu du lieu
    │   │   ├── robot_config.hpp
    │   │   └── types.hpp
    │   ├── math/              # Toan hoc, odometry va tinh tam vach
    │   │   ├── centroid.hpp/.cpp
    │   │   ├── collision.hpp/.cpp
    │   │   ├── kinematics.hpp/.cpp
    │   │   └── odometry.hpp/.cpp
    │   ├── control/           # Thuat toan dieu khien chuyen dong
    │   │   ├── pid_controller.hpp/.cpp
    │   │   ├── relay_tuner.hpp/.cpp
    │   │   ├── stanley_tracker.hpp/.cpp
    │   │   └── trajectory_follower.hpp/.cpp
    │   ├── strategy/          # Chien thuat phan xa, ne vat can va an toan
    │   │   └── tactical_arbiter.hpp/.cpp
    │   ├── track/             # Quy hoach quy dao va toi uu hinh hoc sa ban
    │   │   ├── global_trajectory_optimizer.hpp/.cpp
    │   │   └── trajectory_planner.hpp/.cpp
    │   └── system/            # Bo dieu phoi trung tam
    │       ├── robot_coordinator.hpp/.cpp
    │       └── telemetry_packet.hpp
    ├── src/                   # Thuc thi tren tung nen tang
    │   ├── sim/               # Mo phong Desktop (Raylib)
    │   │   ├── hud.hpp/.cpp   # Giao dien cockpit va tham so thoi gian thuc
    │   │   ├── race_arbiter.hpp/.cpp # Trong tai thi dau so doc lap giam sat luat
    │   │   ├── raycast.hpp/.cpp # Giai toan re quat quang hoc 21 tia ToF vs OBB
    │   │   ├── sim_engine.hpp/.cpp # Dong co vat ly mo phong headless tach biet GUI
    │   │   ├── theme.hpp      # Bang mau UI Glassmorphism & Cyberpunk SSoT
    │   │   ├── types.hpp      # Dinh nghia kieu du lieu va hang so mo phong
    │   │   ├── world.hpp/.cpp # Ve sa ban 2D, khung xe va cum cam bien
    │   │   └── main.cpp       # Orchestrator dieu phoi mo phong Raylib GUI
    │   └── esp32/             # Firmware nhung (ESP32-S3 FreeRTOS)
    │       └── main.cpp       # Diem vao FreeRTOS tren phan cung thuc
    └── test/                  # Kiem thu tu dong Headless
        ├── test_core.cpp      # Kiem thu don vi toan hoc Centroid, Kinematics, Stanley
        └── test_simulation.cpp # Kiem thu mo phong headless 2 luot va adversarial tests
```

## 2. Lenh thao tac nhanh (Task Runner - Just)

Du an su dung `just` (tuong tu npm scripts trong package.json) de quan ly va thuc thi tat ca tac vu:

```bash
# Xem danh muc toan bo lenh ho tro
just

# Cac lenh thuong dung
just sim          # Chay mo phong Desktop Raylib (GUI danh cho nguoi dung quan sat)
just test         # Chay toan bo unit tests headless (< 0.3s)
just sim-test     # Chay kiem thu mo phong headless 2 luot (Map & Sprint) (< 0.5s)
just check        # Kiem tra toan dien: format + lint + test + sim-test (~ 1.1s)
just lint         # Kiem tra tinh voi clang-tidy
just lint-fix     # Tu dong sua loi clang-tidy va format lai code
just format       # Dinh dang toan bo ma nguon voi clang-format
just format-check # Kiem tra format (phuc vu CI/CD)
just compiledb    # Tao compile_commands.json cho linter va clangd
just track        # Sinh sa ban procedural (vi du: just track --complexity 5 --seed 42)
just upload       # Nap chuong trinh xuong ESP32-S3
just monitor      # Mo serial monitor ESP32-S3 (115200 baud)
```

## 3. Sinh sa ban ngau nhien (Procedural Generation)

Script `firmware/assets/generate_track.py` ho tro sinh sa ban tu dong bang thuat toan duong cong kin ngau nhien (Radial Spline / Catmull-Rom), dong thoi dam bao ban kinh goc cua du lon (> 120 mm) va goc cat du rong (> 45 do) de robot khong bi ket vach.

```bash
# Xem danh sach tham so ho tro
python3 firmware/assets/generate_track.py --help

# Sinh sa ban ngau nhien theo do phuc tap va seed
python3 firmware/assets/generate_track.py --mode procedural --complexity 5 --seed 42

# Sinh sa ban hinh so 8 (figure-8)
python3 firmware/assets/generate_track.py --mode figure8

# Sinh sa ban mau thi dau tieu chuan
python3 firmware/assets/generate_track.py --mode competition
```

## 4. Chay mo phong tren Desktop (Raylib)

Yeu cau may tinh da cai dat thu vien Raylib va PlatformIO.

```bash
cd firmware
pio run -e sim -t exec
```

Hoac bien dich truc tiep qua GCC:

```bash
g++ -std=c++17 -O3 -Wall -Wextra -Wpedantic \
  -I firmware/lib -I firmware/src/sim \
  firmware/lib/*/*.cpp firmware/src/sim/*.cpp \
  -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 \
  -o sim_app && ./sim_app
```

### Phim tat dieu khien trong mo phong

* Che do hoat dong:
  * `[1]`: Chay Luot 1 (Map Scanning & Profile Learning) o toc do an toan.
  * `[2]`: Chay Luot 2 (Sprint theo ho so toi uu, hoac Reactive Sprint neu chua co map).
  * `[SPACE]`: Tam dung hoac tiep tuc mo phong.
  * `[R]`: Dat lai robot ve vi tri xuat phat (giu nguyen ban do da hoc).
  * `[Shift + R]`: Dat lai toan bo trang thai robot va xoa sach ban do da hoc.

* Dieu khien goc nhin:
  * `[C]`: Bat hoac tat che do camera bam theo robot.
  * `[F]`: Can chinh camera vua van toan bo sa ban thi dau.
  * `[Chuot phai + Keo]`: Di chuyen goc nhin camera (Pan).
  * `[Cuon chuot]`: Phong to hoac thu nho camera (Zoom).

* Tinh chinh tham so chien thuat truc quan (Cockpit HUD Tuning Drawer):
  * Click chuot truc tiep vao cac nut `+` / `-` tren thanh Tuning Drawer duoi day man hinh:
    * `Map V`: Van toc tham do Luot 1 ($[0.20, 1.50]\text{ m/s}$, buoc $\pm 0.05\text{ m/s}$).
    * `Sprint V`: Van toc toi da Luot 2 ($[0.50, 4.00]\text{ m/s}$, buoc $\pm 0.10\text{ m/s}$).
    * `Max Dev`: Hanh lang lech ngang toi da ($[0.0, 120.0]\text{ mm}$, buoc $\pm 5.0\text{ mm}$, mac dinh $80.0\text{ mm}$).
    * `Jerk`: He so phat giat dao ham do cong ($[0.001, 0.100]$, buoc $\pm 0.005$, mac dinh $0.020$).
  * Nut `[Optimize]`: Kich hoat tai toi uu hoa quy dao QP ngay lap tuc sau khi thay doi vat can hoac tham so.
  * Nut `[Auto Tune]`: Kich hoat bo do tham so tu dong Relay Feedback Auto-Tuner de tim $K_p, K_d$ toi uu cho cam bien line.

* Tuong tac vat can tren sa ban:
  * `[Chuot trai]`: Click len vi tri trong de dat vat can moi, hoac giu chuot tren vat can de keo tha den vi tri bat ky.
  * `[Cuon chuot khi hover]` hoac `[Q] / [E]`: Xoay goc vat can (+/- 5 do moi nac) de kiem thu o moi goc do cheo/vuong bat ky.
  * `[O]`: Dat nhanh vat can tai vi tri con tro chuot hien tai.
  * `[X]`: Xoa sach toan bo vat can tren sa ban.
  * `[Delete] hoac [Backspace]`: Xoa vat can dang nam duoi con tro chuot.

* Hien thi va So sanh quy dao:
  * Vet net dut xanh ngoc (Cyan dashed trail): Quy dao di chuyen thuc te cua robot o Luot 1 (Unoptimized trail).
  * Dai ribbon chuyen mau gradient (Amber to Emerald): Quy dao dua toi uu toan cuc (Global Convex QP Trajectory) cho Luot 2. Mau sac the hien toc do tu thap (cam ho phach) den cao (xanh ngoc luc bao).
  * Vong tron an toan (Clearance Ring - $R \approx 155\text{ mm}$): Hien thi truc quan ban kinh an toan tuyet doi quanh vat can o moi goc xoay.
  * Dong ho do goc xoay (Rotation Badge): Hien thi goc xoay $\theta$ thoi gian thuc tren dau moi vat can.
  * Thong so dong hoc banh xe (Cockpit HUD): Hien thi truc quan van toc dat vs thuc te cua banh trai/phai (m/s), xung encoder va khoang cach 3 truc ToF (Front, Left, Right).

## 5. Kiem thu tu dong (Automated Headless Tests)

He thong ho tro bo kiem thu tu dong Headless khong can mo giao dien do hoa, hoan tat toan bo duoi 1.5 giay:

```bash
# 1. Kiem thu toan bo 41 unit tests giai thuat loi (< 1.5s)
just test

# 2. Kiem tra toan dien chat luong ma nguon (0 warning policy)
just format-check # 100% tuan thu Google C++ Style
just lint         # 0 loi, 0 canh bao tren 22 translation units
```

## 6. Kiem tra dinh dang ma nguon (Clang-Format)

Moi file C++ tuan thu chuan 2-space indent Google C++ Style:

```bash
# Kiem tra loi format (tra ve 0 neu dat yeu cau)
just format-check

# Tu dong dinh dang lai toan bo ma nguon
just format
```

## 7. Phan tich ma nguon tinh (Clang-Tidy)

Toan bo ma nguon tuan thu 0 loi va 0 canh bao theo bo quy tac `.clang-tidy`:

```bash
# Tao co so du lieu bien dich compile_commands.json
just compiledb

# Chay kiem tra linter tinh tren toan bo ma nguon C++
just lint

# Tu dong sua loi (cho cac check ho tro --fix)
just lint-fix
```

## 8. Nap chuong trinh xuong phan cung ESP32-S3

Ket noi board mach voi may tinh qua cong USB Type-C:

```bash
just upload
just monitor
```
