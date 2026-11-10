# Lam viec voi LF-Bot

LF-Bot la robot bam line va vuot chuong ngai vat cho cuoc thi F-Maze-Solving. Tai lieu nay quy dinh cach lam viec, khong lap lai thiet ke hay lich su trien khai.

## Doc dung nguon

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): kien truc dich, ranh gioi du lieu va cac diem chua chot. Doc truoc khi sua core.
- [docs/decisions/README.md](docs/decisions/README.md): quyet dinh va ly do; kiem tra status truoc khi ap dung ADR cu.
- [docs/NAMING.md](docs/NAMING.md): nguon quy uoc dat ten duy nhat, bat buoc ap dung.
- [docs/manual.md](docs/manual.md), [docs/fqa.csv](docs/fqa.csv): nguon luat thi dau. Neu mau thuan, neu ro thay vi tu dien giai.
- [docs/HARDWARE_BOM.csv](docs/HARDWARE_BOM.csv): danh muc de xuat, khong phai ket qua hieu chuan.

Code hien tai va ghi chu `.scratch/` co the thuoc thiet ke cu. Khong lay chung lam bang chung rang kien truc moi da duoc trien khai. Khong sua code chi de lam no khop tai lieu khi chua duoc giao viec do.

## Pham vi va ky luat thay doi

1. Xac dinh yeu cau, doc cac seam lien quan, chot tieu chi kiem chung truoc khi sua.
2. Kiem tra working tree; giu nguyen thay doi co san cua nguoi dung. Khong reset, stage hay commit neu chua duoc yeu cau.
3. Sua dung pham vi. Yeu cau tai lieu khong phai uy quyen viet lai code hay sua build.
4. Dung skills phu hop; ap dung nguyen tac, khong sao chep framework/abstraction trong vi du cua skill.
5. Bao cao lenh da chay, ket qua va gioi han xac minh. Khong dung ket qua cu de tuyen bo code hien tai pass.
6. Sau moi ticket hoac khi dung mot slice, cleanup file/code/report da chet trong pham vi vua lam; cap nhat tham chieu va kiem tra lai. Chi xoa khi co bang chung khong con dung, giu recovery cho file chua tracked; khong xoa du lieu nguon, CAD/thu vien con tham chieu, lock dang hoat dong hay thay doi ngoai pham vi.

## Ranh gioi bat buoc

- Core `firmware/lib/` doc lap phan cung/OS: khong Raylib, ESP-IDF, FreeRTOS, file I/O, `printf` hay bien toan cuc. Trang thai co chu so huu ro rang.
- Core `firmware/lib/robot/` phang: `robot_data.hpp` chi du lieu (khong include Robot/Race); `Drive` so huu conversion/freshness/deadline va duoc Robot cung environment training dung chung; Robot la scoped typed owner; Race quyet dinh chuyen dong. Platform chi borrow Robot qua RobotSession/run_robot, khong own/cap storage core. Simulation chi nhan `MotorOutput`, khong include/call/link Robot, Race hay policy; composition nam o `RaceTrial`. ESP32/HIL deferred, khong restore core cu de cuu build.
- `SensorReadings` la so do da chuan hoa, khong phai tui trang thai hay duong muc tieu. Khong sua so do de ne vat can; khong them `LineView`/nhan ahead-side lam lop trung gian mac dinh.
- Khong dua ground truth cua sim vao quyet dinh robot. Vat ly va cham luat phai chay headless, GUI chi quan sat/dieu khien cong cu mo phong.
- Giu hai luot thi dau va phan biet voi multi-pass xu ly. Khong tu bo yeu cau multi-pass, cung khong them pass-manager framework khi chua can.
- Khong doan kich thuoc phan cung, he so bam, gain hay tinh hop le cua duong dua. Track mau la fixture, khong phai dai dien sa ban bi mat.
- Khong block hay cap phat dong trong vong dieu khien; khong cat timing, hieu chuan, tinh hop le cua so do va an toan de lay it dong code.

## Clean code

- C++17, Google C++ Style, 2-space theo `.clang-format`; `-Wall -Wextra -Wpedantic`, 0 errors, 0 warnings.
- Ten ham la viec robot lam, struct la danh tu mien; chi tiet xem `docs/NAMING.md`.
- Interface nho, implementation sau. Khong moi ham mot module, khong class/factory/plugin chi de boc mot implementation.
- DRY theo kien thuc va contract: hieu chuan/hinh hoc dung chung mot nguon; khong sao chep controller giua sim/that hoac hai luot.
- Comment chi giu ly do, don vi, rang buoc kho thay. Khong restate code, banner trang tri hay code da comment-out.

## Cong cu va lenh

Dung `just` cho build, test, lint va format. Xem `just --list` va [justfile](justfile) de biet recipe thuc te; khong sao chep lenh build thu cong vao tai lieu.

PlatformIO CLI la frontend build duy nhat cho native va ESP32; xem [platformio.ini](firmware/platformio.ini) va recipe trong [justfile](justfile). Native can compiler C++17, pkg-config, libpng, nlohmann-json va Raylib cho GUI; clang-format/clang-tidy cho quality gates. ESP32/ESP-IDF/HIL van deferred; CMake noi bo ESP-IDF do PIO quan ly, khong la frontend thu hai. Xem [ADR-013](docs/decisions/ADR-013-platformio-only.md) va [ADR-004](docs/decisions/ADR-004-esp32-toolchain.md).

| Lenh | Muc dich |
|---|---|
| `just test` | Kiem thu core va lifecycle/pose GUI composition headless |
| `just test-sim-runtime` | Kiem thu runtime headless: loader/world/referee/tick, khong phai hai luot |
| `just format-check` | Kiem tra clang-format |
| `just lint` | clang-tidy native; ESP32 lint/HIL deferred trong milestone sim+lib |
| `just build-native` | PIO build GUI va tests native |
| `just check` | Format, lint, core/composition/runtime tests va native build |
| `just sanitize` | PIO build va chay tests voi ASan/UBSan |
| `just compiledb` | PIO xuat `firmware/compile_commands.json` |
| `just sim` | Mo GUI cho nguoi dung, khong phai smoke test tu dong |

Cai `clang-format`, `clang-tidy`, `cppcheck` cho cac quality gate tuong ung. Cac recipe tham chieu file da xoa co the chua chay duoc trong dot tai thiet; bao ro loi, khong tu khoi phuc thiet ke cu.

## Hop dong xac minh

- Thay doi code: chay `just test` va `just format-check`; lint/build nen tang bi anh huong khi phu hop. Thay doi engine/interface phai giu `test_sim_runtime` chi link engine + data, khong keo controller vao.
- Thay doi bam line, ne vat can, dieu khien hay lap duong: them regression nho cho hanh vi bi anh huong; chay `just test` va `just test-sim-runtime`. Gate full-race `sim-test` da bo theo yeu cau; tests xanh khong chung minh robot hoan thanh duong dua.
- Khong yeu cau nguoi dung mo `just sim` de tu xac nhan thay cho kiem thu tu dong.
- Thay doi chi tai lieu: kiem tra diff, link noi bo, recipe duoc nhac toi va tinh nhat quan giua cac nguon. Neu chay test cu, khong goi ket qua do la xac minh kien truc moi.
- Neu bi chan boi dependency, file thieu hay loi co san, ghi dung nguyen nhan va pham vi chua kiem chung; khong sua ngoai pham vi chi de lay ket qua xanh.

## Duy tri tai lieu

`README.md` chi giu muc dich, diem bat dau va link; khong them cay file chi tiet, cong thuc, phim tat GUI hay so lieu test. `ARCHITECTURE.md` mo ta thiet ke hien hanh, ADR giai thich lua chon. Khi thay quyet dinh, tao ADR moi va danh dau ADR cu bi thay the; khong xoa lich su hay bien gia thuyet thanh ket luan da do.
