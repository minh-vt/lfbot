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
- Core `firmware/lib/robot/` phang va doc lap ESP-IDF; module va trach nhiem theo [ARCHITECTURE](docs/ARCHITECTURE.md) (ADR-020/021). `firmware/src/esp32/` giu driver, moi thiet bi mot lop nho; core khong goi driver, platform khong quyet dinh chuyen dong. Simulator da bo; khong khoi phuc core/sim cu. Chi hai so vat ly do chu du an la gia tri khoi dau: banh 43 mm, be ngang mep ngoai 110 mm (khong phai wheelbase); con lai do tai nha (profile robot), robot tu do tai san (profile san) hoac thich nghi co bien khi chay. Thieu profile bat buoc thi khong cho Start.
- Mau do giu nguyen tung kenh, co thoi gian/sequence/trang thai. Phan loai mau IR (khong line / mot cum / nhieu cum / toan den / toan trang) la bang chung suy ra kem do tin cay o `line_sensing`, khong thay the so do goc. Khong sua so do de ne vat can.
- Khong dung nguong trang/den, gain hay toc do do o san nay lam hang so cho san khac; chung thuoc profile san hoac thich nghi.
- Giu hai luot thi dau va phan biet voi multi-pass xu ly. Khong tu bo yeu cau multi-pass, cung khong them pass-manager framework khi chua can.
- Khong doan kich thuoc phan cung, he so bam, gain hay tinh hop le cua duong dua. Track mau la fixture, khong phai dai dien sa ban bi mat.
- Khong block hay cap phat dong trong vong dieu khien; khong cat timing, hieu chuan, tinh hop le cua so do va an toan de lay it dong code.

## Clean code

- C++17, Google C++ Style, 2-space theo `.clang-format`; `-Wall -Wextra -Wpedantic`, 0 errors, 0 warnings.
- Ten ham la viec robot lam, struct la danh tu mien; chi tiet xem `docs/NAMING.md`.
- Interface nho, implementation sau. Khong moi ham mot module, khong class/factory/plugin chi de boc mot implementation.
- DRY theo kien thuc va contract: hieu chuan/hinh hoc mot nguon; sensing, uoc luong, bam toc do banh va safety dung chung cho hai luot. Hai luot duoc phep co controller rieng (luot 1 phan hoi IR, luot 2 bam ke hoach map).
- Comment chi giu ly do, don vi, rang buoc kho thay. Khong restate code, banner trang tri hay code da comment-out.

## Cong cu va lenh

Dung `just` cho build, test, lint va format. Xem `just --list` va [justfile](justfile) de biet recipe thuc te; khong sao chep lenh build thu cong vao tai lieu.

PlatformIO CLI la frontend build duy nhat; xem [platformio.ini](firmware/platformio.ini) va [justfile](justfile). Env `esp32s3` dich firmware (ESP-IDF, CMake noi bo do PIO quan ly); env `test` chay test hop dong tren host. Can clang-format/clang-tidy cho quality gates. Xem [ADR-013](docs/decisions/ADR-013-platformio-only.md), [ADR-020](docs/decisions/ADR-020-esp32-firmware-restart.md).

| Lenh | Muc dich |
|---|---|
| `just test` | Test hop dong core tren host (calibration, nut bam) |
| `just format-check` | Kiem tra clang-format |
| `just lint` | clang-tidy cho core va test host |
| `just build` | Dich firmware ESP32-S3 |
| `just check` | Format, lint, test host va build ESP32 |
| `just sanitize` | PIO build va chay tests voi ASan/UBSan |
| `just compiledb` | PIO xuat `firmware/compile_commands.json` |

Cai `clang-format`, `clang-tidy`, `cppcheck` cho cac quality gate tuong ung. Cac recipe tham chieu file da xoa co the chua chay duoc trong dot tai thiet; bao ro loi, khong tu khoi phuc thiet ke cu.

## Hop dong xac minh

- Thay doi code: chay `just test` va `just format-check`; lint/build nen tang bi anh huong khi phu hop. Thay doi engine/interface phai giu `test_sim_runtime` chi link engine + data, khong keo controller vao.
- Thay doi dieu khien/bam line/ne vat can: them regression nho, chay `just test`; test host khong chung minh robot chay. Chi board/san that moi la bang chung hanh vi.
- Thay doi chi tai lieu: kiem tra diff, link noi bo, recipe duoc nhac toi va tinh nhat quan giua cac nguon. Neu chay test cu, khong goi ket qua do la xac minh kien truc moi.
- Neu bi chan boi dependency, file thieu hay loi co san, ghi dung nguyen nhan va pham vi chua kiem chung; khong sua ngoai pham vi chi de lay ket qua xanh.

## Duy tri tai lieu

`README.md` chi giu muc dich, diem bat dau va link; khong them cay file chi tiet, cong thuc, phim tat GUI hay so lieu test. `ARCHITECTURE.md` mo ta thiet ke hien hanh, ADR giai thich lua chon. Khi thay quyet dinh, tao ADR moi va danh dau ADR cu bi thay the; khong xoa lich su hay bien gia thuyet thanh ket luan da do.
