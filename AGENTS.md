# Lam viec voi LF-Bot

LF-Bot la robot bam line va vuot chuong ngai vat cho cuoc thi F-Maze-Solving (chung ket 2026-12-02). Tai lieu nay quy dinh cach lam viec, khong lap lai thiet ke.

## Doc dung nguon

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): thiet ke hien hanh. Doc truoc khi sua core.
- [docs/IMPLEMENTATION.md](docs/IMPLEMENTATION.md): code dang co, da kiem gi, so con phai do.
- [docs/PLAN.md](docs/PLAN.md): viec con lai va nghiem thu tren xe.
- [docs/decisions/README.md](docs/decisions/README.md): quyet dinh con hieu luc va ly do.
- [docs/NAMING.md](docs/NAMING.md): nguon quy uoc dat ten duy nhat, bat buoc ap dung.
- [docs/manual.md](docs/manual.md), [docs/fqa.csv](docs/fqa.csv): nguon luat thi dau. Neu mau thuan, neu ro thay vi tu dien giai.
- [hardware/wiring.md](hardware/wiring.md): gan chan va cach dau; [docs/HARDWARE_BOM.csv](docs/HARDWARE_BOM.csv): danh muc, khong phai ket qua hieu chuan.

`.scratch/` va cac thu muc agent (`.agents/`, `.pi/`) la ghi chu cuc bo, khong phai nguon.

## Bo cuc repo

| Thu muc | Noi dung |
|---|---|
| `firmware/core/` | Logic robot, C++17, phang, test duoc tren host |
| `firmware/esp32/` | Chuong trinh ESP32: `main.cpp`, `console.*`, `pins.hpp`, `drivers/` (moi thiet bi mot lop), `storage/` (vung flash) |
| `firmware/test/` | Test hop dong tren host; `plant.hpp` la xe gia |
| `hardware/base/single/` | Board xe: PCB 1 mat lam tai nha |
| `hardware/base/fab/` | Ban 2 lop: nguon vi tri linh kien cho ban 1 mat, khong lam |
| `hardware/base/gen/`, `Base.pretty/` | Script sinh schematic/footprint/route dung chung, footprint rieng |
| `docs/` | Thiet ke, trang thai, ke hoach, ADR, luat; `research/` (nghien cuu), `track/` (anh sa ban BTC) |

## Pham vi va ky luat thay doi

1. Xac dinh yeu cau, doc cac seam lien quan, chot tieu chi kiem chung truoc khi sua.
2. Kiem tra working tree; giu nguyen thay doi co san cua nguoi dung. Khong reset, stage hay commit neu chua duoc yeu cau.
3. Sua dung pham vi. Yeu cau tai lieu khong phai uy quyen viet lai code hay sua build.
4. Dung skills phu hop; ap dung nguyen tac, khong sao chep framework/abstraction trong vi du cua skill.
5. Bao cao lenh da chay, ket qua va gioi han xac minh. Khong dung ket qua cu de tuyen bo code hien tai pass.
6. Sau moi ticket hoac khi dung mot slice, xoa file/code/tai lieu da chet trong pham vi vua lam; cap nhat tham chieu va kiem tra lai. Khong xoa du lieu nguon (anh sa ban BTC, CAD, datasheet con tham chieu), lock dang hoat dong hay thay doi ngoai pham vi. File chua tracked phai co ban sao truoc khi xoa.

## Ranh gioi bat buoc

- `firmware/core/` phang va doc lap phan cung/OS: khong ESP-IDF, FreeRTOS, file I/O, `printf` hay bien toan cuc. Trang thai co chu so huu ro rang. Module va trach nhiem theo [ARCHITECTURE](docs/ARCHITECTURE.md) ([ADR-020](docs/decisions/ADR-020-core-and-esp32.md)).
- `firmware/esp32/` giu driver va luu flash; core khong goi driver, platform khong quyet dinh chuyen dong. Khong co simulator.
- Board xe la ban 1 mat; `pins.hpp` theo board do. Khong them bang chan cho ban 2 lop.
- Chi hai so vat ly do chu du an la gia tri khoi dau: banh 43 mm, be ngang mep ngoai 110 mm (khong phai khoang cach hai banh). Su that lap rap khac do tren xe, ghi vao `config.hpp` kem nguon; robot tu do nhung gi doi theo san (muc trang/den, cuc tinh IR, bias gyro) moi lan Start.
- Mau do giu nguyen tung kenh, co thoi gian/sequence/trang thai. Hinh dang line (`None` / `Line` / `Wide` / `Split`) la suy ra o `line`, khong thay the so do goc. Khong sua so do de ne vat can.
- Khong dung nguong trang/den, gain hay toc do do o san nay lam hang so cho san khac.
- Giu hai luot thi dau: luot 1 ghi, luot 2 doc truoc ban ghi. Khong them pass-manager framework.
- Khong doan kich thuoc phan cung, he so bam, gain hay tinh hop le cua duong dua. Track mau la fixture, khong phai dai dien sa ban bi mat.
- Khong block hay cap phat dong trong vong dieu khien; khong cat timing, hieu chuan, tinh hop le cua so do va an toan de lay it dong code.

## Clean code

- C++17, Google C++ Style, 2-space theo `.clang-format`; `-Wall -Wextra -Wpedantic`, 0 errors, 0 warnings.
- Ten ham la viec robot lam, struct la danh tu mien; chi tiet xem `docs/NAMING.md`.
- Interface nho, implementation sau. Khong moi ham mot module, khong class/factory/plugin chi de boc mot implementation.
- DRY theo kien thuc va contract: hieu chuan/hinh hoc mot nguon; sensing, uoc luong, luat lai va safety dung chung cho hai luot.
- Comment chi giu ly do, don vi, rang buoc kho thay. Khong restate code, banner trang tri hay code da comment-out.

## Cong cu va lenh

Dung `just` cho build, test, lint va format. Xem `just --list` va [justfile](justfile) de biet recipe thuc te; khong sao chep lenh build thu cong vao tai lieu.

PlatformIO CLI la frontend build duy nhat ([ADR-013](docs/decisions/ADR-013-platformio-only.md)); xem [platformio.ini](firmware/platformio.ini). Env `esp32s3` dich firmware (ESP-IDF, `src_dir = esp32`, CMake noi bo do PIO quan ly); env `test` dich `core/` va `test/` tren host.

| Lenh | Muc dich |
|---|---|
| `just test` | Test hop dong core tren host |
| `just format-check` | Kiem tra clang-format |
| `just lint` | clang-tidy cho core va test host |
| `just build` | Dich firmware ESP32-S3 |
| `just check` | Format, lint, test host va build ESP32 |
| `just sanitize` | PIO build va chay tests voi ASan/UBSan |
| `just compiledb` | PIO xuat `firmware/compile_commands.json` |

Cai `clang-format`, `clang-tidy` cho cac quality gate.

## Hop dong xac minh

- Thay doi code: chay `just test` va `just format-check`; lint/build khi phu hop.
- Thay doi dieu khien/bam line/ne vat can: them regression nho, chay `just test`; test host khong chung minh robot chay. Chi board/san that moi la bang chung hanh vi.
- Thay doi chi tai lieu: kiem tra diff, link noi bo, recipe duoc nhac toi va tinh nhat quan giua cac nguon.
- Neu bi chan boi dependency, file thieu hay loi co san, ghi dung nguyen nhan va pham vi chua kiem chung; khong sua ngoai pham vi chi de lay ket qua xanh.

## Duy tri tai lieu

- Tai lieu chi mo ta hien tai; git giu lich su. Khong viet nhat ky "da lam gi, ngay nao" vao tai lieu.
- `README.md` chi giu muc dich, diem bat dau va link. `ARCHITECTURE.md` mo ta thiet ke hien hanh; `IMPLEMENTATION.md` ghi code dang co, da kiem gi, so con phai do; `PLAN.md` chi ghi viec con lai.
- ADR chi giu quyet dinh con hieu luc. Quyet dinh bi thay hoan toan thi xoa ADR cu va sua tham chieu; bi thay mot phan thi sua ADR do cho dung hien tai. Khong danh so lai.
- Khong bien gia thuyet thanh ket luan da do.
