# Quyet dinh kien truc

ADRs ghi boi canh, lua chon va danh doi; [ARCHITECTURE.md](../ARCHITECTURE.md) mo ta thiet ke hien hanh. Accepted nghia la chap nhan quyet dinh, khong co nghia da trien khai hay kiem chung tren phan cung.

| ADR | Trang thai | Pham vi |
|---|---|---|
| [001](ADR-001-two-layer-architecture.md) | Superseded by 007 | Lich su core khong-thu-nguyen va adapters |
| [004](ADR-004-esp32-toolchain.md) | Accepted | Lua chon ESP-IDF; chi tiet trien khai cu khong phai bang chung timing |
| [006](ADR-006-naming-clean-code.md) | Accepted | Ten theo viec robot lam; quy uoc hien hanh o NAMING.md |
| [007](ADR-007-readings-first-core.md) | Superseded by 009 | So do, trang thai, muc tieu va ranh gioi core |
| [008](ADR-008-shared-line-following.md) | Superseded by 009 | Chung co che bam duong, hai luot va ne vat can |
| [009](ADR-009-scoped-robot-native-runtime.md) | Accepted; partly superseded by 010/011 | Scoped typed owner, dimensionless contracts, native packaging/shared sim runtime |
| [010](ADR-010-preserve-ui-and-name-sensor-contracts.md) | Accepted; partly superseded by 011 | Preserve UI, explicit sensor/motor contracts, direct just dispatch |
| [011](ADR-011-remove-custom-architecture-checker.md) | Accepted | Remove custom architecture checker; keep native build/lint/behaviour tests |
| [012](ADR-012-action-driven-simulation.md) | Accepted; partly superseded by 013 | Action-driven simulator, shared Drive, flat core/native layout; chưa chọn RL làm controller |
| [013](ADR-013-platformio-only.md) | Accepted; hook/sanitizer superseded by 015 | PlatformIO CLI duy nhất cho native/ESP32; CMake nội bộ ESP-IDF không là frontend riêng |
| [014](ADR-014-main-a4950-prototype.md) | Superseded by 019; giữ lịch sử PCB | MAIN A4950: envelope, copper/via/EP process; not qualified/released |
| [015](ADR-015-build-and-hardware-verification.md) | Accepted cho build/verification; MAIN superseded by 019 | Native không hook riêng; KiCad/calculation/bench đúng phạm vi, bỏ gate SPICE giả định |
| [016](ADR-016-main-fixed-placement-routing.md) | Superseded by 019; giữ lịch sử routing | MAIN routing bằng công cụ đã cài, thay kế hoạch R01–R12 |
| [017](ADR-017-main-connectivity-placement.md) | Superseded by 019; giữ lịch sử placement | MAIN placement V1 theo connectivity, không đổi pinout/footprint/keepout |
| [018](ADR-018-main-spaced-placement.md) | Superseded by 019; giữ lịch sử placement | Nhận placement V2; kiểm chứng OUT/return/thermal trước full routing, giữ envelope điện |
| [019](ADR-019-module-based-hardware.md) | Accepted; firmware/hardware chưa kiểm chứng | Dừng PCB riêng; IR analog 8 mắt, hai DRV8871, pin 3S và pin map module |

ADR-013 thay phần frontend native CMake của ADR-012. ADR-012 thay một phần ADR-009/010; các ADR đó vẫn Accepted và không bị xóa. [ARCHITECTURE](../ARCHITECTURE.md) mô tả thiết kế hiện hành, [IMPLEMENTATION](../IMPLEMENTATION.md) tách phần đã chạy khỏi phần còn thiếu.

Giu so thu tu da dung; khong danh so lai de lap cac khoang trong. ADR cu duoc giu lam lich su, danh dau Superseded khi co quyet dinh thay the. Chi tao ADR cho lua chon co danh doi dang ghi lai; khong tao mot ADR cho moi ham hay file.
