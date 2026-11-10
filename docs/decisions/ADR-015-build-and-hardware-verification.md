# ADR-015: Build native không hook riêng; kiểm chứng phần cứng đúng phạm vi

> Phần workflow MAIN được thay bởi [ADR-019](ADR-019-module-based-hardware.md). Quyết định build native và nguyên tắc kiểm chứng đúng phạm vi vẫn giữ.

**Status:** Accepted theo yêu cầu chủ dự án, 2026-11-01. Thay phần hook `tools/native.py`/target sanitizer của [ADR-013](ADR-013-platformio-only.md) và vai trò ngspice bắt buộc của [ADR-014](ADR-014-main-a4950-prototype.md). Giữ PlatformIO-only, source membership, envelope prototype và các gate fit/release/bring-up.

**Cập nhật phạm vi MAIN:** [ADR-016](ADR-016-main-fixed-placement-routing.md) thay workflow R01–R12 và cách giữ candidate/report; không thay quyết định build hoặc nguyên tắc phân biệt CAD/calculation/bench của ADR này.

## Bối cảnh

Build phụ thuộc script đã mất; option `custom_*` không còn bên xử lý. Các deck MAIN viết tay không tự đồng bộ với CAD và dùng tham số tải/nguồn chưa xác minh. Giữ kết quả số của chúng bằng regression không kiểm chứng MAIN; manifest mỗi lần chạy không khắc phục khoảng cách đó.

## Quyết định

- Khai báo flags và `pkg-config` trực tiếp trong `platformio.ini`; chuyển include ngoài từ `-I` sang `-isystem`, không hạ warnings của project. Dùng native target `exec` và `--program-arg=--self-test` có sẵn của PlatformIO; không viết hook thay thế.
- `just sanitize` thêm ASan/UBSan bằng `PLATFORMIO_BUILD_FLAGS`, build riêng trong `firmware/.pio/sanitize`, rồi gọi cùng recipes test. Không có custom target `sanitize`; không trộn object thường/sanitized.
- Bỏ recipe/deck/regression MAIN SPICE và output phụ trợ khỏi nguồn đang dùng. Bản trước cleanup có recovery ngoài repository; R01 baseline giữ nguyên.
- KiCad ERC/DRC/parity kiểm tra CAD; datasheet và phép tính ngắn kiểm tra lựa chọn linh kiện/copper; bench kiểm tra nguồn, motor, tín hiệu và nhiệt. Không dùng một loại làm bằng chứng cho loại khác.
- Phép tính ADC/I²C đặt trong routing spec. Mô phỏng chỉ bổ sung khi có câu hỏi thiết kế cụ thể, model/đầu vào đủ căn cứ và ảnh hưởng được quyết định; không là gate mặc định.
- Output mới sinh vào `hardware/.generated/` được ignore. Giữ nguồn, kết luận và baseline lịch sử; chỉ đóng gói/hash bộ bằng chứng tại mốc release cần truy xuất. Không ignore rộng các đuôi file nguồn hoặc sửa lịch sử hash.
- KiCad giữ connectivity/geometry/rules; BOM giữ MPN/lắp/DNP; spec giữ giới hạn và phép tính; ADR giữ lý do; tickets giữ công việc/điều kiện ra. Không thêm parser hoặc file cấu hình đồng bộ riêng.

## Hệ quả

Bỏ gate ngspice không nới giới hạn an toàn hoặc biến giả định thành số đo. R02 đóng quyết định, R03 áp dụng rules, R04/R11 xác nhận fit/process, R12 xác nhận trên PCB đã lắp. Nguồn hấp thụ hồi dòng vẫn phải được kiểm tra trước thử motor; không suy C7 hoặc nhãn pack protected là đủ.

Native vẫn cần compiler C++17, pkg-config, libpng và Raylib cho GUI. `just` dispatch công cụ chuẩn; không trở thành build system khác. ESP32/HIL vẫn deferred.

## Căn cứ

- [PlatformIO dynamic build flags](https://docs.platformio.org/en/latest/projectconf/sections/env/options/build/build_flags.html#dynamic-build-flags), [environment variables](https://docs.platformio.org/en/latest/envvars.html#building).
- Native platform 1.2.1 `builder/main.py` khai báo `exec` dùng `$PROGRAM_ARGS`; PlatformIO Core 6.2.0 hỗ trợ `--program-arg` và nối thêm `PLATFORMIO_BUILD_FLAGS` vào flags cấu hình. Kiểm chứng bằng recipes của repository, không sao chép builder vào project.
