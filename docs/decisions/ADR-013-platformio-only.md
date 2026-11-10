# ADR-013: PlatformIO CLI là frontend build duy nhất

**Status:** Accepted theo yêu cầu chủ dự án. Phần hook `tools/native.py` và custom target sanitizer được thay bởi [ADR-015](ADR-015-build-and-hardware-verification.md); các mục tương ứng bên dưới là lịch sử. Thay phần frontend native CMake của [ADR-012](ADR-012-action-driven-simulation.md); giữ các quyết định engine/core/ownership và ESP32 deferred.

## Bối cảnh

Chủ dự án yêu cầu dùng duy nhất PlatformIO cho toàn bộ project. Tiếp tục gọi CMake/CTest cho native là trái yêu cầu, dù các tests có pass. Đổi frontend không sửa được controller hoặc biến dừng trước gạch thành né thành công.

## Quyết định

- `firmware/platformio.ini` khai báo native GUI, evaluator và từng test bằng source filters explicit; test engine không compile Robot/Race/Drive/Route. `esp32s3` vẫn deferred.
- `justfile` chỉ dispatch PIO cho build, thực thi tests, compile database, sanitizer và upload; format/lint dùng công cụ tương ứng. Không giữ root CMake hoặc recipe bot riêng.
- `tools/native.py` thêm dependency flags qua pkg-config và target sanitizer vào PIO/SCons. Headers bên ngoài là system headers; không tắt warnings cho project.
- CI dùng cùng recipes. CMake bên trong ESP-IDF vẫn do PlatformIO quản lý, không là frontend hay quy trình build riêng.

## Hệ quả

Native cần compiler C++17, pkg-config, libpng, nlohmann-json và Raylib cho GUI. Compile database của từng environment được giữ riêng để lint không trộn source membership; database GUI phục vụ clangd. ASan/UBSan chạy qua target PIO.

Root CMake cũ được giữ recovery ngoài repository trước khi bỏ. Lịch sử kết quả CMake/CTest không bị xóa, nhưng không còn là hướng dẫn hiện hành. Tests hợp đồng pass không chứng minh vòng tránh gạch hoặc hoàn thành hai lượt; các hành vi đó phải được đánh giá riêng.
