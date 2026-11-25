# ADR-013: PlatformIO CLI là frontend build duy nhất

**Status:** Accepted theo yêu cầu chủ dự án.

## Bối cảnh

Chủ dự án yêu cầu một cách build duy nhất cho cả firmware ESP32 lẫn test trên máy tính. Hai hệ build song song (CMake/CTest riêng cho host, PlatformIO cho board) làm lệch danh sách file và cờ biên dịch giữa hai nơi.

## Quyết định

- [firmware/platformio.ini](../../firmware/platformio.ini) khai báo hai env: `esp32s3` dịch firmware (ESP-IDF, CMake bên trong do PlatformIO quản lý) và `test` dịch core cùng test hợp đồng trên host.
- [justfile](../../justfile) chỉ gọi PlatformIO cho build, test, compile database, sanitizer và upload; format/lint gọi clang-format/clang-tidy. Không có CMake gốc hay script build riêng.
- CI chạy cùng các recipe đó.

## Hệ quả

Danh sách file của mỗi env nằm một chỗ. Test hợp đồng pass không chứng minh robot chạy; hành vi được nghiệm thu trên xe.
