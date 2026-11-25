# ADR-020: Core độc lập phần cứng, driver ESP32 tách riêng

**Status:** Accepted theo yêu cầu chủ dự án ngày 2026-11-10. Bố cục thư mục cập nhật ngày 2026-11-22.

## Bối cảnh

Robot chỉ có ý nghĩa trên xe thật, nhưng logic quyết định (bám line, an toàn, hai lượt) cần kiểm được trên máy tính mà không cần board. Chủ dự án không muốn simulator hay mô hình vật lý: số đo lấy từ robot và sân thật.

## Quyết định

- **[firmware/core/](../../firmware/core/)**: C++17 thuần, phẳng, không ESP-IDF, FreeRTOS, file I/O, `printf` hay biến toàn cục. Test trên host bằng env `test`.
- **[firmware/esp32/](../../firmware/esp32/)**: chương trình ESP32. `drivers/` mỗi thiết bị một lớp nhỏ, `storage/` các vùng flash, cùng `main.cpp`, `console.*`, `pins.hpp`. Core không gọi driver; platform không quyết định chuyển động.
- VL53L1X dùng ST ULD (vendor, đã vá vòng chờ có timeout; xem `third_party/vl53l1x/README.md`).
- Chỉ hai số vật lý do chủ dự án đo là giá trị khởi đầu: bánh 43 mm, bề ngang mép ngoài 110 mm (**không phải** khoảng cách hai bánh). Còn lại đo trên robot hoặc robot tự đo tại sân.
- Test trên host chỉ kiểm hợp đồng và bất biến, không chứng minh robot chạy.

## Hệ quả

Core dịch được ở cả hai env; thay board hay driver không đụng tới core.
