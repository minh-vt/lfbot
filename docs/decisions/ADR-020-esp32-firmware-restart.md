# ADR-020: Viết lại firmware ESP32, bỏ simulator và core cũ

**Status:** Accepted theo yêu cầu chủ dự án ngày 2026-11-10. Thay các phần ADR-009/010/012/013 về simulator, `Drive`/`Race`/`RaceTrial` và recipe native sim. Giữ PlatformIO-only (ADR-013), ESP-IDF, luật hai lượt và wiring ADR-019.

## Bối cảnh

Phần cứng đã có trên tay. Adapter cũ sai pin map, dùng ba VL53L0X và finish GPIO giả. Controller cũ không hoàn thành đường sim nào. Chủ dự án yêu cầu tập trung vào firmware ESP32, hiệu chuẩn tại sân thật, không dựa vào số vật lý giả định.

## Quyết định

- Xóa `firmware/lib/robot`, `src/sim`, `test` cũ. Recovery ngoài repo: `~/.local/state/lfbot/recovery/esp32-reset-20261110-*`.
- Chỉ hai số vật lý do chủ dự án đo được lưu: đường kính bánh 43 mm; bề ngang mép ngoài hai bánh 110 mm (**không phải wheelbase**). Wheelbase, scale encoder, IR trắng/đen, gyro bias đều phải đo trên robot và sân thật; thiếu thì không arm.
- Nút GPIO40 là Start/Stop cho cả bắt đầu và kết thúc lượt; không có cảm biến đích.
- Core `firmware/lib/robot/` độc lập ESP-IDF, test native. `firmware/src/esp32/` giữ driver. VL53L1X dùng ST ULD (vendor, đã vá vòng chờ có timeout; xem `third_party/vl53l1x/README.md`).
- Native tests chỉ kiểm hợp đồng toán/lỗi, không chứng minh robot chạy.

## Chưa làm

Điều khiển motor/MCPWM, vòng tốc độ, bám line, né gạch, map, lượt 2, lưu NVS đã thử trên board. Kết quả hiện chỉ là: cross-compile + host tests. Chưa nạp lên board.
