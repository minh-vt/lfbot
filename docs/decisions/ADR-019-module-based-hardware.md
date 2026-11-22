# ADR-019: Dừng PCB riêng, dùng module cho prototype 3S

**Status:** Accepted theo quyết định của chủ dự án trong phiên làm việc 2026-11-10. Mô tả nút và mạch đo pin được thay bởi [ADR-021](ADR-021-field-adaptive-two-run-design.md); phần “firmware chưa migrate” được cập nhật bởi [ADR-020](ADR-020-esp32-firmware-restart.md). Phần dừng PCB riêng được thay bởi [ADR-025](ADR-025-pcb-base-carrying-modules.md) cho một PCB base mang module; MAIN và IR-03 vẫn dừng. Chốt phân bổ GPIO và cấu hình module; không phải nghiệm thu phần cứng hoặc firmware.

## Bối cảnh

Chủ dự án dừng chế tạo PCB riêng vì khó mua A4950 và cảm biến của BOM cũ, chuyển sang phần cứng mua sẵn. Đã xác nhận giữ pin 3S, motor 12 V, có ESP32-S3, buck 5 V, ToF và GY-9250. Đã mua một thanh IR analog 8 mắt và hai MKE-S16. Không yêu cầu người dùng thử A/B các module để mới đưa ra lựa chọn thiết kế.

## Quyết định

- Một thanh **IR analog 8 mắt** làm cảm biến line chính; hai MKE-S16 để dự phòng, không ghép thành cụm line chính 10 mắt.
- Giữ LiPo thường 3S (đầy 12,6 V) và hai motor GA12-N20 encoder 12 V. Loại DRV8833 khỏi phương án vì VM làm việc chỉ tới 10,8 V; PWM không hạ điện áp IC phải chịu.
- Chọn **hai module DRV8871**, mỗi module một motor, thay hai A4950 tích hợp. DRV8871 làm việc 6,5–45 V và nhận logic 3,3 V; không diễn giải 3,6 A đỉnh thành dòng liên tục. Chưa xác nhận người dùng đã mua driver mới.
- [hardware/wiring.md](../../hardware/wiring.md) là nguồn phân bổ GPIO/đấu dây hiện hành. Dùng tên GPIO, không số pad của MAIN hoặc vị trí header DevKitC suy đoán cho TDM2533. [BOM](../HARDWARE_BOM.csv) tách thiết bị đã xác nhận có, vật tư đề xuất và phần dự phòng.
- Dừng fabrication/routing của MAIN và IR-03. Giữ nguyên source CAD, local libraries, datasheets, mechanical models và lịch sử cho tới khi có duyệt cleanup riêng. Không gửi Gerber cũ đi in.

Lựa chọn thanh IR ưu tiên analog trực tiếp ADC1, một cụm liền và ít giao diện hơn hai MKE. Không tuyên bố tám mắt luôn tốt hơn mười mắt, hoặc module đã đạt contrast/timing của robot. Chọn DRV8871 vì hợp nguồn 3S và tìm được nhiều nguồn module trong nước, thay vì tiếp tục phụ thuộc IC A4950 rời. Giá/tồn website không phải cam kết giao hàng.

## Quyết định bị thay thế

- ADR-014/016/017/018: các bước PCB riêng đã dừng và được xoá khỏi tài liệu; quyết định dừng ghi ở chính ADR này.
- [ADR-015](ADR-015-build-and-hardware-verification.md): phần workflow MAIN không còn dùng. Quyết định build native và nguyên tắc phân biệt kiểm tra công cụ với kiểm chứng phần cứng vẫn giữ.
- Không thay PlatformIO/ESP-IDF, contract core/sim, luật hai lượt hoặc kiến trúc Robot/Drive/Race. Không xóa các ADR phần mềm vì đổi driver.

## Hệ quả và phần chưa triển khai

- Adapter `firmware/src/esp32/` còn phụ thuộc core đã bỏ, pin map khác và ba VL53L0X. Nó **không sẵn sàng upload/actuation**. Đổi tài liệu không tự migrate code; ESP32/HIL vẫn deferred.
- Core hiện có tám IR, một ToF và hai wheel counters, chưa có producer/consumer IMU. Có GY-9250 không chứng minh sensor fusion đã có.
- Geometry/calibration sim, lịch LED off/on IR-03 và giới hạn nguồn/nhiệt A4950 không chuyển nguyên sang module mới. Current limit của DRV8871 phụ thuộc điện trở ILIM module, không mặc định là 1 A của mạch cũ.
- Cleanup phải có danh sách duyệt, kiểm tra references và recovery cho dữ liệu untracked/ignored. Không coi CAD, datasheet, lock hoặc thư viện đang được tham chiếu là file rác.

Nguồn thông số và sơ đồ đấu chi tiết được giữ tại [wiring](../../hardware/wiring.md#nguồn-đối-chiếu), không sao chép bảng GPIO vào ADR.
