# Kế hoạch

Theo [ARCHITECTURE](ARCHITECTURE.md). Code của cả hai lượt đã có và qua test host ([IMPLEMENTATION](IMPLEMENTATION.md)); việc còn lại là làm board, lắp xe và nghiệm thu từng phần trên xe thật. Build và test xanh chỉ là điều kiện cần. Chung kết 2026-12-02.

## Quy tắc làm việc

- Motor chỉ chạy khi chủ dự án nói "chạy" và đang quan sát.
- Mỗi lần sửa: `just check` → nạp → chạy trên xe → đổ `log` → ghi kết quả vào [IMPLEMENTATION](IMPLEMENTATION.md).
- Mở `/dev/ttyACM0` luôn với `dtr=False, rts=False`. Mở cổng làm board khởi động lại.
- Số chỉnh trên đường thử ghi vào `config.hpp` kèm ngày và cách đo.

## Chuẩn bị (chủ dự án)

| Việc | Dùng cho |
|---|---|
| Đường thử: băng đen 20 mm trên nền trắng, có đoạn thẳng ≥ 1 m, góc 90°, góc nhọn, giao cắt chữ thập, đầu cụt | Bước 3–4 |
| Một viên gạch hoặc hộp 205×95×95 mm | Bước 5 |

## Bước 1 — Board 1 mặt

Theo [single/README](../hardware/base/single/README.md).

- [ ] Ăn mòn, khoan, hàn; kiểm thông GND, cầu nối và dây nhảy, không chập nguồn trước khi cắm module.
- [ ] Chỉnh buck lên 5,2–5,3 V khi chưa cắm ESP32.
- [ ] Đối chiếu chữ in trên devkit TDM2533: U1.8 = GPIO15, U1.10 = GPIO17.

## Bước 2 — Bring-up trên board

- [ ] `monitor`: IMU `0x70`, ToF `0xEACC`, IR đổi theo giấy trắng/đen, pin khớp đồng hồ vạn năng.
- [ ] `irctl high|low`: biết chân IR của thanh có bật/tắt LED không, ghi `pins::kIrCtrlWiring`.
- [ ] Nhấc bánh, cho motor tăng tốc và đổi chiều: IR trong `log` không nhảy theo motor (GND chỉ đổ một mặt).
- [ ] Đo `kBarAheadMm` (mắt tới trục) và `kEyePitchMm` (khoảng cách hai mắt).
- [ ] Bấm nút giữa lúc chạy: dừng ngay, LED vàng. Rút USB giữa lúc chạy: motor dừng.
- [ ] `kNodeCrawlDuty` thắng được vùng chết trên sàn.

## Bước 3 — Bám line lượt 1

- [ ] Bấm nút: LED cyan, xe tự quét, LED tím; bấm lần 2: xanh lá và bám.
- [ ] Lấy nhật ký IR thật, chốt ngưỡng `Wide` / `Split`; dựng test hồi quy cho phần đọc line từ nhật ký đó.
- [ ] Qua góc 90° cả hai chiều, góc nhọn, giao cắt chữ thập; đến đầu cụt thì dừng trong ô đích, LED trắng.
- [ ] Góc gắt và giao cắt **không** bị báo đích.
- [ ] Ba lần liên tiếp đi hết đường thử mà không chạm tay.

## Bước 4 — Lượt 2

- [ ] Lượt 1 về đích, LED trắng; rút pin, cắm lại: LED hồng.
- [ ] Lượt 2 về đích, **nhanh hơn lượt 1** theo đồng hồ. Chỉnh `kFastPace`, `kHardEffort`, `kLookaheadTicks` theo cột `pace_pct`.
- [ ] Đặt thêm vật làm lệch một đoạn: lượt 2 không mất line, về nhịp lượt 1 tới mốc kế tiếp.
- [ ] Giữ nút lúc cắm pin: bản ghi bị xoá, LED xanh dương.

## Bước 5 — Né gạch

- [ ] Gạch đặt ngang line, dọc line, lệch trái, lệch phải: xe vòng bên phải, không đẩy đổ gạch (chạm nhẹ hợp lệ theo Q-14), bắt lại đúng line.
- [ ] Gạch gần một góc: không bắt nhầm sang đoạn line khác.
- [ ] Gạch ở lượt 2, trên đoạn lượt 1 đi nhanh: vẫn vòng được. Nếu tới quá nhanh, thêm luật ToF giảm tốc (IMPLEMENTATION).

## Rủi ro mở

| Rủi ro | Xử lý |
|---|---|
| JP1 (0 Ω 1206) nằm trên đường cấp nguồn motor; dòng kẹt N20 chưa đo | Thay bằng dây nhảy có vỏ hoặc 0 Ω có dòng định mức đủ |
| Pad JST ZH vành 0,15 mm dễ đứt khi khoan tay | Khoan đúng chấm tâm, mũi 0,7 mm; kiểm thông từng chân encoder |
| Ánh sáng sân (Q-16; chung kết trong nhà) làm lệch IR | Tự quét ở mỗi lần Start; đọc bật/tắt LED nếu thanh cho phép |
| Gạch đặt sát một đoạn line khác hoặc gần góc 90° | Hành lang nhận lại line; nghiệm thu riêng ở bước 5 |
| Motor N20 thiếu mô-men khi tăng tốc lượt 2 | Chỉnh nhịp và giới hạn đổi tốc trên đường thử |
