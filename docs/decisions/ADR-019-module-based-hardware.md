# ADR-019: Dùng module mua sẵn cho phần cứng 3S

**Status:** Accepted, chủ dự án quyết định ngày 2026-11-10. Module được gắn lên PCB base theo [ADR-025](ADR-025-pcb-base-carrying-modules.md).

## Bối cảnh

Linh kiện của các PCB tự thiết kế trước đó (A4950, cảm biến rời) khó mua. Chủ dự án chuyển sang module có sẵn, giữ pin 3S, motor 12 V, ESP32-S3, buck 5 V, ToF và GY-9250.

## Quyết định

- Một thanh **IR analog 8 mắt** làm cảm biến line, đọc thẳng bằng ADC1. Hai MKE-S16 để dự phòng, không ghép thành cụm line chính.
- LiPo 3S thường (đầy 12,6 V) và hai motor GA12-N20 encoder 12 V.
- **Hai module DRV8871**, mỗi module một motor: làm việc 6,5–45 V, nhận logic 3,3 V. Không dùng DRV8833 (VM chỉ tới 10,8 V). 3,6 A là dòng đỉnh, không phải dòng liên tục; giới hạn dòng phụ thuộc điện trở ILIM trên module.
- [hardware/wiring.md](../../hardware/wiring.md) là nguồn duy nhất của gán GPIO và cách đấu; [BOM](../HARDWARE_BOM.csv) tách thiết bị đã có, vật tư đề xuất và dự phòng.

## Hệ quả

- Thông số của module (contrast thanh IR, dòng kẹt motor, ILIM của DRV) chưa đo; số nào cần thì đo trên xe, không suy từ shop.
- Đổi module thì kiểm lại wiring và các hằng số lắp ráp trong `config.hpp`.
