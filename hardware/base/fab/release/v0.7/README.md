# BASE v0.7: file đặt in và danh sách mua linh kiện

Xuất ngày 2026-11-19 từ [base.kicad_pcb](../../base.kicad_pcb). Mặt sau board in chữ "LF-Bot base v0.7 - 2026-11-19".

- **[LF-Bot_base_v0.7_gerber.zip](LF-Bot_base_v0.7_gerber.zip):** file gửi xưởng. Gồm 7 lớp Gerber (đồng, mặt nạ hàn và chữ ở hai mặt, viền board), file khoan Excellon tách lỗ mạ (PTH) và lỗ không mạ (NPTH), và file job. Không có lớp kem hàn vì board hàn tay, không cần stencil.
- **[BOM_mua_linh_kien.csv](BOM_mua_linh_kien.csv):** danh sách mua, gồm cả giắc, công tắc, dây, vít và module đã có. Mở được bằng Excel hoặc Google Sheets.

## Thông số khi đặt in

Điền vào form đặt hàng (tên mục theo JLCPCB; xưởng khác có mục tương đương):

| Mục | Giá trị |
|---|---|
| Base material | FR-4 |
| Layers | 2 |
| Dimensions | 73,1 × 116,9 mm (xưởng tự đọc từ file) |
| PCB thickness | 1,6 mm |
| Outer copper weight | 1 oz |
| Surface finish | HASL (có chì hoặc không chì đều được) |
| Mask / silkscreen color | Tuỳ chọn; xanh lá + chữ trắng thường nhanh và rẻ nhất |
| Via covering | Tented (board thiết kế phủ mặt nạ hàn lên via ở cả hai mặt) |
| Min via hole / diameter | 0,4 / 0,8 mm (mức tiêu chuẩn là đủ) |
| Min track / spacing | 0,3 / 0,2 mm (mức tiêu chuẩn 6/6 mil là đủ) |
| Gold fingers, castellated holes, impedance control | Không |
| Remove order number | Tuỳ. Để mặc định thì xưởng in mã đơn vào một chỗ trống trên board |

Board có 4 khe không mạ (2 khe M3 cho thanh IR, 2 khe M2 cho bi cầu), nằm trong `base-NPTH.drl` dưới dạng lệnh khe G85. Kiểm ở trang xem trước của xưởng rằng 4 khe hiện đúng hình chữ nhật bo tròn, không chỉ là lỗ tròn.

**Đã kiểm trên đúng gói này:**
- DRC theo luật đã siết theo năng lực xưởng, ERC và kiểm khớp schematic đều sạch.
- `analyze_gerbers` không báo gì; đủ lớp, các lớp thẳng hàng với viền board.
- 36 via, 143 lỗ chân linh kiện, 6 lỗ bắt.

## Nên làm trước khi bấm đặt

1. In [fitcheck_top_1to1.pdf](../../../fitcheck/fitcheck_top_1to1.pdf) đúng 100%, đặt devkit, DRV, gá motor, thanh IR, bi cầu, buck lên giấy để kiểm lỗ và khoảng hở. Kích thước thân module ước từ ảnh; lỗ lệch thì board phải bỏ.
2. Đo thông mạch dây ZH của encoder (nối thẳng hay đảo đầu).
3. Kiểm vỏ bi cầu không quá Ø23 mm.

## Thứ tự lắp gợi ý

1. Linh kiện dán trước: 18 điện trở, C2, D1. Hàng 10k ở trước, hàng 47k ở sau, cột trái nhất là kênh D1. D1 có vạch quay về phía +5V.
2. Linh kiện thấp: nút, giắc ZH, hàng rào đực.
3. GY-9250 hàn phẳng ở J6 và buck MP1584 hàn chân ở U4, cả hai trước khi gắn hàng rào cái của ESP32 (chúng nằm dưới gầm ESP32).
4. Hàng rào cái của ESP32, DRV, ToF; C1, F1; dây pin, dây công tắc.
5. Cấp nguồn khi **chưa cắm ESP32**. Đo 5 V ở chân 5V và chỉnh buck lên khoảng 5,2–5,3 V, rồi mới cắm ESP32 và các module.

Cách đấu từng module: [wiring.md](../../../../wiring.md).
