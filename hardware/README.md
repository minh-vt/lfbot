# Phần cứng LF-Bot

Thiết kế phần cứng duy nhất đang hoạt động: dải cảm biến IR-03 (`ir-array/`).

## Bắt đầu

- [IR project](ir-array/ir-array.kicad_pro): board 2 lớp 85×24.5 mm, 8×TCRT5000, header đực 1×12.
- [File Gerber để gửi in](ir-array/release/ir-array-gerber-thegioiic.zip): board final, DRC sạch, đã verify.
- [Review IR/ADC/DMA](ir-array/REVIEW.md), [giới hạn/căn cứ IR](ir-array/README.md), [log tối ưu route/DFM](ir-array/OPTIMIZATION_REVIEW.md).
- [BOM mua hàng duy nhất](../docs/HARDWARE_BOM.csv).

## Thư viện

Ba file tùy biến được giữ trong repo:

- `ir-array/IR_Array.kicad_sym`: symbol TCRT5000 (ánh xạ pad 1=C/2=E/3=K/4=A).
- `ir-array/AO3400A.kicad_sym`: symbol MOSFET AO3400A.
- `ir-array/IR_Array.pretty/Outline_IR03.kicad_mod`: contour bo mạch 85×24.5 mm.

Footprint và symbol chuẩn (TCRT5000, điện trở 0805, tụ, header 1×12, SOT-23, lỗ vít) không copy vào repo; chúng được nạp từ thư viện global của KiCad (giống hệt KiCad 10 stock).

## Đã retire

Toàn bộ project chức năng cũ (root `lfbot` + sheet `front`/`power`/`controller`), project `carrier` rỗng, symbol library cũ, bản cơ khí M03, tài liệu cũ và các thư viện footprint không còn tham chiếu nằm trong [`_archive/`](_archive/). Xóa thư mục này khi không cần phục hồi nữa.
