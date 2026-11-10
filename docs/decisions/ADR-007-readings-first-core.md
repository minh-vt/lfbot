# ADR-007: Giu so do lam contract cua core

## Status
Superseded by [ADR-009](ADR-009-scoped-robot-native-runtime.md). Truoc day thay the [ADR-001](ADR-001-two-layer-architecture.md); giu lich su ben duoi.

## Date
2026-10-13

## Context
Thiet ke cu rut gon so do thanh tin hieu dieu khien va rang buoc drive chi nhan dai luong khong-thu-nguyen. De xuat tiep theo them `LineView { ahead, side, certainty }`, nhung chua chung minh bieu dien nay giu du thong tin cho giao cat va goc vuong. Viec sua LineView de ne gach con tron so do voi y dinh dieu khien.

Can giu chung core cho sim/robot ma khong tao nhieu lop dien giai trung gian hay dua thong tin chi simulator biet vao robot.

## Decision
- Giu `sensing` (hieu chuan/chuan hoa) va `drive` (quyet dinh chuyen dong) doc lap phan cung. Nen tang chi so huu I/O, tai nguyen va lap lich.
- `Readings` giu so do tung kenh cung thong tin thoi gian/tinh hop le can thiet; drive khong sua chung. Khong dung LineView hay nhan ahead/side lam contract mac dinh.
- Tach so do, trang thai uoc luong, duong muc tieu va lenh motor. Dai luong suy ra can thiet van duoc tinh noi bo; khong moi ket qua trung gian mot public type.
- Contract quy dinh don vi/thang do, dau va timing. Khong giu rang buoc "co don vi thi chi duoc nam trong sim"; bo don vi cu the chua chot.
- Dung contract du lieu chung va ham nen tang, khong bat buoc hierarchy driver. Khong phan chia module theo tung ham.

## Alternatives Considered
- **LineView rut gon** — interface ngan nhung dong dinh mot phep dien giai chua duoc kiem chung; khong chon.
- **Sua so do de tao line ao** — controller khong biet vat can, nhung khong con phan biet duoc quan sat va muc tieu; khong chon.
- **Truyen raw hardware handles vao drive** — giu du thong tin nhung mat seam sim-to-real; khong chon.
- **Them perception/controller/driver class cho moi buoc** — nhieu interface hon nhu cau; chua can.

## Consequences
- Co the replay so do de tai hien quyet dinh ma khong can GUI hay phan cung.
- Readings lon hon mot offset rut gon; can gioi han kich thuoc, quyen ghi va vong doi ro rang, khong bien no thanh God object.
- Phai kiem chung hieu chuan va timing; so do khong dong nghia voi ground truth.
- Giu ranh gioi core/nen tang cua ADR-001, nhung bo cac rang buoc ve tin hieu rut gon va don vi. API cu khong phai contract cua dot tai thiet.

Chi tiet hien hanh: [ARCHITECTURE.md](../ARCHITECTURE.md).
