# ADR-008: Chung co che bam duong cho hai luot va ne vat can

## Status
Superseded by [ADR-009](ADR-009-scoped-robot-native-runtime.md); giu lich su huong kien truc ben duoi.

## Date
2026-10-13

## Context
Robot can giu chien luoc hai luot tren sa ban bi mat. Tach React cho luot dau, Follow cho luot sau va controller ne vat can se tao ba noi quyet dinh chuyen dong, kho giu hanh vi nhat quan.

Y tuong ne la "line uon quanh gach": thay doi duong muon di thay vi gia mao so do hay chuyen sang chuoi pha ne rieng.

## Decision
- Dung chung co che bam duong. Luot dau ghi thong tin da quan sat/thuc hien; luot sau khai thac ke hoach tu ban ghi cung so do moi.
- Ne vat can thay doi duong muc tieu, khong sua Readings va khong them FSM/controller ne rieng.
- Lap ke hoach giua hai luot tren robot, ngoai vong dieu khien; khong dua ban do ground truth cua sim vao ke hoach.
- Giu multi-pass xu ly co thu tu, dau vao/dau ra va quyen ghi ro rang. Day khong phai hai luot thi dau, cung khong yeu cau pass-manager framework. So pass va bieu dien trung gian con mo.

## Alternatives Considered
- **Bo chien luoc hai luot** — don gian hon nhung khong dap ung yeu cau; khong chon.
- **Controller rieng theo luot/pha ne** — de trien khai tung nhanh nhung lap lai chinh sach lai/toc do; khong chon.
- **Phat lai lenh motor theo thoi gian** — khong tu xu ly duoc troi vi tri hay dieu kien thay doi; khong chon.
- **Suy toan bo hanh lang tu dau vet Run 1** — khong du bang chung ve line lan can/vung hop le; khong chap nhan lam tien de.

## Consequences
- Thong tin duong muc tieu thay doi, co che bam duong khong bi nhan ban.
- Phai giai quyet bieu dien duong, dinh vi theo ban ghi, sai so va tinh hop le cua duong vong; khong the coi day la mot phep cong offset da du.
- Sensor cong khong chung minh controller chi can mot bieu thuc. Hinh hoc, toc do va an toan can kiem chung doc lap.
- Kiem thu phai bao gom hai luot va ne vat can qua cung contract; pass trong sim khong thay the do tren robot.

Chi tiet hien hanh va cac cau hoi mo: [ARCHITECTURE.md](../ARCHITECTURE.md).
