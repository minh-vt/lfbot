# ADR-010: Giu UI va lam ro hop dong cam bien khi sua native migration

**Status:** Accepted; thay the mot phan ADR-009 ve vi tri/ten cac data contracts va cach migrate UI. Phan giu custom architecture tests duoc thay the boi [ADR-011](ADR-011-remove-custom-architecture-checker.md). Scoped ownership, portability va controller direction cua ADR-009 van giu nguyen.

## Context

Migration da thay renderer bang ban rut gon thay vi giu UI, trong khi Race van not-ready. Build/architecture checks khong chung minh UI duoc bao toan hay robot chay duoc. `RobotModel { bool valid; }` khong phai mo hinh robot; `Measurements`, `Readings` va `DriveResult` khong lam ro du lieu/ownership.

## Decision

- Khoi phuc renderer tu snapshot truoc migration: giu docks, camera, chassis/ghost va trail cache; thay data binding, khong mang controller cu tro lai. Du lieu chua co thi N/A. Chi mot front ToF, khong phuc hoi hai side channels gia.
- `SensorMeasurements` la acquisition theo don vi vat ly; `SensorReadings` la sensor input da chuan hoa. Schema ca hai nam trong `robot_contract`; Robot so huu conversion, Race chi doc so do. Model/control state khong nam trong schema nay.
- `MotorDemand` la yeu cau hai motor voi duration theo T0; Robot chuyen thanh `MotorOutput` co physical deadline. Bo `DriveResult`: Race tra demand va so huu status doc qua const accessor. Bo model chi co bool; khong khai bao mo hinh thay the khi chua co du lieu that su duoc su dung.
- Native simulation geometry duoc khai bao mot noi cho sampling va drawing, gan nhan simulation proposal. Khong coi chung la calibration phan cung.
- `just` dispatch truc tiep format/lint/build. Chi giu script merge compile databases va compiler-backed architecture tests vi chung kiem tra cac ranh gioi compiler thong thuong khong tu cam. Khong them custom CLI cho cac lenh co san.

## Consequences

App-facing raw sensor type duoc doi ten; khong compatibility alias cho ten mo ho. Borrowed Robot lifetime va core/platform seam khong doi. UI regression co fixture/captures rieng, khong la controller/race test. Controller mot luot, hai luot, detour va full UI acceptance van can implementation/kiem chung; ESP32/HIL van deferred.
