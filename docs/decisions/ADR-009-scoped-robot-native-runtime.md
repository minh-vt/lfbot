# ADR-009: Scoped Robot owner, dimensionless Race va native sim runtime

**Status:** Accepted; ten/ownership cua sensor contracts va UI migration duoc thay the mot phan boi [ADR-010](ADR-010-preserve-ui-and-name-sensor-contracts.md). Yeu cau custom architecture gate duoc thay the boi [ADR-011](ADR-011-remove-custom-architecture-checker.md). Scoped owner va cac quyet dinh khac van hieu luc; xem [implementation evidence](../IMPLEMENTATION.md).

## Context

 Chung chua chot construction, units, source ownership hay executable enforcement. Public concrete state va caller source globs lam controller substitution lan sang adapters. Sim hien tai con old contracts; core cu da duoc xoa mot phan.

## Decision

- `robot_contract` chi khai bao du lieu acquisition/calibration/output/request/status/observation, khong implementation. Robot chuan hoa bang L0=wheelbase, V0=reference speed, T0=L0/V0; Race chi thay so do da chuan hoa. Raw/per-channel evidence khong bi ep thanh offset/danger. (Ten/vi tri cac type nay duoc chot lai o [ADR-010](ADR-010-preserve-ui-and-name-sensor-contracts.md).)
- Public Robot la noncopyable borrowed facade voi private construction/destruction/incomplete State. `run_robot(const Calibration&, RobotSession&) noexcept` so huu typed State trong lexical frame, disable truoc init va khi callback return. RobotSession la virtual lifetime callback duy nhat, khong controller/driver plugin. Callback termination/stall khong duoc normal-return hook bao ve.
- Private State so huu calibration/conversion/scheduling va Race bang value; Race so huu control/history. Khong heap-PIMPL, caller storage/sizeof, hidden singleton hay placement-new. Native owner budget la software profile, khong ESP32 stack measurement. Nested owners can combined stack budget; C++17 khong ngan reference escape.
- Native packages so huu source lists/dependencies; applications khong compile private callee sources. Gates dung compiler-resolved paths/AST/build membership va positive/negative fixtures; regex khong du. Fresh compile database, empty-suite/stale-input failure va source-locality mutations la acceptance, khong convention.
- Sim GUI/headless se dung chung CPU Simulation/World/referee/loader/trace, chi simulation.cpp goi Robot.drive/observe va step World trong production. Renderer chi doc snapshots. Fixed logical tick 2 ms, pause/scrub khong mutate Robot. Native assets dung libpng/nlohmann; Raylib chi o GUI.
- Controller da duyet: ordered local 2D path, common pure-pursuit/near-zero heading handling va wheel-speed feedback; mot front ToF, curved IR, conservative brick envelope va bounded two-run speed preparation. Khong feedback truth/referee vao acquisition, khong fake IR de ne.
- Milestone hien tai chi sim+lib native. ESP32 source/build/HIL deferred, khong chan native acceptance va khong restore deleted core de cuu old ESP32 build.

## Consequences

Application source khong biet Race size/representation; recompile/relink van can. Borrowed lifetime, stack usage, watchdog va hardware calibration van can kiem chung, khong duoc suy ra tu access control. Gates ngan accidental misuse, khong sandbox nguoi sua toolchain/checker. CI configuration khong tu establish required branch protection.

ADR nay chot units, module ownership, construction va implementation direction. Research va approved breakdown duoc giu trong repo/scratch; architecture canonical khong duoc coi planning la runnable implementation.
