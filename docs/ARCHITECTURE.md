# Kiến trúc LF-Bot

## 1. Trạng thái và phạm vi

Thiết kế này đã được duyệt và triển khai một phần ngày 2026-10-19 theo [ADR-012](decisions/ADR-012-action-driven-simulation.md): core phẳng với `Drive` dùng chung, engine nhận `MotorOutput`, `Referee`, `RaceTrial`, và composition ngoài engine. Frontend build hiện hành là PlatformIO CLI cho toàn bộ project theo [ADR-013](decisions/ADR-013-platformio-only.md). [IMPLEMENTATION](IMPLEMENTATION.md) ghi phần đã chạy được và giới hạn; [PLAN](PLAN.md) giữ backlog còn mở. Tài liệu này không phải bằng chứng hoàn thành đường đua, timing hay an toàn phần cứng.

Phần cứng chuyển sang module mua sẵn theo [ADR-019](decisions/ADR-019-module-based-hardware.md); pin map hiện hành ở [wiring](../hardware/wiring.md). MAIN A4950 và IR-03 đã dừng, không còn là sơ đồ đấu dây. Thay đổi này chưa migrate adapter ESP32 hoặc thêm IMU vào contract core.

Chưa triển khai: environment/policy RL, chẩn đoán nguyên nhân DNF, GUI automation, ESP32/HIL. RL vẫn là **ứng viên cần đo**, không phải controller đã chọn. ESP32/ESP-IDF giữ nhãn deferred; không đổi OS.

Nguồn quy ước duy nhất là [NAMING](NAMING.md). Luật thi ở [manual](manual.md) và [FAQ](fqa.csv); khi mâu thuẫn phải làm rõ, không tự sáng tác luật. [BOM](HARDWARE_BOM.csv) và geometry sim là đề xuất, không phải calibration đã đo. ESP32/ESP-IDF/HIL vẫn là milestone riêng; không đổi OS.

## 2. Các quyết định thiết kế

1. **Simulator nhận lệnh motor, không nhận controller.** Engine không include/call/link Robot, Race hoặc RLtools; phụ thuộc dữ liệu cảm biến/motor là có chủ đích.
2. **Ghép chương trình ở bên ngoài engine.** GUI, đánh giá hai lượt và trainer là callers, không phải các nhánh trong `Simulation`.
3. **Một implementation normalization và kiểm tra motor.** Robot triển khai và môi trường training dùng cùng module `Drive`; không nhân bản math, counter wrap hoặc deadline logic.
4. **Giữ Robot là scoped owner cho đường chạy robot.** GUI/ESP32 chỉ borrow qua `run_robot`/`RobotSession`; không được dựng hoặc cấp storage cho private Robot/Race.
5. **Một package core phẳng, PlatformIO CLI là frontend build duy nhất.** Tách trách nhiệm không đòi mỗi class có package, manifest, `include/` và `src/` riêng.
6. **Không dựng controller framework.** Không `IController`, factory, registry, type erasure, `void*` context hoặc template hóa toàn bộ codebase để đổi controller. RL là ứng viên cần đo, không phải kiến trúc mặc định.

### Module và interface

| Module | Interface/caller cần biết | Implementation được che giấu |
|---|---|---|
| Dữ liệu robot | `SensorSample<T>`, `SensorMeasurements`, `SensorReadings`, `Calibration`, `MotorDemand`, `MotorOutput`, request/status | Không có implementation điều khiển, platform hay file I/O |
| `Drive` | Nhận calibration, đọc/chuẩn hóa một packet, kiểm tra demand và tạo output, báo fault, rebase khi bắt đầu lượt | Kiểm tra calibration, clock/counter history, đơn vị, age/validity, freshness theo nguồn, giới hạn và chuyển duration thành deadline |
| `Robot` / `Race` | `run_robot`, `Robot::drive/observe`, các yêu cầu lifecycle rõ nghĩa | Robot owns Drive + Race; Race owns quyết định chuyển động, trạng thái và trí nhớ hai lượt; representation private |
| `Simulation` | Borrow track bất biến; chuẩn bị/bắt đầu/dừng lượt, nhận `MotorOutput` để tiến một tick, đọc snapshot | World, acquisition cadence, logical clock, referee, trạng thái vật lý và kết quả một lượt |
| `RaceTrial` | Chạy tick với Robot đang borrow; yêu cầu start/stop/reset; đọc kết quả cả hai lượt | Ghép Robot với Simulation, handshake, reposition/rebase, accounting; không tự điều khiển motor |
| Renderer | Đọc frame/trace và trạng thái playback | Raylib, camera, docks, trails/ghost; không chạy vật lý/controller |
| Environment training | Operation RLtools cho state/action/observation/reward/terminal | Ghép Drive với Simulation và trạng thái episode; không sao chép physics, safety hoặc referee |

`Drive` không chỉ forward một lời gọi: nó gom các invariant conversion/output vốn nằm trong `robot.cpp`, được hai callers thực dùng. `RaceTrial` không phải episode manager tổng quát: chỉ giữ một chỗ cho orchestration Robot + sim mà GUI và kiểm thử hai lượt hiện đang cần. Không tạo thêm lớp bọc cho mỗi operation.

## 3. Cấu trúc thư mục đích

Đây là **layout đề xuất**, không phải danh sách file đã tạo. Chỉ tạo file training/model khi đến slice có caller thật.

```text
justfile                          dispatch các công cụ chuẩn
README.md                         mục đích, điểm bắt đầu, links
.clang-format / .clang-tidy

docs/
  ARCHITECTURE.md                  thiết kế và contract
  NAMING.md                       quy ước tên duy nhất
  PLAN.md                         công việc, regression, trạng thái
  IMPLEMENTATION.md               bằng chứng mới chạy, giới hạn
  decisions/                      quyết định và lịch sử
  research/                       nghiên cứu, không phải implementation

firmware/
  platformio.ini                  native GUI/tests/evaluator và ESP32 deferred
  CMakeLists.txt                  entry ESP-IDF, milestone riêng
  assets/                         các track fixtures hiện có
  lib/
    robot/
      robot_data.hpp              dữ liệu chung; không include Robot/Race
      drive.hpp / drive.cpp       conversion + output safety dùng chung
      robot.hpp / robot.cpp       scoped facade/owner, interface platform
      race.hpp / race.cpp         lifecycle, controller, memory; private
      route.hpp / route.cpp       acquisition-time estimate + bounded prefix map; private
  src/
    CMakeLists.txt                registration ESP-IDF, không native frontend
    sim/
      simulation.hpp / simulation.cpp    engine: track borrow, clock, run
      world_physics.hpp / world_physics.cpp
      track_asset.hpp / track_asset.cpp
      referee.hpp / referee.cpp
      robot_geometry.hpp          fixture geometry + calibration mapping
      race_trial.hpp / race_trial.cpp    Robot + Simulation composition
      trial_frame.hpp             frame/trace để quan sát
      renderer.hpp / renderer.cpp
      main.cpp                    ghép session + trial + GUI
    train/                        chỉ thêm khi triển khai thử nghiệm RL
      environment.hpp             adapter operation RLtools theo đúng upstream
      train.cpp                   entry training C++
      evaluate.cpp                entry chấm checkpoint trên tập giữ riêng
    esp32/
      main.cpp
      sensors.hpp / sensors.cpp
      actuators.hpp / actuators.cpp
  test/
    test_robot.cpp
    test_sim_runtime.cpp
    test_sim_trial.cpp            pose/reset/lifecycle nhỏ, không full-race gate
    test_training.cpp             chỉ thêm khi có environment/export thực

hardware/                         giữ nguyên nguồn CAD/cơ khí hiện có
``` Không thêm `common/`, `utils/`, `interfaces/`, `controllers/`, thư mục cho từng test hay file model rỗng. Không di chuyển/xóa hardware hoặc dữ liệu nguồn trong đợt refactor này.

`robot_data.hpp` là header dùng chung có consumers thật, không phải phục hồi package `robot_contract`. Nó giữ các type khác nghĩa, không ép chúng vào một `RobotState`. `race.hpp` không phải interface của platform; header cùng folder không có nghĩa mọi caller được dùng private representation.

`train.cpp` và `evaluate.cpp` dùng chung `environment.hpp`; không thêm thư mục `training/include/src`. Tham số, seed, kết quả và checkpoint của mỗi thử nghiệm nằm trong output build/experiment được ignore, không trộn với code hoặc assets nguồn. Chỉ quản lý model được chọn và test vectors cần triển khai khi có model thật; không tạo sẵn cây artifacts.

## 4. Dependency và build

Mũi tên dưới đây nghĩa là **phụ thuộc**, không phải luồng tín hiệu:

```text
lf_robot_data                     chỉ header dữ liệu
  ↑          ↑         ↑
lf_drive   lf_simulation          không phụ thuộc lẫn nhau
  ↑          ↑
lf_robot     │                    Robot + Race; dùng lf_drive
  ↑          │
  └── lf_race_trial ───┘          ghép hai module, không Raylib
           ↑
       GUI / test_sim_trial

trainer + environment ──→ lf_drive + lf_simulation + RLtools
ESP32 session ──────────→ lf_robot + platform drivers
renderer ──────────────→ dữ liệu frame/track + Raylib
```

Tên targets chỉ để khai báo build, không tạo package/folder tương ứng:

| Target | Sources/trách nhiệm | Dependencies |
|---|---|---|
| `lf_robot_data` | INTERFACE của `robot_data.hpp` | C++17, include path; không `.cpp` |
| `lf_drive` | `drive.cpp` | data; không Robot/Race/OS |
| `lf_robot` | `robot.cpp`, `race.cpp`, `route.cpp` | Drive + data; không simulator |
| `lf_simulation` | simulation/world/referee/track sources | data; PNG/JSON ở implementation loader |
| `lf_race_trial` | `race_trial.cpp` | Robot + Simulation; không Raylib/RLtools |
| GUI executable | main + renderer | trial, frame/track, Raylib |
| Train/evaluate executables | entry + environment | Drive + Simulation + RLtools; chưa tạo mặc định |

`Race` owns `Route`: wheel/pose histories 64 entry, pending acquisition 32, hai curve 1024 điểm và 16 vùng ToF. Line acquisition order, nominal body arc và absolute axle travel là ba tọa độ khác nhau; không biến gap thành độ dài line đã biết. No-return không xóa occupied evidence; association chưa chắc chỉ được giữ bao hợp. Map giữ qua prepare, reset session mới xóa. Overflow/correspondence không hỗ trợ được báo tường minh. Các bounds còn conditional free-rolling; recorded path chưa phải free-space certificate và controller hiện chưa dùng map cho navigation.

Engine được phép load assets khi setup; JSON/PNG không nằm trong control tick hoặc public data header. Không nhập `lf_robot` chỉ để có include directory. `test_sim_runtime` chỉ link engine, không link Robot/Drive/trial; test thật bằng motor output cố định thay vì stub controller.

`firmware/platformio.ini` giữ source filters explicit cho GUI, evaluator và từng test; engine tests không compile controller. Theo [ADR-015](decisions/ADR-015-build-and-hardware-verification.md), pkg-config được khai báo trực tiếp trong PIO; sanitizer thêm flags và dùng build directory riêng, không hook Python tùy biến. Các recipe `just` gọi PlatformIO CLI, gồm compile database và ASan/UBSan; root CMake đã bỏ. Không mỗi module một `CMakeLists.txt`/`library.json`, không thêm frontend native khác. Chưa có recipe training thì không mô tả nó là lệnh dùng được. CMake nội bộ ESP-IDF do PIO quản lý, không là frontend thứ hai.

Project dùng C++17 và `-Wall -Wextra -Wpedantic`, không nâng lên C++20/23 theo skill. Warning policy/sanitizer đặt theo target; dependency bên thứ ba không chịu format/naming của project. Không để RLtools full target lan `-ffast-math`/`-march=native` sang safety hoặc firmware. Pin dependency lúc thực hiện thử nghiệm, không thêm dependency ngay trong đợt thiết kế.

## 5. Contract Simulator: dữ liệu vào/ra, không callback

Các operation sau mô tả interface cần triển khai, không phải declaration đã tồn tại:

| Operation | Contract |
|---|---|
| Bind track ở setup | Caller load/validate bằng `TrackAsset::load`, giữ asset bất biến rồi cho Simulation borrow const. Lỗi load trả rõ trước khi thay world; đổi track kết thúc trial/Robot session cũ. |
| `prepare_run` | Chỉ giữa các lượt: đặt pose/velocity/motor/cảm biến vật lý về điều kiện đầu, reset referee, chuẩn bị packet mới. Chưa tính là một lượt đã chạy; không gọi Robot hoặc xóa memory của nó. |
| `start_run` | Commit lượt đã chuẩn bị sau khi caller chấp nhận start; bắt đầu thời gian/kết quả của lượt. Không tự hỏi `Robot::observe()`. |
| `advance` | Nhận `MotorOutput`, tiến đúng một logical tick, tích phân với deadline thực, cập nhật acquisition/referee rồi publish snapshot hoàn chỉnh. Idle tick dùng output disabled; không tính thời gian idle vào race. |
| `stop_run` | Kết thúc một lượt đang chạy với lý do từ công cụ như abort/timeout; không đổi verdict terminal đã chốt. Referee tự chốt kết quả theo thế giới. |
| `observe` | Đọc const snapshot đã publish; không tiến clock, không lấy mẫu lại và không gọi controller. |

`Simulation` owns world/referee/clock và chỉ borrow track/geometry bất biến. Caller setup owns assets lâu hơn mọi Simulation/episode dùng chúng; không reload/mutate asset đang được borrow. Nó không owns trace GUI, `RobotObservation`, request của controller hoặc RL network. Nó xử lý một lượt tại một thời điểm; quota hai lượt và tổ hợp kết quả nằm ở RaceTrial, không trong engine. Quy tắc thi có bao nhiêu lượt không thay đổi physics.

Snapshot engine, tên đề xuất `WorldObservation`, giữ thời gian, pose thật, packet `SensorMeasurements`, kết quả referee và identity của world/run. Reference tới snapshot chỉ dùng đến lần mutate engine tiếp theo; recorder/state history phải copy giá trị cần giữ, không giữ reference rồi đọc sau `advance`. Ground truth ở đây chỉ dành cho renderer, reward và bộ chấm; không truyền cả struct cho actor. Actor chỉ nhận projection từ đường sensor đã định nghĩa.

Collision detector trả sự kiện hình học, không tự đồng nghĩa DNF; Referee phải áp dụng luật, gồm ngoại lệ tiếp xúc theo FAQ Q-14. Luật finish/timing/đẩy vật cản chưa rõ hoặc chưa mô hình hóa phải báo là giới hạn, không suy ra race hợp lệ chỉ vì không có collision.

`advance` ở engine kiểm tra format motor, finite/range/mode và deadline như actuator thực, kể cả caller cố tình không qua Drive. Đây là validation ở hai trust seams khác nhau: engine/actuator không được tin caller; Drive bảo đảm hành vi robot/training nhất quán. Không viết một controller "an toàn hơn" bên trong physics để che action sai.

### Clock và thứ tự một tick

Với packet tại `t`, output có hiệu lực trên `[t, t + dt)`:

1. Caller đọc packet đã cache tại `t`; Drive chuẩn hóa, giữ nguyên timestamp/sequence theo nguồn.
2. Robot hoặc actor tạo demand; Drive tạo output có deadline theo clock đó. Chỉ một quyết định cho packet này.
3. Engine tích phân output đến deadline hoặc cuối tick; phần còn lại phải disabled theo actuator contract, không kéo dài hiệu lực ngầm.
4. Tại `t + dt`, engine cập nhật acquisition theo cadence, chấm referee, chốt terminal nếu có rồi publish packet/snapshot mới.
5. Caller ghép bản ghi quyết định tại `t` với outcome tại `t + dt`. Nếu terminal thì không gọi Robot/actor thêm để làm đẹp trace.

`Simulation` là chủ logical clock; GUI accumulator chỉ quyết định chạy bao nhiêu tick. Pause/scrub không mutate world/Robot; UI requests được xử lý tại tick có kiểm soát, không gọi `drive` thêm lần nữa ở cùng timestamp. Đọc lại snapshot không sinh sequence mới. Mẫu ToF giữ timestamp cũ cho đến lần acquisition tiếp theo.

`prepare_run` là discontinuity có chủ đích giữa lượt, không phải bước vật lý: clear output còn hiệu lực và reset physical state. Clock của **cùng Robot session** vẫn monotonic; scheduler phải dành packet decision mới cho request start/retry, không tái dùng timestamp đã gọi Robot. Simulation session mới có thể bắt đầu clock mới cùng một Robot session mới; không giữ Robot cũ qua việc tua ngược thời gian.

## 6. Core C++: ownership, preprocessing và an toàn

`run_robot(calibration, session)` tiếp tục owns typed private `Robot::State` theo lexical lifetime; State owns Drive + Race bằng value. Robot không copy/move, không heap-PIMPL, singleton, caller buffer hoặc `void*`. Session chỉ borrow; mọi RaceTrial tham chiếu Robot phải được dựng và hủy bên trong callback. C++17 không tự ngăn reference escape; cần review và sanitizer. Disable khi callback return không thay watchdog chống stall.

`Drive` là module core không I/O, giữ calibration bất biến và history normalization có giới hạn. Interface dự kiến gồm `read_sensors`, `drive_motors`, `prepare_run` và đọc fault. Tên nói việc thực hiện, không expose optimizer/filter/controller theory. Các method phải trả dữ liệu/fault rõ, không âm thầm chuyển lỗi calibration, clock hay motor thành kết quả thành công.

- `read_sensors`: physical measurements → normalized readings, kiểm tra monotonic time, finite/range, status/age, counter wrap/delta. Có chính sách freshness theo nguồn, không đòi mọi cảm biến cập nhật đồng thời. Những ngưỡng chưa đo là điểm chưa chốt, không default giả.
- `drive_motors`: demand + trạng thái được phép chạy → output có physical deadline; không gia hạn demand từ dữ liệu hết hiệu lực như bình thường. Đầu ra lỗi phải disabled, kèm fault; không chỉ clamp NaN/range để giấu lỗi policy.
- `prepare_run`: rebase encoder/per-run origin khi world hoặc robot được đặt lại; giữ clock history và calibration. Không xóa trí nhớ của Race.
- Robot owns lifecycle/control memory; Drive không chứa path, first-run speed profile hoặc RL replay buffer. Không để trainer cập nhật trực tiếp status của Robot để vượt safety.

Environment training là **module native khác**, được own Drive riêng và state episode của nó; nó không dựng, borrow private representation hoặc cấp storage cho `Robot::State`/Race. GUI/ESP32 vẫn chỉ đi qua Robot. Như vậy trainer dùng được preprocessing/output semantics mà không phải gọi controller baseline hoặc được mở quyền can thiệp Robot.

Giữ nghĩa các type trong `robot_data.hpp`:

| Type | Nghĩa |
|---|---|
| `SensorSample<T>` | Physical value, acquisition timestamp/sequence/status; thay tên `Acquired<T>` |
| `SensorMeasurements` | Packet physical ở clock monotonic, giữ riêng từng kênh |
| `SensorReadings` | Số đo normalized, age/elapsed time theo `T0`; không phải actor history hoặc mục tiêu |
| `Calibration` | Nominal calibration được robot biết; `L0=wheelbase`, `V0=reference_speed`, `T0=L0/V0`, giữ nhất quán hai lượt |
| `MotorDemand` | Yêu cầu motor với duration normalized |
| `MotorOutput` | Duty/mode và physical absolute deadline |
| `RobotObservation` | Telemetry có producer thật từ robot, không chứa pose/map thật của sim |

Xóa `GeometryObservation` placeholder cùng assertion availability hiện có nếu vẫn không có production producer/consumer; giữ các test có nghĩa khác. Không ép physical/normalized, duration/deadline hoặc sample time/decision time thành một type vì muốn ít header.

Không block/cấp phát động trong tick robot hoặc physics. Track allocation/PNG/JSON load, trace reserve và trainer setup diễn ra ngoài tick. Trace có capacity/overflow rõ; trainer không tự lưu vector dài ở mỗi environment. Weights triển khai read-only; activation/recurrent state có owner riêng, không mutable global. Training optimizer/replay có thể cấp phát trên host, không kéo chúng vào firmware.

## 7. Hai lượt, kết quả và quan sát

`RaceTrial` borrows Simulation + Robot, owns trạng thái orchestration và hai kết quả. Nó không owns Robot storage, không nhận diện loại controller và không chứa luật va chạm/finish thứ hai.

1. Chuẩn bị world về start và thông báo chuẩn bị lượt cho Robot để rebase, không full reset trí nhớ.
2. Trên packet mới, gửi start tới Robot; chỉ khi Robot chấp nhận mới commit start engine và tăng số lượt đã dùng. Start bị từ chối phải là rejected/not-run, không DNF giả; có regression cho failure của bước commit.
3. Lặp một đường tick chung cho GUI/headless. Khi referee hoặc safety chốt dừng, không áp dụng thêm demand; thông báo kết thúc lượt cho core mà không đưa pose/collision map vào quyết định chuyển động.
4. Lượt sau dùng cùng start conditions/calibration và history được phép giữ; counter origin rebase, clock không quay lại. Full session reset mới bỏ history toàn bộ.

Các lệnh prepare/end-run là semantics cần bổ sung rõ vào request contract khi triển khai; không giả chúng đã có trong enum hiện tại. Tín hiệu start/stop của công cụ tương đương điều hành cuộc chạy, không được biến nhãn Completed/collision truth thành input chọn chuyển động chỉ có trong sim.

DNF do referee/timeout không đồng nghĩa lỗi calibration hay lỗi nội bộ controller. Proposed policy: chỉ giữ prefix measurements thực sự đã ghi, có coverage/capacity rõ; không coi memory sau DNF là một vòng hoàn chỉnh. Safety fault không tự được gỡ bởi Start. Chính sách phục hồi fault/giữ prefix cần chốt và test trước slice lifecycle, không tự gán nghĩa cho `kReset` cũ. Hai lượt khác với các pass xử lý; vẫn giữ các biến đổi có thứ tự/contract cần thiết, không tạo pass-manager.

`RaceTrial` aggregates Completed/DNF/Pending/rejected/not-run riêng, best time chỉ từ lượt hợp lệ; gain chỉ khi hai lượt hợp lệ và comparable, có thể âm hoặc 0. Engine không biết baseline hay neural. Reward không được dùng thay result schema hoặc safety.

`SimFrame` là bản ghi quan sát do trial/observer tạo, không là acquisition input. Nó giữ identity, decision time, outcome time, physical measurements/output, telemetry robot và referee result với đơn vị rõ; không gắn pose sau tick vào nhãn thời gian trước tick. GUI dùng ToF physical cho mm/ray length, không coi normalized ToF là mm. Unavailable telemetry hiển thị N/A.

Trace recording là tùy chọn ở caller vì GUI/playback cần còn headless training không cần. Một chỗ tạo frame đảm bảo verdict đã final trước khi record; terminal không sửa riêng current mà bỏ quên trace. Renderer giữ docks/camera/chassis/ghost/trails/palette/IR/heatmap, chỉ thay binding. Không yêu cầu người dùng mở GUI để thay kiểm thử tự động.

## 8. RLtools ở đâu và không ở đâu

Training C++ có vòng: sensor → Drive → actor → Drive → Simulation → reward/update. Chỉ `environment.hpp` cần tuân theo tên/type operation template của upstream; không đổi signature của thư viện để ép NAMING hoặc cho tên thuật toán lan ra domain interface.

RLtools có contract `step(state, action, next_state)`: trạng thái tiến hóa phải nằm trong State tương ứng, không giấu một Simulation mutable dùng chung trong environment rồi bỏ qua `state`. State owns bản Simulation + Drive/history của episode; parameters giữ asset/nominal setup bất biến. Copy state chỉ copy phần world/referee/clock/sensor caches/history có giới hạn, không copy raster track, trace, weights hay replay buffer. Simulation/Drive cần value semantics độc lập thích hợp cho native state; Robot facade vẫn noncopyable. Cache/RNG/cadence cần để tái hiện phải có owner theo episode hoặc được truyền theo contract upstream, không singleton. Regression: bước từ cùng state/action/seed có cùng outcome, input state không bị sửa và hai environment không ảnh hưởng nhau.

Actor nhận các kênh sensor hợp lệ, age/elapsed time và history/context được định nghĩa. Không nhận `WorldObservation`, tọa độ thật, centerline, vị trí gạch hoặc khoảng cách thật tới đích. Privileged critic nếu có phải tách rõ. Physics có thể tạo acquisition từ ground truth; reward/referee có thể dùng truth mà deployed actor không có.

Nominal calibration robot được biết và tham số plant thật là hai thứ khác nhau. `robot_geometry.hpp` giữ fixture và mapping `simulated_calibration()` dùng chung, không copy ở GUI/test. Khi randomize plant, không tự cấp chính tham số sai lệch thực cho actor hoặc thay nominal calibration để triệt uncertainty. Ranges noise/latency/grip/motor phải có nguồn; seed tái hiện được, không biến fixture thành sa bàn bí mật.

Chưa chốt thuật toán, observation vector, hidden size, recurrent state hoặc quantization. Hai output liên tục là phạm vi thử nghiệm đề xuất, không cam kết đủ cho chiến lược hai lượt. Enable/freshness checks và bound của command duration/deadline không phải action để actor học cách vượt safety; adapter dùng cùng quy tắc như Robot. Memory xuyên lượt cần episode/reset/training tương ứng, không tự có vì weights cố định. Chỉ thêm model/inference file khi có implementation dùng được; Race vẫn là chỗ quyết định chuyển động, không tạo controller chạy song song ở platform.

Khi có learned policy dùng thật, thêm `motion.hpp/.cpp` trong core nếu cần để Race và trainer dùng chung việc pack observation, action scaling, model configuration/inference và reset state; không tạo file placeholder ở đợt thiết kế. RLtools vẫn owns optimizer/gradient trên host; core motion không chứa training loop. Weights có thể thay đổi ở trainer nhưng được chốt/read-only khi triển khai, buffers/history thuộc từng rollout hoặc Race, không share mutable state giữa episodes. Không viết một mạng/feature packing thứ hai trong `race.cpp` chỉ vì đó là firmware.

Trong thử nghiệm giữ baseline để so sánh trên cùng sim đã sửa. Chọn implementation khi ghép/build executable hoặc trong harness thử nghiệm có phạm vi nhỏ, không yêu cầu firmware ship cả hai controller. Trước khi tích hợp model phải chứng minh chung preprocessing, action scaling, reset/history và output validation; không suy parity chỉ từ việc cả hai dùng RLtools. Nếu chọn learned controller, xóa logic baseline bị thay thế sau acceptance; nếu không đạt, bỏ experiment glue đã chết, không giữ framework dự phòng.

Khả năng C++/export của RLtools đã được nghiên cứu trong [assessment](research/rl-tools-assessment.md), chưa có kết quả LF-Bot hoặc ESP32-S3. Chưa dùng backend DSP chỉ vì enum có S3. Host golden vectors, hardware inference latency/jitter/RAM/stack và safety/watchdog là acceptance riêng; native 2 ms tick không phải WCET proof.

## 9. Tên và các phần phải cắt

Theo [NAMING](NAMING.md): hàm là động từ việc làm; type là danh từ miền CamelCase; không đưa PD/pure-pursuit/TD3/GRU ra tên public domain. Tên thuật toán vẫn được dùng đúng trong implementation, dependency và config thử nghiệm; không đặt alias che giấu thư viện chỉ để đổi chữ.

| Hiện tại | Đích/hành động |
|---|---|
| Ba package `robot`/`race`/`robot_contract` | Một `lib/robot/` phẳng; data header riêng, Race private |
| `include/<module>/src`, module manifests/CMake | Bỏ sau cập nhật callers/build; PIO khai báo source filters |
| `sim/runtime/` | Làm phẳng vào `sim/`, target tách sources engine/trial/GUI |
| `Simulation::advance(Robot&, RaceRequest)` | Engine `advance(MotorOutput)`; Robot orchestration sang RaceTrial |
| `Simulation` giữ trace/RobotObservation | Snapshot engine độc lập; frame/trace ở caller |
| `TrackRules`, `track_rules.*` | `Referee`, `referee.*` |
| `Acquired<T>` | `SensorSample<T>` |
| `ControlModel` | Bỏ fields không đọc; geometry còn cần thì `RaceGeometry`, không generic model |
| `planned_speed`, `observed_offset` | `choose_speed`, `measure_line_offset` nếu logic còn giữ |
| `apply_memory` | Tách nghĩa ghi/dùng memory khi sửa implementation; không rename code sắp xóa hai lần |
| `kHalfLengthMm` dùng làm ToF offset | Tên phản ánh offset ToF, không đổi số/hình học |
| Geometry placeholder, getter/wrapper/config lặp | Cắt theo consumer audit trong PLAN; không cắt validation thật |

Giữ `Simulation`, `WorldPhysics`, `Renderer`, `Race`, `Robot`: không rename toàn bộ vì có thiết kế mới. Không thêm một naming document khác. Không khôi phục custom architecture checker, negative fixtures, stack report plumbing hoặc OS migration.

## 10. Chuyển đổi và tiêu chí kiểm chứng

Mỗi slice tách mechanical moves khỏi behavior changes; mọi cut đi kèm cập nhật caller/build/docs. Không đổi constants/assertions để hợp thức hóa refactor.

Đã làm ở slice 2026-10-19 (1–4 bên dưới); 5–6 còn mở:

1. **Data/core phẳng và Drive dùng chung.** `firmware/lib/robot/` phẳng; ba package cũ, `library.json` và `include/src` đã xóa. `Drive` giữ conversion/clock/counter/output; Robot test vẫn đi qua `Robot::drive`.
2. **Engine theo motor output.** `Simulation::advance(MotorOutput)`; orchestration ở `RaceTrial`; GUI và hai test đã chuyển. `test_sim_runtime` chỉ link `lf_simulation`.
3. **Regression correctness/lifecycle.** Collision đủ bốn trục, gate overlap, freshness theo nguồn, ToF unknown không phải clear, đơn vị vật lý trong frame, verdict chốt trước khi record, reposition/rebase hai lượt, summary hai lượt, abort/reset/third-start.
4. **Một nguồn build.** PlatformIO CLI dựng GUI, test, sanitizer và compile database theo ADR-013; root CMake đã bỏ. Registration CMake nội bộ ESP-IDF được giữ.
5. **Environment và phép so sánh RL nhỏ.** Chưa có: cần environment RLtools dùng chung engine/Drive, kiểm tra action/observation không rò truth, nhiều seed và held-out tracks. Không bắt buộc hoàn thiện controller hình học trước thử nghiệm.
6. **Acceptance riêng.** Còn mở: GUI offscreen/input/playback, CI full-race khi ổn định, ESP32 adapter/HIL và measured timing. Chưa có evidence thì giữ deferred/blocked, không tick pass vì build xanh.

Các kiểm chứng quan trọng của interface mới: đọc snapshot nhiều lần không đổi world; mỗi tick đúng một quyết định; output hết hạn giữa tick; prepare không xóa memory; rejected start không tiêu lượt; abort/terminal không phát thêm action; trace verdict/time/units nhất quán; clock monotonic qua lượt hai; memory overflow và thiếu sensor có kết quả rõ.

Dùng recipe `just` thực có trong [justfile](../justfile), theo [AGENTS](../AGENTS.md). Code đổi phải test/format/lint/build phù hợp; physics/control/lifecycle đổi phải chạy thêm runtime và regression nhỏ cho hành vi bị ảnh hưởng. Gate `sim-test` đã bỏ theo yêu cầu; kết quả full-race cũ chỉ là lịch sử, không suy race pass từ tests xanh. Đợt chỉ thiết kế kiểm tra links/diff/recipe references, không lấy test lịch sử làm bằng chứng kiến trúc mới đã pass.
