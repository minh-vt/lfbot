# ADR-014: MAIN tích hợp A4950 — envelope và quy trình prototype

> **Superseded by [ADR-019](ADR-019-module-based-hardware.md)** cho phần cứng hiện hành: dừng PCB riêng, chuyển sang module. Các status và quyết định bên dưới là lịch sử, không phải quyền tiếp tục chế tạo.

**Status:** Phần gate ngspice được thay bởi [ADR-015](ADR-015-build-and-hardware-verification.md), 2026-11-01; giữ các lựa chọn envelope/process. Accepted cho thiết kế prototype theo yêu cầu chốt quyết định của chủ dự án, 2026-10-31. Không phải chấp thuận gia công, xác nhận fit hay kết quả đo. R11 vẫn cần duyệt release riêng. Quyết định này thay các lựa chọn còn OPEN trong R02 của spec/tickets; không thay topology hoặc mở lại phạm vi bảo vệ đầu vào đã dừng.

**Cập nhật phạm vi:** [ADR-016](ADR-016-main-fixed-placement-routing.md) thay workflow R01–R12/mở lại placement bằng routing trên CAD baseline đã place. Các số ticket bên dưới là lịch sử. Giữ envelope điện; đổi footprint/paste không tự nằm trong phạm vi routing mới. Footprint baseline đã có bốn cửa paste riêng, không phải EP thiếu paste.

## Bối cảnh

MAIN đã có hai IC A4950ELJTR-T, nhưng chưa có copper/thermal via/paste EP hoàn chỉnh. Giữ R02 chờ dòng motor và nhiệt đo trên một PCB chưa sản xuất là vòng phụ thuộc sai. Cần chọn envelope thiết kế có giới hạn, kiểm tra mô hình trước routing rồi kiểm chứng phần cứng sau. Datasheet GA12-N20 generic không xác nhận đúng lô motor 1000 RPM; rating ±3,5 A của A4950 cũng không phải khả năng liên tục của MAIN.

## Quyết định và lý do

### 1. Giữ IC rời, prototype hai lớp 2 oz

- Giữ hai A4950, C3/C4 220 nF/50 V, C7 220 µF/35 V, chia áp 180 kΩ/22 kΩ/100 nF và DNP hiện hành. Không quay lại module và không thêm TVS/fuse/FET trong slice này.
- Chọn **JLCPCB bare PCB, FR4 hai lớp, 1,6 mm, copper thành phẩm mục tiêu 70 µm mỗi mặt (2 oz), ENIG**, stencil rời và reflow cục bộ có kiểm soát; THT hàn sau. Không chọn turnkey assembly hoặc via-in-pad filled/capped vì tăng chi phí/quy trình không cần cho mẫu đầu.
- Báo giá/order phải xác nhận copper, PTH plating tối thiểu 20 µm và quy trình này trước R11. Đây là yêu cầu mua hàng, chưa phải xác nhận từ fab. Nếu không nhận được, quay tính R03; không âm thầm coi copper nominal là bảo đảm tối thiểu.
- Outline 112 × 70 mm hiện tại là ứng viên prototype giữ để kiểm tra R04, không phải kích thước lắp thực đã duyệt. Không thu nhỏ để lấy vẻ gọn khi chưa chứng minh corridor và return.

2 oz giúp giảm điện trở và lan nhiệt, phù hợp họ ví dụ thermal của Allegro. Nó không làm MAIN tự có RθJA = 62°C/W. Hai lớp đủ cho envelope hạn chế bên dưới; chưa có lý do đổi bốn lớp.

### 2. Giới hạn thiết kế khác với thông số motor đã đo

| Đại lượng | Chọn cho prototype đầu | Lý do / cách xác minh |
|---|---|---|
| Dòng motor | ≤0,50 A RMS mỗi kênh; peak quan sát ≤1,20 A, mỗi đợt khởi động ≤100 ms | Giới hạn bench do thiết kế chọn, không khẳng định motor đáp ứng. Kẹt/đảo chiều tải không nằm trong envelope; dừng nếu không khởi động trong 100 ms. |
| Tải chung | Copper bus/return tính 3 A; hai peak motor 2,4 A + nhánh buck vào ≤0,3 A = 2,7 A | Không đồng nhất dòng motor với dòng pin. 3 A là khả năng thiết kế conductor, không phải giới hạn nguồn khi thử lần đầu. |
| Nguồn | 3S đã bảo vệ ngoài MAIN, tối đa 12,6 V; lần thử tải đầu dùng nguồn 10,5–12,6 V, VM tại IC ≥9,0 V | Giữ margin với VBB min 8 V; không khai thác pack gần cạn trước khi đo sụt áp. Cần nguồn/hệ thống có đường hấp thụ năng lượng khi thử hồi dòng. |
| Rail logic | 5 V: 4,75–5,25 V, ≤0,50 A tổng; 3,3 V: 3,135–3,465 V, tải ngoài DevKit ≤0,10 A | Là tiêu chí bench cho buck/DevKit clone, không lấy rating MP1584 IC làm rating module. Buck input ≤0,3 A phải đo đồng thời. Nếu không đạt, không mở tải motor. |
| Ripple/transient | VM: 9–16 V trong thử được phép, ripple plateau ≤0,5 Vpp; 5 V ≤100 mVpp; 3,3 V ≤50 mVpp | Đo tại tụ với ground spring/bandwidth 20 MHz; spike nhanh kiểm tra riêng bằng probe phù hợp, không dùng bandwidth limit để che spike. |
| Stress bus | 25 V chỉ là trần sàng lọc mô hình/thiết kế, **không phải mức được phép thử** | C7 35 V và ADC có margin danh định nhưng module U2 chưa xác nhận transient. Bench dừng nếu VM >16 V. |
| Nhiệt | Ambient 10–40°C; mục tiêu Tj ≤125°C, không dùng shutdown 150°C làm điều kiện pass | Bench dừng ở case driver 80°C, thân shunt 90°C hoặc PCB quanh EP 70°C; case không chứng minh Tj. Thử 30 s →2 phút →10 phút; không tuyên bố liên tục lâu hơn khi chưa đạt cân bằng nhiệt. |
| PWM/reset | Mục tiêu 20 kHz; bắt đầu burst duty thấp, hai input LOW khi reset; tắt lệnh và ngắt nguồn an toàn nếu quá giới hạn | Không sửa firmware ở đây. Phải xác nhận truth table, coast/brake/recirculation thực bằng waveform trước khi tăng tải. |
| Phanh/đảo chiều | Cấm plug-reverse và phanh tái sinh có tải trong bring-up đầu; cho rotor dừng trước đổi chiều | C7 chỉ lưu năng lượng hữu hạn, không thay thế nguồn hấp thụ năng lượng; chưa có waveform để dự báo peak VM thật. Coast cũng có thể hồi dòng qua diode; không xem lệnh LOW là bảo đảm không hồi dòng. |
| ADC/I²C | ADC pin đọc 10 Hz sau ≥20 ms ổn định, hiệu chuẩn gain/offset; I²C 100 kHz, bus ≤200 pF, pull-up tương đương 2,2–4,7 kΩ về 3,3 V | RC pin không dùng bắt spike hay làm bảo vệ nhanh. Pull-up tính cả adapter; R8/R9 vẫn DNP. Encoder R12–R15 vẫn DNP, không có chức năng được xác nhận. |

Bench cấp nguồn theo bậc: chưa DevKit/motor, current limit VM0,05 A; thêm logic tối đa0,30 A; thử một motor0,80 A input; hai motor tối đa1,50 A input ban đầu. Các mức nguồn này không tương đương giới hạn dòng winding; nếu limit tác động/reset thì dừng và tìm nguyên nhân, không tự tăng đến3 A sizing. Trước **bất kỳ** thử motor nào phải xác nhận đường nhận năng lượng hồi (nguồn sink phù hợp hoặc pack/BMS có điều kiện nhận sạc/điện áp được xác nhận); nếu chưa có thì chỉ thử rail, không motor. Nhãn “pack protected” không tự chứng minh điều đó, đặc biệt khi pack đã đầy. Những bậc thử trên là yêu cầu test plan R10, chưa là firmware cutoff được triển khai.

Current trip danh định vẫn là 1 A: `3,3/(10 × 0,33)`. Datasheet không có hàng bảo đảm gain tại đúng VREF 3,3 V; không nội suy thành guaranteed tolerance. Vì thế 1,20 A là **ngưỡng dừng đo**, không phải hứa hẹn current-limit IC luôn giữ được nó. Peak ngắn/blanking và LSS overshoot phải đo; LSS dừng khi |V| ≥0,45 V, thấp hơn absolute ±0,50 V. Không dùng bảo vệ phần mềm thay cho nguồn giới hạn dòng ngoài.

### 3. Kích thước copper/via được chọn, không để lại câu hỏi cho router

Chọn clearance chung 0,25 mm; copper–edge ≥0,50 mm; signal mặc định 0,25 mm; không dùng microvia/blind/buried. Ngoại lệ pad escape phải có rule vùng và số đo, không hạ minima toàn board.

| Path | Width liên tục / giới hạn chiều dài một conductor | Pad escape | Chuyển lớp |
|---|---|---|---|
| J1–VM bus–C7, GND hồi chung | ≥2,0 mm, ≤60 mm, 3 A sizing | Chỉ theo land bắt buộc, ≤1,5 mm | ≥3 via song song |
| Mỗi VBB/OUT/LSS/shunt return | ≥1,0 mm, ≤50 mm, 1,2 A sizing | ≥0,30 mm, ≤1,5 mm tại chân SOIC | ≥2 via song song |
| 5 V/JP1/U1 | ≥0,75 mm, ≤60 mm, 0,5 A | ≥0,30 mm, ≤1,5 mm | ≥2 via |
| 3,3 V phân phối | ≥0,50 mm, ≤60 mm, 0,1 A tải ngoài + VREF | ≥0,25 mm, ≤1,5 mm | ≥1 via |
| Logic/ADC/I²C | 0,25 mm | Không giảm để né lỗi placement | Return via GND cách chỗ đổi lớp ≤2 mm nếu không có pad GND gần hơn |

Via chọn thống nhất **land 0,90 mm / finished drill 0,30 mm**. Annular ring danh định 0,30 mm ưu tiên độ bền, không chạy sát giới hạn fab. Power via chia dòng cần đầu vào/ra cân đối; không để cả cụm ba via bị cấp qua một neck mảnh. THT pad/courtyard vẫn theo đúng linh kiện, không áp quy tắc via vào drill chân C7/socket.

Với Cu 70 µm, điện trở suất sàng lọc tại khoảng 80°C `ρ=2,13×10⁻⁸ Ωm`, `R=ρL/(wt)`:

- Bus 2 mm ×60 mm: 9,13 mΩ/conductor; ở 3 A mất 27,4 mV, 82,2 mW. Nếu đi/về cùng hình học, nhân đôi drop/loss.
- Motor 1 mm ×50 mm: 15,21 mΩ; ở 1,2 A mất 18,3 mV, 21,9 mW/conductor. Escape 0,30 mm ×1,5 mm: 1,52 mΩ, không được kéo dài nó thành đường chính.
- Via 1,6 mm, lỗ 0,30 mm, plating 20 µm: `R≈ρL/(πdt)=1,81 mΩ`; ba via lý tưởng 0,60 mΩ, mất 1,81 mV ở 3 A. Đây là ước lượng DC/plating, không là rating nhiệt hoặc chia dòng bảo đảm.
- Sàng lọc công thức IPC-2221 ngoài lớp (`I=0,048×ΔT^0,44×A^0,725`, A tính mil²): motor khoảng 0,67°C, bus khoảng 1,70°C tăng nhiệt. Dùng như kiểm tra sơ bộ, không là IPC-2152/thermal simulation của MAIN; cổ chai/pad/nguồn nhiệt kề và dòng hồi thực phải audit R08/R09.

Các width này có margin điện trở và lớn hơn fab minima 2 oz 0,16/0,16 mm. Nếu layout vượt chiều dài, mất tiết diện sau fill hoặc không đạt ngân sách nhiệt: tính lại hoặc sửa placement, không tự nối airwire bằng đường 0,2 mm. GND là một net nhưng rule vùng công suất phải giữ các tiết diện này, không gán mọi GND logic width 2 mm.

### 4. EP: reflow với paste xác định, thermal via ngoài land

- EP vẫn GND, LSS tách riêng đến shunt; shunt return độc lập ngắn đến star gần EP. Không split GND plane. EP nối **solid** vào copper, không thermal relief.
- Tạo bản footprint project-local ở R05 (giữ nguyên pin/EP land): thêm **bốn cửa paste 0,90 ×1,30 mm**, tổng 4,68 mm² = **58,85%** land 2,41 ×3,30 mm; đặt đối xứng hai hàng/hai cột, tâm x=±0,55 mm, y=±0,80 mm theo trục local EP. Stencil chọn **0,12 mm**, yêu cầu follow F.Paste, không mở lại 100% EP. Đây là lựa chọn kỹ thuật mẫu đầu, không phải pattern Allegro quy định.
- Sáu thermal via 0,90/0,30 mm mỗi IC: hai hàng ba via tại local EP x=−0,95/0/+0,95 mm, y=±2,40 mm. Tâm/land nằm ngoài EP/paste; solid copper nối từ hai đầu EP ra các hàng via, không neck <1 mm tổng mỗi đầu. Kiểm tra va pin, mask và khả năng hàn thực ở R04/R05; không chuyển via vào land để cứu placement.
- Via ngoài land được tent solder mask hai mặt nếu fab nhận; **không có via hở trong EP**, không yêu cầu resin fill/cap. Tenting không phải filled via. Nếu mask fab không bảo đảm, giữ via xa cửa paste và xác nhận stencil/reflow trước R11.
- Mục tiêu copper GND nối thực ≥500 mm² **mỗi mặt/mỗi driver**, bridge rộng, B.Cu liên tục; hai vùng nhiệt không được tính trùng để tuyên bố gấp đôi khả năng tản. Không count island, keepout hay đồng bên kia cổ chai. Đây là mục tiêu bố trí, không suy ra RθJA từ diện tích.
- Sáu via ngoài EP không tương đương thermal-via-in-pad. Lý do chọn là tránh hút thiếc/chi phí VIP ở mẫu đầu với tải thấp. Nếu R09 thấy đường nhiệt không đủ, giảm envelope hoặc trình thay quy trình; không tuyên bố số lượng sáu là bằng chứng đạt nhiệt.
- Reflow trước socket/THT; kiểm tra EP bằng X-ray khi khả thi. Nếu không có X-ray, ghi độ bất định và chỉ cho phép bench tăng tải có đo nhiệt; mối EP không được chấp nhận chỉ vì thông mạch GND. Không dùng hàn mỏ từ mép làm quy trình mặc định.

### 5. Chốt shunt và budget nhiệt, không mượn RθJA của board hãng

Chọn mục tiêu mua **Vishay WSL2512R3300FEA**, 0,33 Ω/1%/2512. Đây là quyết định MPN cho R05/R09 đồng bộ schematic/BOM sau; slice này chưa sửa CAD/BOM, không tuyên bố parity của MPN mới. Kiểm tra footprint land và hàng mua thật trước release.

WSL2512 ghi 1 W tại 70°C nhưng giá trị >0,1 Ω phải derate tuyến tính đến 80% ở 0,5 Ω: tại 0,33 Ω chỉ **0,885 W ở 70°C**, không gọi là 1 W đầy đủ. Ở **ambient** 90°C, đường derating về zero ở ambient170°C còn khoảng 0,708 W; không lấy nhiệt thân shunt làm trục ambient. Công suất ở 0,5 A RMS là 0,0825 W; ngay cả upper-bound giả định 1,2 A RMS cũng 0,475 W (nhưng chế độ này không được phép liên tục). Cần tính waveform shunt thực và pulse rating khi mở envelope.

Mỗi A4950: dùng RDS tổng max nóng **1,5 Ω** để screening, `Pcond=0,5²×1,5=0,375 W`. Cộng **0,25 W dự trữ kỹ thuật** cho switching/recirculation/IC, tổng 0,625 W/driver; chưa là bound bảo đảm do thiếu model switching. Hai IC + hai shunt ≈1,415 W, chưa gồm buck/DevKit. Ở ambient 40°C, Tj mục tiêu 125°C và reserve bất định 10°C đòi hỏi effective RθJA ≤120°C/W cho mỗi IC. Mô hình lumped 62/90/120/150°C/W cho Tj lần lượt 78,8/96,3/115/133,8°C trước reserve; trường hợp 150 không đạt. MAIN chưa có RθJA đo/tính 3D, do đó **không kết luận đạt nhiệt**. R09 kiểm tra copper/airflow; R12 kiểm chứng cùng hai kênh, đúng Z-stack. Nếu nhiệt còn tăng ở 10 phút hoặc không suy luận được junction với margin, chỉ ghi qualified đến điều kiện/thời gian đã đo, không ghi continuous.

### 6. Cơ khí không được lẫn với cổng điện

R02 đóng envelope điện/fab/method. R03 có thể tính/cấu hình rules mà không chờ đo motor. Dữ liệu **fit thật** giữ thành gate R04 trước khóa placement và R11 trước gia công: clone DevKit/socket/pin pitch/antenna edge, USB/cáp, gá/lỗ và Z-stack. Không tự coi bao 85 ×26 mm hoặc height socket 8,5 mm là số đo module thật. Đặt mục tiêu antenna clearance 15 mm theo Espressif, nhưng vị trí vùng cấm phải theo anten thực. Không vẽ vùng cấm từ ảnh ước lượng rồi đánh dấu verified. Nếu người dùng muốn release chỉ kiểm tra điện, phải duyệt riêng ngoại lệ cơ khí; tài liệu này không cấp ngoại lệ đó.

## Vai trò ngspice và hệ quả

Quyết định ban đầu yêu cầu `just test-main-spice` ở R02/R03/R08/R09. [ADR-015](ADR-015-build-and-hardware-verification.md) đã bỏ gate này cùng deck/output phụ trợ: model viết tay không tự khớp CAD và chưa có căn cứ đủ cho tải/nguồn. Các kết quả số trước đây chỉ đúng với giả định đã nhập, không chứng minh MAIN. Phép tính ADC/I²C chuyển vào spec; nguồn/hồi dòng/nhiệt phải đo theo test plan. Không tạo model A4950 giả từ nguồn lý tưởng.

Quyết định được đóng không có nghĩa mọi test đều PASS: counterexample VM hồi dòng và I²C quá tải bus cố ý thất bại envelope. Phải giới hạn sử dụng theo kết quả đó. Không yêu cầu thử quá áp nguy hiểm để tái hiện mô hình.

## Nguồn

- [Allegro A4950-DS Rev.7](https://www.allegromicro.com/-/media/files/datasheets/a4950-datasheet.pdf), tr.2–3 absolute/thermal/RDS, tr.5 trip, tr.7–8 layout/EP. [Trang sản phẩm](https://www.allegromicro.com/en/products/motor-drivers/multichannel-and-solenoid-drivers/high-side-and-low-side-drivers/a4950): không thấy model SPICE công khai trong lần tra 2026-10-31.
- [JLCPCB copper weight/rules](https://jlcpcb.com/help/article/jlcpcb-copper-weight), [capabilities](https://jlcpcb.com/capabilities/pcb-capabilities), [stencil order](https://jlcpcb.com/help/article/instructions-for-stencil-order): hỗ trợ lựa chọn, không thay xác nhận báo giá/order.
- [Vishay WSL datasheet 30100](https://www.vishay.com/docs/30100/wsl.pdf): part code, rating, value/temperature derating.
- [DigiKey trace-width calculator](https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-pcb-trace-width): công thức IPC-2221, giới hạn của screening thực nghiệm.
- [NXP UM10204](https://www.nxp.com/docs/en/user-guide/UM10204.pdf): Standard-mode 100 kHz, rise time 30–70% tối đa 1 µs. [ngspice manual](https://ngspice.sourceforge.io/docs/ngspice-html-manual/manual.xhtml): `.control`, AC/transient, `.meas`.
- [Espressif module/baseboard](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/pcb-layout-design.html#general-principles-of-pcb-layout-for-modules-positioning-a-module-on-a-base-board): clearance anten và thử với sản phẩm cuối.
