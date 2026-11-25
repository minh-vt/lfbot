# Hardware design of competitive line-follower robots

Scope: hardware choices of fast line followers (RoboTrace / Japan, Polish/European LF and LF Turbo, RobotChallenge, Pololu-community builds) and line-follow-plus-obstacle robots. "Documented" = stated in the cited source. "Inference" = my reasoning, not sourced. Collected 2026-11-16 via web search/fetch; about 20 tool calls, so coverage is broad but shallow. Several high-value pages could not be fetched (noted under Gaps).

Local rule context (from repo, not web): F-Maze-Solving robots are max 16 x 16 cm, max 12 V nominal (3S allowed), line 2 cm wide, black on white, random obstacles 10.5 x 6 cm (lying or standing), 2 runs per round, and suction fans that press on or damage the course are banned; only rolling wheels may touch the floor — `docs/manual.md` Dieu 3.2, 3.3, 3.8, Dieu 4.2, 4.4; `docs/fqa.csv` Q-01, Q-02.

## IR sensor arrays (count, spacing, preview, height, analog/digital, ambient rejection, ADC rate)

### Takeaway
Top robots use 14-16 analog reflective sensors read by MCU ADCs at kHz rates (one documented case above 10 kHz), with mechanical light shielding. Placing the array further ahead of the axle buys reaction time, but the extra front mass hurts cornering. No source gave exact pitch or exact axle-to-array distance for a winning robot.

### Cited Findings
- Aegis2024 (RT68, 1st place, 44th All Japan Micromouse Contest, robotrace class, 2023): 16 x SCM-014TB IR line sensors, analog (A/D converted) — [NTF tech data RT68](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html)
- HBFS Robotics line follower: 16 IR reflective sensors with variable pitch, read through two automotive analog multiplexers; STM32F303 with two 5 MSPS ADCs; sensor read rate "above 10 kHz". It also adds a 30x30 px IR camera at 250 FPS with 7 W of IR LEDs, seeing curves about 10 cm ahead — [hbfsrobotics.com/linefollower](https://hbfsrobotics.com/linefollower). The vendor page's title claims "most advanced"; no competition results are shown.
- Impact (Polish, STM32F103RBT6): 14 analog KTIR0711S sensors in later version; an earlier version used 19 sensors (16 in an arc, plus front/rear). It used the MCU's built-in ADC — [elektroda search summary](https://www.elektroda.com/rtvforum/topic2325660.html) (page fetch timed out; facts from search snippets only).
- OPRobots PX-01 kit: 16-sensor module through two 8-channel multiplexers with serial-addressable emitter LEDs — [PCBWay PX-01](https://www.pcbway.com/project/shareproject/PX_01_OPRobots_Linefollower_KIT_b15659f3.html)
- Semreh (ESP32-WROOM, podiums in Portuguese and RoboChallenge events 2024): 8 x QRE1113 through an MCP3008 SPI ADC — [Hackaday.io Semreh](https://hackaday.io/project/202208-semreh-advanced-line-follower-robot)
- Pololu 3pi+ 2040: 5 RC-type reflectance sensors along the front edge; RP2040 charges the output then times the decay; emitters switchable (DNE line, GP26) — [Pololu 3pi+ 2040 User's Guide](https://www.pololu.com/docs/0J86/all)
- Suckbot (Pololu blog): 6 x QTR-1RC, mounted about 12 mm above the surface, adjustable spacing — [Pololu blog 491](https://www.pololu.com/blog/491/nathans-line-following-robot-suckbot)
- The Chariot (2nd place, LVBots): QTR-8RC cut to 6 sensors, with a shield around the array against room lighting — [Pololu blog 487](https://www.pololu.com/blog/487/brandons-line-following-robot-the-chariot)
- RoboTrace (Japan) field, per a 2017 forum observer: nearly all have encoders and gyros, and "most have moving sensors". Cartis03 used a maxon motor to slide the sensor bar sideways into turns — [Pololu forum](https://forum.pololu.com/t/fast-line-follower-build/11956) (second-hand observation, not a primary source).
- Placement trade-off (practitioner): sensors ahead of the wheels compensate for motor delay, but forward mass hurts high-speed turns; keep mass near the wheel midpoint — BrandonM, [Pololu forum](https://forum.pololu.com/t/fast-line-follower-build/11956). Another forum user: "the further ahead of the axle the better" for a 5-sensor array — [Parallax forum](https://forums.parallax.com/discussion/127266/optimal-line-following-configuration). A builder who lengthened the chassis to move sensors forward found balancing became a problem — [Pololu forum](https://forum.pololu.com/t/fast-line-follower-build/11956).
- The same Pololu forum poster says digital (RC) QTR sensors are "generally preferred" over analog, and ambient light and course material affect readings — [Pololu forum](https://forum.pololu.com/t/fast-line-follower-build/11956). This conflicts with top-tier practice, where analog arrays dominate (Aegis2024, HBFS, Impact).
- Polish LF rules warn that lighting varies and sensors "should be properly protected" against light; competitors may not shade the track — [Robotic Arena/Skaner LF rules](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf)

### Inferences
- Analog arrays let firmware interpolate line position between sensors and compute per-channel calibration. That fits the repo rule that raw per-channel values are kept and classification is derived evidence.
- Multiplexed 16-channel arrays (two 8:1 analog muxes) are the common way to fit many sensors onto few ADC pins. The ESP32-S3 has two SAR ADCs, but ADC2 conflicts with Wi-Fi; verify sampling throughput on the actual board instead of assuming it.
- Emitter on/off differential sampling (read with LEDs on, read with LEDs off, subtract) is the standard ambient-rejection technique. Switchable emitters (3pi+ DNE, PX-01 addressable LEDs) enable it, but no fetched source documented a winner doing it explicitly.
- With a 20 mm line (F-Maze) versus 15-19 mm in RobotChallenge/Polish rules, a sensor pitch at or below about half the line width keeps at least two sensors on the line. That suggests roughly 8-10 mm pitch. This is inference, not documented.
- With a 16 cm maximum footprint, the sensor arm can't be long; a 16-sensor array at 8 mm pitch is about 128 mm wide and fits.

### Gaps
- No primary source gave a winning robot's exact sensor pitch, height, or axle-to-array distance. The Aegis2024 160 x 250 mm footprint implies a long preview, but the geometry isn't stated.
- No source documented emitter modulation frequency for a winner.
- No rigorous paper on look-ahead distance vs. stability for line followers was found. The closest was a JHU wall-following antenna paper ([PDF](https://limbs.lcsr.jhu.edu/wp-content/papercite-data/pdf/lamperskidynamical2005.pdf)), which wasn't read.

## Chassis geometry (mass, wheelbase, track, CoM, arm length, 2WD vs 4WD)

### Takeaway
Japanese robotrace winners are about 100-140 g with low CoM. Several are 4-wheel (2 per side, geared together) differential drive with no casters. Hobby-tier 2WD+caster robots are 200-300 g and much slower.

### Cited Findings
- Aegis2024 (robotrace winner 2023): 130 g, 160 x 250 x 40 mm (L x W x H), 4 driven wheels of 20 mm diameter x 10 mm wide, no auxiliary wheels, left/right differential steering — [NTF RT68](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html)
- Japanese robotrace blog: 2018 All-Japan winner about 120 g including downforce unit; another machine reached 100 g. A 2023 machine was 138 g with two turbofans, another 130 g — search snippets from [anikinonikki 2018](http://anikinonikki.cocolog-nifty.com/blog/2018/12/post-4f91.html), [ak-rcroom RT2023](http://ak-rcroom.com/sakuhin/robotracer/RT2023/rt2023.html) (pages not fetched; snippet-level only).
- maxon interview (2019 All-Japan Student robotrace winner): winners aim for low CoM, motors mounted side by side; lighter drivetrain and battery mean less downforce is needed, which in turn makes the downforce unit lighter — [maxon Japan report](https://maxonjapan.com/report/umemoto_2/)
- Cartis03: each side has one motor driving two wheels through a shared gear — [Pololu forum](https://forum.pololu.com/t/fast-line-follower-build/11956). Semreh also drives two wheels per side through nylon gears (11:70) — [Hackaday Semreh](https://hackaday.io/project/202208-semreh-advanced-line-follower-robot)
- The Chariot: about 210 g, 1/8" acrylic chassis, 70 mm wheels, 3/8" ball caster, 1.17 m/s average (2nd). The winner averaged 1.20 m/s — [Pololu blog 487](https://www.pololu.com/blog/487/brandons-line-following-robot-the-chariot)
- Suckbot: about 300 g with a 75 g battery, 70 mm wheels, two 1/2" ball casters, 0.93 m/s average (mid-pack) — [Pololu blog 491](https://www.pololu.com/blog/491/nathans-line-following-robot-suckbot)
- Pololu 3pi+ 2040: 97 mm diameter round chassis — [Pololu guide](https://www.pololu.com/docs/0J86/all). Hyper edition about 100 g per a third-party listing (not from Pololu) — search snippet [eckstein-shop](https://eckstein-shop.de/Pololu-3pi-2040-Robot-Hyper-Edition-151-HPCB-Motors-Assembled-EN)
- Size limits: Polish LF requires the robot to fit an A4 sheet, weight unlimited — [Skaner LF rules](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf). RobotChallenge (China edition, rev. 2025) allows 30 x 30 cm and 3 kg — [RC-LineFollower PDF](https://www.robotchallenge.org.cn/doc/en/RC-LineFollower.pdf)

### Inferences
- The F-Maze limit of 16 x 16 cm is tighter than Aegis2024's 250 mm width. The user's 110 mm outer width leaves about 50 mm across for a wider sensor bar, while the 160 mm length bounds wheel-to-array preview. Exact preview length must be measured on the robot (repo rule: no guessed dimensions).
- 4-wheel geared-pair drive removes casters (no sliding contact — relevant because F-Maze Dieu 3.8 bans non-rolling floor contact), but adds scrub in turns. Tank-style skid steering needs more torque in tight turns.
- Mass trend: winners sit near 100-140 g; every extra gram needs grip that F-Maze robots can't get from suction.

### Gaps
- No wheelbase, track width, or CoM height numbers found for any winner.
- Mass of RobotChallenge / Polish Turbo winners not found.

## Motors and drivetrain (coreless/Pololu vs BLDC, gearing, encoders, tyres, wheel diameter)

### Takeaway
Top robotrace uses maxon DCX10L coreless motors (often run near 3x rated voltage) with built-in optical encoders, or Faulhaber/BLDC with custom ESCs. The mid-tier uses Pololu HPCB micro metal gearmotors or 1020/0615 drone coreless motors, which are cheap but have high current and unit-to-unit spread.

### Cited Findings
- Aegis2024: 2 x maxon DCX10L, TB67H420 driver, 2 x maxon ENX 10 EASY optical encoders; max straight speed 5 m/s, acceleration 10 m/s^2 — [NTF RT68](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html)
- 2019 student winner: maxon DCX10L rated 4.5 V but run at about 12 V; TB6614FNG driver; planned DRV8874 for 4S (16.8 V) in 2020. Chose maxon for consistency because drone coreless motors (e.g. 0615) vary widely between units — [maxon Japan](https://maxonjapan.com/report/umemoto_2/)
- APOLLON 2025 (stated as 46th All-Japan robotrace winner): switched suction motor to BLDC; uses gyro and Hall-sensor motors — search snippet of [ak-rcroom 2025](http://ak-rcroom.com/sakuhin/robotrace/robotrace_2025/robotrace_2025/) (not fetched; TLS error).
- RoboTrace field (2017 observation): "high RPM Swiss-made coreless motors with gear reduction"; about 50% use STM32; of 100+ entrants only about 5 used 8-bit AVR — [Pololu forum](https://forum.pololu.com/t/fast-line-follower-build/11956)
- HBFS: Faulhaber 1028 BLDC (coreless winding), custom STSPIN32F0 ESC, 6 A continuous, 24 x 16 mm 4-layer, over 35,000 rpm; gearbox with Delrin spur gear; polyurethane/resin wheels — [HBFS](https://hbfsrobotics.com/linefollower)
- Semreh: started with Pololu N20 6 V 3000 rpm with Hall encoders on 3S (overvolted, gave acceleration). Moved to 1020 coreless (7.4 V); TB6612 overcurrent tripped on tight turns, so they built an RZ7886 H-bridge — [Hackaday Semreh](https://hackaday.io/project/202208-semreh-advanced-line-follower-robot)
- Pololu 3pi+ 2040 variants: 30:1 MP 720 rpm about 1.5 m/s; 15:1 HPCB 2100 rpm about 4 m/s ("hard to control; keep under 50%"); 75:1 LP about 0.4 m/s. Encoders 12 CPR at motor shaft, so wheel CPR = ratio x 12 (about 358 for 29.86:1). Wheels 32 x 7 mm silicone — [Pololu guide](https://www.pololu.com/docs/0J86/all)
- The Chariot: 35:1 15.5D gearmotors, about 575 rpm at 7.5 V, 70 mm wheels, theoretical free run about 2 m/s — [Pololu blog 487](https://www.pololu.com/blog/487/brandons-line-following-robot-the-chariot)
- Sticky tyres (Pololu 694) gave no significant difference at high speed; clean tyres matter more — BrandonM, [Pololu forum](https://forum.pololu.com/t/fast-line-follower-build/11956)
- PX-01: DC or "OPR coreless" options, 0.5 module gears, rubber wheels with 2 bearings each, RZ7886 drivers (15 V, 13 A) — [PCBWay PX-01](https://www.pcbway.com/project/shareproject/PX_01_OPRobots_Linefollower_KIT_b15659f3.html)

### Inferences
- Wheel diameter trend: winners use small wheels (20 mm on Aegis2024) for low CoM and high torque per motor size; hobby builds use 32-70 mm. The user's 43 mm wheel is in between. At a given rpm it gives about 2x Aegis2024's ground speed per rev, so it needs a higher reduction for the same acceleration.
- Encoder resolution: maxon ENX/Pololu 12 CPR x gear ratio. For odometry-based map runs (F-Maze run 2), encoder CPR per mm of travel matters: CPR / (pi x 43 mm).
- Driver current headroom is a recurring failure (Semreh TB6612 trips); coreless stall currents need drivers rated well above the continuous current.

### Gaps
- Gear ratios of the maxon-based winners not stated.
- No measured tyre compound/grip data from winners.

## Downforce (suction fans/EDF/propellers), measured gains, rules

### Takeaway
Downforce is the biggest hardware differentiator at the top. Polish rules separate "Turbo" (allowed) from classic LF (banned); robotrace historically banned suction (teams used propeller units), then later allowed it. F-Maze bans suction fans that press on or damage the course, so this lever is mostly unavailable for the user. No source gave measured cornering-speed gains.

### Cited Findings
- Polish LF rules: classic LF "must not contain parts that actively increase their downforce (in particular tunnel drives)"; those go to LF Turbo / Enhanced3D. Enhanced3D includes driving on the ceiling or a vertical loop (diameter 200 +/- 50 cm) — [Skaner LF rules](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf)
- Robotrace: 2019 winner used a drone motor + propeller downforce unit because rules banned suction — [maxon Japan](https://maxonjapan.com/report/umemoto_2/). A later blog says suction "has been allowed from this year"; a 2023 machine had 2 turbofans; Aegis2024 used a suction fan (DRW002 motor) with a vibration-damped mount to reduce gyro noise — [NTF RT68](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html); unlock year conflicts between sources (search snippets of [ak-rcroom RT2023](http://ak-rcroom.com/sakuhin/robotracer/RT2023/rt2023.html)).
- Suckbot: 30 mm 8-blade EDF rotor, 4500 kV BLDC (about 30,000 rpm), about 1.3 mm air gap, could lift about 140 g inverted. The builder ran a low fan speed and suspects it made no difference at those speeds — [Pololu blog 491](https://www.pololu.com/blog/491/nathans-line-following-robot-suckbot)
- PX-01: EDF30 at 14,000 kV with 20 A ESC (recommended at most 40% test / 70% competition), or a centrifugal 1104 4000 kV fan with about 3x lower consumption but less robust — [PCBWay PX-01](https://www.pcbway.com/project/shareproject/PX_01_OPRobots_Linefollower_KIT_b15659f3.html)
- Semreh: 4 x 1103 8000 kV brushless with fans/wings on a 4-in-1 15 A ESC; caused problems at RCX 2025 — [Hackaday Semreh](https://hackaday.io/project/202208-semreh-advanced-line-follower-robot)
- Turkish MEB line-follower rules forbid vacuum or fan motors — search snippet of [MEB 2024 rules](https://robot.meb.gov.tr/yukleme/Robot2024/1_line_follower_basic2024_erzurum.pdf)
- RobotChallenge (China edition, rev. 2025): the fetched rulebook has no fan/suction clause — [RC-LineFollower PDF](https://www.robotchallenge.org.cn/doc/en/RC-LineFollower.pdf)

### Inferences
- F-Maze Dieu 3.8 / FQA Q-02 ban suction fans that damage the course. A propeller "lift-down" unit with no floor contact is a grey area; per AGENTS.md, flag it to organisers rather than interpret it ourselves.
- Without downforce, max cornering speed is bounded by tyre friction: v_max = sqrt(mu * g * R). Mass reduction doesn't raise this limit. It only lowers inertia and motor/battery demand.
- Fan vibration couples into the gyro (Aegis2024 needed a damped mount). Relevant if any spinning mass is added.

### Gaps
- No measured cornering-speed or lap-time gain from suction was found in any source.

## IMU/gyro, MCU, battery, loop frequency

### Takeaway
Winners pair a 6-axis IMU (LSM6DSR/LSM6DSO/MPU6500) with wheel encoders for track mapping. STM32 dominates (about 50% of RoboTrace in 2017), with Renesas RX and ESP32 also present. Batteries are 2S-4S LiPo, often small (300 mAh 3S on a 130 g winner).

### Cited Findings
- Aegis2024: Renesas RX66T 160 MHz; LSM6DSR gyro; 3S 11.4 V 300 mAh LiPo powering everything; maps the course on run 1 and runs a shortcut path from run 2 (run 1 19.700 s, best 4.846 s) — [NTF RT68](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html)
- HBFS: STM32F303 (Cortex-M4F), MPU6500 on a replaceable module, multiple PID loops (rate not stated) — [HBFS](https://hbfsrobotics.com/linefollower)
- 3pi+ 2040: RP2040 125 MHz; LSM6DSO + LIS3MDL; 4 x AAA, motors boosted to 8 V — [Pololu guide](https://www.pololu.com/docs/0J86/all)
- Semreh: ESP32-WROOM, 2S then 3S with regulators removed — [Hackaday Semreh](https://hackaday.io/project/202208-semreh-advanced-line-follower-robot). PX-01: ESP32-C3 super mini — [PCBWay PX-01](https://www.pcbway.com/project/shareproject/PX_01_OPRobots_Linefollower_KIT_b15659f3.html)
- RoboTrace field: nearly all use encoders + gyros; about 50% STM32 — [Pololu forum](https://forum.pololu.com/t/fast-line-follower-build/11956)
- Hobby fast builds: about 75% used Pololu micro metal gearmotors + QTR + LiPo with boost converter (forum poster's survey); 16 vs 20 MHz clock unlikely to matter — [Pololu forum](https://forum.pololu.com/t/fast-line-follower-build/11956)
- Suckbot: 2S 1300 mAh competition pack, 8 V boost for motors — [Pololu blog 491](https://www.pololu.com/blog/491/nathans-line-following-robot-suckbot)

### Inferences
- Aegis2024's 4x speedup between run 1 and best run (19.7 s to 4.8 s) is the documented value of map-then-optimise. That matches F-Maze's two-run structure (run 1 IR-reactive, run 2 planned), but F-Maze bans changes after the course is revealed and allows only one 3-minute scouting run (manual Dieu 4.5, 4.7).
- ESP32-S3 is comparable in class to the MCUs used by podium robots (ESP32 on Semreh; STM32F3/RX66T on winners); MCU choice is not the bottleneck.

### Gaps
- No winner documented its control loop frequency. HBFS's above-10 kHz sensor read rate is the only rate found.

## Obstacle sensing for line-follow + obstacle contests

### Takeaway
Obstacle categories (RobotChallenge LF Enhanced, Polish LF Enhanced) use brick-like obstacles about 25 x 12 x 6.5 cm that the robot detours around. No fetched source documented a specific winning robot's obstacle sensor; VL53L0X timing-budget trade-offs are the main documented latency data.

### Cited Findings
- RobotChallenge LF Enhanced: obstacle about 25 x 12 x 6.5 cm; robot must detour and immediately return to the line; off-line allowed during the detour; also a 10 cm straight line gap and a seesaw at least 50 cm long, axle under 12 cm — [RC-LineFollower PDF](https://www.robotchallenge.org.cn/doc/en/RC-LineFollower.pdf)
- Polish LF Enhanced: obstacle about 25 x 6.5 x 12 cm, touching allowed, pass on the right side, rejoin within 40 cm behind it; line breaks up to 10 cm; line width may change from 19 mm to about 38 mm or about 10 mm; loops up to 30 cm diameter; hills up to 15 cm — [Skaner LF rules](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf)
- VL53L0X: about 20 ms timing budget (about 50 Hz) in fast mode vs 200 ms for high accuracy; all units default to I2C 0x29, so multiple sensors need address reassignment (XSHUT) — [ThinkRobotics tutorial](https://thinkrobotics.com/blogs/tutorials/vl53l0x-time-of-flight-sensor-tutorial-complete-setup-and-programming-guide) (tutorial, secondary)
- HBFS camera reportedly detects objects up to 1 m — [HBFS](https://hbfsrobotics.com/linefollower)

### Inferences
- At 1 m/s, a 20 ms ToF budget is 20 mm travel per sample, plus I2C and filter latency. Detection range must exceed stopping/steering distance plus a few samples. VL53L1X / VL53L4CD offer shorter timing budgets but would need datasheet verification.
- F-Maze obstacles are 10.5 x 6 cm and can stand on end (so tall) or lie flat. Lying flat, the height may be only about 6 cm or less, so a ToF beam mounted high could miss it. Sensor height and tilt must cover the lowest orientation; this needs measuring with the real block.
- A front bumper/microswitch is a cheap fallback, but F-Maze penalties for hitting obstacles must be checked in `docs/manual.md`.

### Gaps
- No documented obstacle-sensor choice, mounting, or latency from a winning LF Enhanced robot was found.
- Ultrasonic vs IR-distance (Sharp GP2Y) comparison in this context not researched.

## Documented example robots (summary table)

### Takeaway
Best-documented winner: Aegis2024 (robotrace 2023). Other entries are mid-tier or kits with partial specs.

### Cited Findings
| Robot | Result | Mass | Size | Drive | Sensors | MCU/IMU | Downforce | Source |
|---|---|---|---|---|---|---|---|---|
| Aegis2024 (RT68) | 1st, All-Japan robotrace 2023, 4.846 s | 130 g | 160x250x40 mm | 2x maxon DCX10L + ENX10, 4 x 20 mm wheels, 5 m/s, 10 m/s^2 | 16 analog SCM-014TB | RX66T 160 MHz, LSM6DSR | suction fan (DRW002) | [NTF](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html) |
| Umemoto 2019 | 1st, All-Japan Student robotrace | n/s | n/s | maxon DCX10L @ about 12 V | n/s | n/s | propeller unit (suction banned) | [maxon](https://maxonjapan.com/report/umemoto_2/) |
| HBFS LF | no results shown | n/s | n/s | Faulhaber 1028 BLDC, custom ESC | 16 IR + 250 FPS IR camera | STM32F303, MPU6500 | n/s | [HBFS](https://hbfsrobotics.com/linefollower) |
| Semreh | 1st-3rd, Portuguese events 2024 | n/s | n/s | N20 then 1020 coreless, 4WD geared | 8 QRE1113 + MCP3008 | ESP32 | 4x 1103 fans | [Hackaday](https://hackaday.io/project/202208-semreh-advanced-line-follower-robot) |
| The Chariot | 2nd LVBots, 1.17 m/s | 210 g | n/s | 35:1, 70 mm wheels, caster | 6 QTR-RC, shielded | ATmega32U4 | none | [Pololu](https://www.pololu.com/blog/487/brandons-line-following-robot-the-chariot) |
| Suckbot | mid-pack, 0.93 m/s | 300 g | n/s | 30:1 HP, 70 mm wheels | 6 QTR-1RC @ 12 mm | ATmega328P | 30 mm EDF | [Pololu](https://www.pololu.com/blog/491/nathans-line-following-robot-suckbot) |
| 3pi+ 2040 | reference platform | about 100 g (3rd party) | 97 mm dia. | 15:1/30:1/75:1, 32 mm wheels | 5 RC | RP2040, LSM6DSO | none | [Pololu](https://www.pololu.com/docs/0J86/all) |
| Impact | 1st, Vienna (claimed) | n/s | n/s | DC gearmotors, TB6612 | 14 analog KTIR0711S | STM32F103 | EDF tunnel | [elektroda](https://www.elektroda.com/rtvforum/topic2325660.html) (snippet) |

(n/s = not stated in source)

### Inferences
- The gap between hobby (about 1 m/s, 200-300 g) and robotrace winner (5 m/s straights, 130 g) comes from lower mass, high-power coreless motors at overvoltage, encoder+gyro mapping, and downforce. Of these, only downforce is unavailable under F-Maze rules.

### Gaps
- Could not fetch: Impact page (timeout), ak-rcroom 2023/2024/2025 (TLS cert error), anikinonikki 2018 (redirect loop). These hold more Japanese winner specs.
- RobotChallenge (Austria) and Robotic Arena winners' names and specs were not found. "Blizzard" was not found in any search.
