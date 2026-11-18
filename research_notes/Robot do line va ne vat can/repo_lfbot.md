# LF-Bot repository: contest rules, design, implementation state, and gaps

Scope: local repo `/home/minhvt/Projects/lfbot` as on disk 2026-11-16 (branch `main`, HEAD `f44dfc7`). The working tree has a lot of uncommitted work: 28 tracked files modified (941+/558-) and several **untracked** files: `firmware/lib/robot/line.{hpp,cpp}`, `race.{hpp,cpp}`, `firmware/test/test_race.cpp`, `docs/IMPLEMENTATION.md`, `docs/PLAN.md`, `docs/fqa.csv`, `docs/research/`. `line_sensor.{hpp,cpp}` were deleted. Everything below describes the files on disk. "Source" links are repo-relative paths with line numbers. Most docs are in Vietnamese; quotes are paraphrased.

## 1. Contest rules (docs/manual.md, docs/fqa.csv)

### Takeaway
The robot must be autonomous, fit in 16x16 cm, and run on at most 12 V nominal (3S allowed). It follows a 20 mm black line on white over a secret track with straights, S-curves and 90° turns, and has to get around brick obstacles placed on the line. Each team gets at most 2 timed runs per round, 3 min max each, and the best valid time counts. Code cannot be reloaded once the official track is revealed. Cutting across another line segment voids the run. The sources disagree on obstacle size and on whether leaving the line is allowed.

### Cited Findings
- Robot rules: fully autonomous, no human input after start except the start button ([docs/manual.md:18](docs/manual.md)). Max 16x16 cm, measured before the first run, and the robot may not change shape while running ([docs/manual.md:19](docs/manual.md)). FAQ Q-08 adds that sensors may not fold out during a run ([docs/fqa.csv:9](docs/fqa.csv)). Battery only, 12 V nominal, and a full 3S at 12.6 V is OK ([docs/manual.md:20](docs/manual.md), Q-01). Sensors and MCU are free choices (manual 3.4, Q-07). Suction fans and sliding contacts are banned: anything touching the floor must be a rolling wheel ([docs/manual.md:25](docs/manual.md), Q-02).
- Practice: unlimited practice runs 07:00–08:00, ending 30 min before the official start ([docs/manual.md:22](docs/manual.md)). During practice, code and parameters may change but the basic hardware may not ([docs/manual.md:42](docs/manual.md)).
- Track: designed by the organizers (BTC) and kept secret until the day. A sample practice track is published but differs from the real one ([docs/manual.md:30](docs/manual.md), Q-03). The line is 2 cm wide, black on white ([docs/manual.md:31](docs/manual.md)). The track is the same for every team within a round and changes only between rounds ([docs/manual.md:32](docs/manual.md)). Q-12 says it includes straight lines, S-curves and 90° turns, but its status is "Cần họp BTC" (needs an organizer meeting) ([docs/fqa.csv:13](docs/fqa.csv)).
- Obstacles: the manual says random boxes of 10.5x6 cm, lying or standing ([docs/manual.md:33](docs/manual.md)). FAQ Q-13 changes this to a brick, 205x95x95 mm, placed **centred on the line** at random positions ([docs/fqa.csv:16](docs/fqa.csv)). The repo's track fixtures still use 105x60 obstacles ([firmware/assets/track_vongloai.json](firmware/assets/track_vongloai.json)).
- Touching an obstacle (Q-14): a light brush that shifts it slightly is legal. Knocking it over, or pushing/dragging it away from its position, voids the run (DNF) ([docs/fqa.csv:19](docs/fqa.csv)). Under manual Điều 8, a team that interferes with an obstacle gets a warning, then disqualification ([docs/manual.md:64](docs/manual.md)).
- "Within 3 minutes and only once, to probe before the official run" ([docs/manual.md:34](docs/manual.md)). This reads as one 3-min scouting/trial pass, but the text does not say whether it happens on the official track.
- Finish: a clearly marked box. Touching it with a wheel or the body counts as finished ([docs/manual.md:35](docs/manual.md)). Timing is automatic: a photogate/sensor at the start line and a sensor zone at the finish box, with a manual stopwatch only as backup (Q-12, Q-17, Q-18) ([docs/fqa.csv:13](docs/fqa.csv), [docs/fqa.csv:22](docs/fqa.csv), [docs/fqa.csv:23](docs/fqa.csv)). Q-18: the organizers do not provide the track design file in advance.
- Impound: after technical check-in the organizers keep the robot, and no code reload or modification is allowed after the official track is announced ([docs/manual.md:36](docs/manual.md), Q-04).
- Runs: at most 2 per round, and the best valid time ranks ([docs/manual.md:40](docs/manual.md)). Max 3 min per run, otherwise DNF ([docs/manual.md:41](docs/manual.md)). Two rounds are planned, qualifier and final ([docs/manual.md:43](docs/manual.md)). Tie-break: second-best run first, then a run-off ([docs/manual.md:53-57](docs/manual.md)). Teams with no valid run rank below all others ([docs/manual.md:49](docs/manual.md)).
- Leaving the line: the manual says going outside the line to find the way is not a foul, but going out and **cutting a line** loses the run ([docs/manual.md:44](docs/manual.md), [docs/manual.md:69](docs/manual.md)). Q-15 replaced an earlier "off-line for more than 5 s" rule with the line-cutting rule. Off-line time is measured by sensors, and avoiding an obstacle then rejoining the line correctly is valid ([docs/fqa.csv:20](docs/fqa.csv)). Q-05 says running "out of the line or cutting the line" counts as a lost run ([docs/fqa.csv:6](docs/fqa.csv)).
- Venue: Q-16 says the planned location is an outdoor lobby on the 5th floor, with wind ([docs/fqa.csv:21](docs/fqa.csv)). Dates: qualifier 11/11/2026, final 2/12/2026, at FPT University HCMC ([docs/manual.md:84-86](docs/manual.md)).

### Inferences
- **Contradictions to flag:**
  1. Obstacle size: 10.5x6 cm in manual 4.4 vs a 205x95x95 mm brick in Q-13. The repo design follows Q-13 ([docs/ARCHITECTURE.md:26](docs/ARCHITECTURE.md)).
  2. Q-05 seems to make any excursion off the line a lost run, while manual 5.5 and Q-15 allow excursions and penalise only cutting across another line.
  3. Practice wording: "30 min before" vs a "07:00–08:00" window.
  4. Q-12 is still marked as needing an organizer meeting.
- The rules give no different allowances for run 1 and run 2. Both are timed and the faster valid one counts. Because code cannot be reloaded after the track is revealed (4.7), any "learn in run 1, exploit in run 2" strategy has to be fully autonomous, with the map kept on the robot (flash). Whether run 1, or the single 3-min probe in Điều 4.5, may be used as a mapping pass is not ruled out; the repo plans to use run 1 only and never mentions the 4.5 probe.
- The qualifier date (2026-11-11) has already passed relative to the repo snapshot (2026-11-16). The repo records no competition result. The final is 2026-12-02.

### Gaps
- No official track drawing. Only the repo's own fixture images and JSON exist (`firmware/assets/track_vongloai.*`, `track_chungket.*`, arenas 6000x1500 and 10000x1500 mm). ARCHITECTURE says they do not represent the secret track ([docs/ARCHITECTURE.md:20](docs/ARCHITECTURE.md)).
- The docs do not specify the finish-box marking (colour or pattern), the start-line geometry, or whether line junctions or crossings exist on the real track.

## 2. Target architecture, ADRs, plan and naming

### Takeaway
The current design is ADR-023 (Accepted 2026-11-11, "chưa triển khai", i.e. not yet implemented). Under it:
- Hardware facts are measured once and stored as constants. The robot calibrates only field-dependent quantities: IR levels and polarity, and gyro bias.
- Run 1 must finish and records a map: ψ and event flags every 10 mm of encoder distance.
- Run 2 uses a curvature-limited v(s) profile, curvature feedforward ω = v·κ(s), and re-localisation at landmarks.
- One `maneuver` primitive handles turn-to-angle and go-distance moves.
- One button, steady LED colours, a 500 Hz control task, and acceptance by real runs only.

### Cited Findings
- Goals: run 1 must finish within 180 s and store the map. Run 2 must be faster than run 1 on the same valid path ([docs/ARCHITECTURE.md:7-10](docs/ARCHITECTURE.md)). No shortcuts and no line-cutting, and nothing is tuned on competition day ([docs/ARCHITECTURE.md:14-15](docs/ARCHITECTURE.md)).
- Track features to handle (from the qualifier fixture): long straights, S-curves, sharp zigzags, 90° square waves, overlapping squares and circles where crossings must be driven straight through, a dead end that is the finish, and bricks bypassed on the **right** ([docs/ARCHITECTURE.md:22-26](docs/ARCHITECTURE.md)).
- One button: press 1 = IR self-scan, then wait (purple). Press 2 = run. Any press while scanning or running = immediate stop. Holding the button at power-up erases the map. The run is picked automatically: run 2 if a complete map is in flash ([docs/ARCHITECTURE.md:28-37](docs/ARCHITECTURE.md)).
- Sensor roles table: IR gives line position and shape. Encoders give wheel speed, distance s and stall detection. Gyro Z gives heading ψ, closed-loop turns, holding the turn rate through junctions, and ψ(s) for the map. ToF handles approach and brick-edge (width) measurement. The battery reading is used for feedforward compensation and to block start. The accelerometer is unused ([docs/ARCHITECTURE.md:72-89](docs/ARCHITECTURE.md)).
- Line reading: per-eye normalisation 0..1. Position is the centroid of eyes above 0.2, in eye pitches from −3.5 to +3.5. Shapes are None / Line (1–3 eyes) / Wide (6 or more) / Split (two clusters). Thresholds are provisional ([docs/ARCHITECTURE.md:120-134](docs/ARCHITECTURE.md)).
- Three control layers:
  1. Wheel speed loop at 500 Hz: motor-model feedforward scaled by battery voltage, plus PI with back-calculation anti-windup.
  2. PD on line position producing ω, plus feedforward ω = v·κ(s) in run 2.
  3. Forward speed v chosen by `race`.
  Wheels get v ± ω·b/2, with the effective track b measured from encoder vs gyro ([docs/ARCHITECTURE.md:136-144](docs/ARCHITECTURE.md)).
- Situations: at a Wide/Split, hold the current turn rate per the gyro, with a distance limit. Line lost at the edge means a corner: slow down and pivot in place until the centre eye sees the line. Line lost mid-bar with ToF clear makes a finish candidate. If the search fails: return, back up, then `lost` ([docs/ARCHITECTURE.md:146-154](docs/ARCHITECTURE.md)).
- Finish: confirm by driving about one body length further and sweeping ± a small angle. Be conservative, since an early stop is a DNF ([docs/ARCHITECTURE.md:156-163](docs/ARCHITECTURE.md)).
- Brick bypass (right side):
  1. Approach, using only ToF readings with range status 0.
  2. Measure the brick width by turning right slowly until the ToF range jumps: w = d·tan θ.
  3. Three legs: +90° then w + 55 mm + margin; −90° then 205 mm + body length + margin; −90° then drive until IR sees the line.
  4. Accept the line only inside a distance window, then realign to the approach heading ψ.
  Time is only a failure bound ([docs/ARCHITECTURE.md:165-176](docs/ARCHITECTURE.md)).
- Two-run plan: run 1 runs at a fixed, verified cruise speed and logs `{ψ, flags}` every 10 mm, 4 bytes per sample. After the finish:
  - compute κ(s) = dψ/ds, smoothed;
  - cap speed at v ≤ √(a_lat/|κ|), with separate caps for corner entry and brick approach;
  - run a backward braking pass and a forward acceleration pass;
  - store the result in a `course` flash partition.
  Run 2 uses the same follower plus v(s) and κ feedforward. s is re-anchored at landmarks (corner, crossing, brick) inside a ±Δs window. A missed landmark falls back to run-1 speed until the next match ([docs/ARCHITECTURE.md:178-196](docs/ARCHITECTURE.md)).
- Runtime:
  - control task: core 1, 500 Hz, sole owner of `Robot`;
  - sensors task: core 0, gyro at 500 Hz, ToF polled every 20 ms;
  - service task: 50 Hz.
  IR averages all ADC sweeps between ticks (about 5 at 2667 sweeps/s). Gyro uses DLPF and ±1000 dps. No heap allocation and no printf in control ([docs/ARCHITECTURE.md:198-215](docs/ARCHITECTURE.md)).
- Safety: an esp_timer heartbeat checked every 10 ms (coast if the control task stops), a 300 ms stall check, a 3-cycle stale-IR check, a battery cut-off, Stop overriding everything, and one enum value per stop reason ([docs/ARCHITECTURE.md:217-226](docs/ARCHITECTURE.md)).
- Planned module list includes `course.*` (map, v(s), landmark matching) and `course_store` (flash) ([docs/ARCHITECTURE.md:228-249](docs/ARCHITECTURE.md)).
- ADR status table ([docs/decisions/README.md](docs/decisions/README.md)):
  - 019 (module hardware): Accepted, firmware not yet accepted.
  - 020 (ESP32 restart, drop the simulator): Accepted.
  - 021: partially superseded by 023.
  - 022 (self-tuning 3-layer control, IMC/lambda, auto speed selection, steering gain "k = 4ζ²v/d²"): **Superseded** by 023.
  - 023: Accepted, not implemented.
  ADR-022 cited a1k0n "Fast line-following robots" and UnderBird (Robotrace) as references ([docs/decisions/ADR-022-control-strategy-and-codebase.md:33](docs/decisions/ADR-022-control-strategy-and-codebase.md)).
- ADR-023 rejected alternatives: self-deriving every coefficient at the field, IR plus open-loop PWM only, a time-based scan, and "8 white eyes for some time = finish" ([docs/decisions/ADR-023-sensor-driven-two-run-firmware.md](docs/decisions/ADR-023-sensor-driven-two-run-firmware.md)).
- The research doc [docs/research/esp32-firmware-restart.md:77-100](docs/research/esp32-firmware-restart.md) is more ambitious than ARCHITECTURE:
  - a short-horizon candidate-motion obstacle planner, explicitly rejecting "turn right N ms → go X mm → turn left" FSMs;
  - run-2 localisation with bounded hypotheses;
  - spatial path optimisation within valid corridors plus forward/backward speed passes.
  ARCHITECTURE adopted the simpler fixed-rectangle bypass instead.
- PLAN steps:
  1. Infrastructure and closed-loop motion: accepted on the robot, except the button-stop and USB-pull checks.
  2. Self-scan and slow following: code done, not accepted on the robot.
  3. Corners, junctions, finish: code written, not tuned.
  4. Brick: not accepted.
  5. Map, flash, run 2: not started.
  Fallback: after step 3, the firmware must be able to finish run 1 on a track with no bricks ([docs/PLAN.md:20-121](docs/PLAN.md)). Open risks: left-wheel dead band about 0.45 on the floor, outdoor light, a brick near another line, a single forward ToF, and N20 torque ([docs/PLAN.md:128-136](docs/PLAN.md)).
- Naming (NAMING.md): functions are verbs for what the robot does, structs are CamelCase domain nouns, and theory words (PD, feedforward, curvature, μ) stay out of public names ([docs/NAMING.md](docs/NAMING.md)).

### Inferences
- The architecture already names several best practices of fast line followers: curvature feedforward, κ-limited speed profile with forward/backward passes, landmark re-anchoring, and a gyro-assisted wheel-speed cascade. All except the gyro/encoder manoeuvres are still on paper.

### Gaps
- No ADR or document decides the run-2 path geometry (racing line within a corridor). Only the speed profile is planned, which is consistent with the no-line-cutting rule.
- The docs give no lateral/braking acceleration limits, tyre friction or target speeds. They are left to be "tuned on the home track".

## 3. Hardware

### Takeaway
Off-the-shelf modules:
- ESP32-S3 N16R8 devkit;
- one Thegioiic 8-channel **analog** IR bar, no emitter control;
- 2x GA12-N20 12 V 1000 RPM gearmotors with AB encoders;
- 2x DRV8871 drivers;
- 43 mm wheels and a front ball caster;
- 3S 450 mAh LiPo;
- VL53L1X ToF at the front;
- "GY-9250" IMU, which is actually an MPU-6500 (gyro and accel only);
- voltage-divider module for battery sensing.

The custom PCBs (MAIN, IR-03 curved TCRT5000 array) are frozen. The IR bar's eye pitch and its distance ahead of the axle are **not measured**.

### Cited Findings
- BOM entries ([docs/HARDWARE_BOM.csv](docs/HARDWARE_BOM.csv)):
  - Motor: GA12-N20 12 V 1000 RPM with AB encoder. The BOM notes that 1000 RPM with a 43 mm wheel is about 2.25 m/s, but this is not a guaranteed robot speed.
  - Wheels: N20 type, Ø43 mm, 18 mm wide, rubber, grip coefficient not measured.
  - Driver: DRV8871, 3.6 A peak.
  - Battery: Ovonic 450 mAh 3S 100C.
  - ToF: VL53L1X TOF400C, one at the front.
  - IR: 8-eye analog bar, "2–10 mm" height as stated by the shop.
  - Backups: two MKE-S16 bars, unused.
  - Caster: a rolling front ball in a 3D-printed mount.
- Assembly facts recorded in ARCHITECTURE:
  - IR order: D1 far left → D8 far right, on ADC1 channels {0,1,3,4,5,6,7,8}.
  - Left motor inverted, right encoder inverted.
  - Ø43 mm wheels; 110 mm outer width, which is not the wheelbase.
  - Gyro sign: to be checked on the first run.
  ([docs/ARCHITECTURE.md:53-68](docs/ARCHITECTURE.md))
- Measured on the robot: `kMmPerCount` = 0.0313 (ToF vs encoder) and `kTrackCountsPerRad` = 3040, giving an effective wheel spacing of about 95 mm ([firmware/lib/robot/config.hpp:28-38](firmware/lib/robot/config.hpp)). Floor motor model: gain 22480/21009 counts/s/duty, τ 0.62/0.55 s, dead band 0.21/0.22. Earlier wheels-in-air measurements were off by about 2x in gain and 4x in τ ([docs/IMPLEMENTATION.md:17-21](docs/IMPLEMENTATION.md)).
- IR wiring: D1–D8 go straight to the ADC **with no divider**, because the 10k/47k divider is drawn but not installed. Emitter on/off modulation is not available on the bought module ([hardware/wiring.md:69](hardware/wiring.md), [hardware/wiring.md:83](hardware/wiring.md)). Real runs saw the floor at 3750–4025 and the line at 4095, so contrast was only about 250 counts and near the ADC rail ([docs/IMPLEMENTATION.md:36](docs/IMPLEMENTATION.md), [docs/IMPLEMENTATION.md:47](docs/IMPLEMENTATION.md)). wiring.md instead reports white at 1880–2850 on 2026-11-10.
- IMU: WHO_AM_I = 0x70, i.e. MPU-6500, no magnetometer ([hardware/wiring.md:118](hardware/wiring.md)). Driver setup: ±1000 dps, 92 Hz DLPF, 500 Hz rate ([firmware/src/esp32/imu.cpp:21-25](firmware/src/esp32/imu.cpp)).
- ToF: short distance mode (1), 20 ms timing budget, 25 ms inter-measurement period, INT pin not connected ([firmware/src/esp32/tof.cpp:37-39](firmware/src/esp32/tof.cpp), [hardware/wiring.md:61](hardware/wiring.md)).
- Motor PWM: MCPWM at 20 kHz, 500 ticks per period ([firmware/src/esp32/motors.cpp:14-15](firmware/src/esp32/motors.cpp)). Encoders: PCNT full quadrature ([firmware/src/esp32/encoders.cpp:10-34](firmware/src/esp32/encoders.cpp)). ADC continuous at 24 kHz over 9 channels ([firmware/src/esp32/adc_inputs.cpp:15-17](firmware/src/esp32/adc_inputs.cpp)).
- Pins: [firmware/src/esp32/pins.hpp](firmware/src/esp32/pins.hpp). IR on GPIO1–9, battery on GPIO10, motors on 11–14, I2C on 15/16, encoders on 17/18/21/47, button on 40, ToF XSHUT on 41, LED on 48.

### Inferences
- With the floor model (gain about 22000 cps/duty, dead band about 0.21), the cruise duty of 0.7 gives roughly (0.7−0.21)·22000·0.0313 ≈ 0.34 m/s at steady state. This is my estimate, not a measured speed. A τ of about 0.6 s is slow; IMPLEMENTATION blames robot mass ([docs/IMPLEMENTATION.md:81](docs/IMPLEMENTATION.md)).
- An analog bar near the ADC rail, with no divider and no emitter modulation, is weak against outdoor sunlight (Q-16). The logs back this up: "lost heading as soon as it entered sun" ([docs/IMPLEMENTATION.md:44](docs/IMPLEMENTATION.md)).

### Gaps
- Unknown: eye pitch, bar width, bar height, bar-to-axle distance, total mass and CoG. The test plant assumes 7 mm pitch and the bar 70 mm ahead of the axle ([docs/IMPLEMENTATION.md:31](docs/IMPLEMENTATION.md)), while config uses `kCornerAdvanceMm` = 45 as a placeholder ([firmware/lib/robot/config.hpp:153-156](firmware/lib/robot/config.hpp)).
- The final chassis footprint and length are not recorded in the docs.

## 4. Current code: modules and what is implemented

### Takeaway
The core (`firmware/lib/robot`) runs at 500 Hz and implements:
- a histogram-based IR self-scan;
- centroid line position with shape classification;
- PD steering and offset-based speed reduction, both output **directly as PWM duty** (no wheel-speed loop);
- gyro/encoder PID manoeuvres;
- stop-and-pivot corner handling;
- an open-loop-geometry right-hand brick bypass;
- a finish confirmation.

No map, no `course` module, no run 2, no speed profile, no curvature feedforward, no flash map storage. The code has drifted from ARCHITECTURE in several places.

### Cited Findings
- **config.hpp** (all constants):
  - control rate 500 Hz ([config.hpp:13-14](firmware/lib/robot/config.hpp));
  - max duty 0.9, min battery 9.9 V;
  - stall when duty is at least 0.35 and both wheels are below 300 cps for 200 ticks, i.e. 0.4 s ([config.hpp:41-48](firmware/lib/robot/config.hpp));
  - manoeuvre PID gains ([config.hpp:73-93](firmware/lib/robot/config.hpp));
  - line-reading rules ([config.hpp:98-125](firmware/lib/robot/config.hpp));
  - scan swings ±0.5/0.9/1.3 rad ([config.hpp:128](firmware/lib/robot/config.hpp));
  - following gains ([config.hpp:132-143](firmware/lib/robot/config.hpp)): `kCruiseDuty` 0.7, `kSteerDutyPerEye` 0.20, `kSteerDutyDamping` 0.02, `kRateFilterPerS` 6, `kSlowByPosition` 0.12, `kSlowByRate` 0.02, `kMinFollowFraction` 0.45;
  - junction crawl duty 0.30 with a 120 mm limit, corner advance 45 mm, probe ±1.66 rad (~95°), finish-confirm distance 100 mm, IR stale after 20 ms ([config.hpp:150-162](firmware/lib/robot/config.hpp));
  - brick ([config.hpp:166-179](firmware/lib/robot/config.hpp)): approach trigger 150 mm, brake duty 0.4, side clearance 210 mm, length clearance 600 mm, search 300 mm;
  - log every 10 ticks, i.e. 50 Hz ([config.hpp:185](firmware/lib/robot/config.hpp)).
- **types.hpp**: `Measurements` carries timestamped samples: raw IR, encoders, gyro Z, distance, battery, ToF range status ([types.hpp:31-39](firmware/lib/robot/types.hpp)). `Target{forward, turn}` is in duty, and its comment says there is no inner speed loop and no motor model ([types.hpp:83-90](firmware/lib/robot/types.hpp)). `RaceState` covers Scan/Return/Ready/Follow/Corner/ConfirmGo/Bypass/Finished/Failed ([types.hpp:102-112](firmware/lib/robot/types.hpp)). `LineShape` is None/Line/Split/Wide, and `StopReason` has 15 values ([types.hpp:115-140](firmware/lib/robot/types.hpp)).
- **line.cpp** (untracked, new):
  - Self-calibration from per-eye histograms ([line.cpp:49-88](firmware/lib/robot/line.cpp)). The floor level is the per-eye median and the line level is the 2% tail on the rarer side. Polarity is decided by one vote per eye. An eye counts as usable if separation ≥ 100 counts and ≥ 2x the floor spread. At least 5 usable eyes are needed.
  - Darkness is normalised and clamped to 0..1 ([line.cpp:100-113](firmware/lib/robot/line.cpp)).
  - Runs of eyes with darkness ≥ 0.5 form patches; single-eye "specks" next to a real patch are dropped ([line.cpp:124-164](firmware/lib/robot/line.cpp)).
  - Position is the darkness-weighted centroid of the patch plus one neighbour on each side, in eye pitches ([line.cpp:167-179](firmware/lib/robot/line.cpp)).
  - The patch nearest the previous position is followed. Wide means dark eyes ≥ 75% of usable eyes, and more than one patch means Split ([line.cpp:183-213](firmware/lib/robot/line.cpp)).
  - Floor-level drift adaptation is bounded to 50% of contrast at 0.5/s ([line.cpp:215-229](firmware/lib/robot/line.cpp)).
  - There is no sub-pixel interpolation (parabolic or LUT), and line-loss extrapolation uses only the last side seen.
- **race.cpp** (untracked, new):
  - `follow_turn` = clamp(−0.20·pos − 0.02·pos_rate), output as duty. `follow_forward` = 0.7 × clamp(1 − 0.12|pos| − 0.02|rate|, 0.45, 1) ([race.cpp:8-19](firmware/lib/robot/race.cpp)).
  - Position rate is a low-passed finite difference ([race.cpp:163-168](firmware/lib/robot/race.cpp)).
  - A ToF reading below 150 mm switches to bypass ([race.cpp:151-157](firmware/lib/robot/race.cpp)).
  - At Wide/Split the robot crawls at duty 0.30 and **holds the last steering duty**, which is open loop and does not use the gyro rate. It fails after 120 mm ([race.cpp:175-185](firmware/lib/robot/race.cpp)).
  - At None it enters a corner: go 45 mm, pivot ±95° to the side the line was drifting toward, then the other side, then back to the start heading. If none of these finds the line, it does the finish confirm, 100 mm forward ([race.cpp:186-237](firmware/lib/robot/race.cpp)).
  - Bypass ([race.cpp:246-299](firmware/lib/robot/race.cpp)) runs this sequence:
    1. reverse-duty braking to below 300 cps;
    2. absolute turn to approach heading −90°;
    3. go 210 mm;
    4. turn back to the approach heading;
    5. go 600 mm;
    6. turn +90°;
    7. go up to 300 mm, accepting **any** non-None shape as the line ([race.cpp:258-263](firmware/lib/robot/race.cpp));
    8. realign to the approach heading, then follow.
  - There is no brick-width measurement, no ToF guard on the legs, and no expected-distance window for reacquisition.
- **motion.cpp**:
  - Wheel speed is the count difference over a 10-tick window, i.e. 20 ms ([motion.cpp:22-47](firmware/lib/robot/motion.cpp), [config.hpp:67](firmware/lib/robot/config.hpp)).
  - Heading ψ is the trapezoidal integral of gyro Z minus bias ([motion.cpp:49-72](firmware/lib/robot/motion.cpp)).
  - s is the mean of the two encoder counts ([motion.cpp:74-76](firmware/lib/robot/motion.cpp)).
  - Gyro bias is taken from the first still 0.25 s block within 3 s ([motion.cpp:110-127](firmware/lib/robot/motion.cpp)).
  - A gyro-vs-encoder yaw sign check is included ([motion.cpp:133-137](firmware/lib/robot/motion.cpp)).
  - There is no x/y pose and no gyro/encoder fusion beyond that sign check.
- **drive.cpp**: duty = target × clamp(11.4 V / Vbatt, 0.8, 1.4). Left = forward − turn and right = forward + turn, each clamped to ±0.9 **independently**, so steering authority is lost when one side saturates. Stall detection is included ([drive.cpp:21-43](firmware/lib/robot/drive.cpp)).
- **maneuver.cpp**: `go` is PID on distance in duty, with only a P heading hold; `heading_integral_` and `kHoldIPerRadS` are declared but unused ([maneuver.cpp:44-60](firmware/lib/robot/maneuver.cpp), [config.hpp:81](firmware/lib/robot/config.hpp)). `turn` is PID on angle, with D taken from the gyro rate ([maneuver.cpp:62-76](firmware/lib/robot/maneuver.cpp)). There is no trapezoidal velocity profile.
- **robot.cpp**:
  - phases Idle → Settling (gyro bias) → Running → Ready (scan done, wheels braked) → Running → Stopped/Finished;
  - safety checks: battery, stale gyro, stale IR (10 ticks), gyro disagreement, stall ([robot.cpp:39-57](firmware/lib/robot/robot.cpp));
  - the manoeuvre timeout is **not** applied in race mode ([robot.cpp:135](firmware/lib/robot/robot.cpp));
  - LED colours ([robot.cpp:146-166](firmware/lib/robot/robot.cpp)): blue means "ready" and has nothing to do with which run;
  - one black-box record per 10 ticks ([robot.cpp:168-202](firmware/lib/robot/robot.cpp));
  - wheel inversion is applied only at the output ([robot.cpp:273-279](firmware/lib/robot/robot.cpp)).
- **log.hpp**: a 4000-record RAM ring holding phase, reason, ToF, s, ψ, yaw rate, cps, duty, raw IR, race state, shape, dark eyes and position ([log.hpp](firmware/lib/robot/log.hpp)). The ESP32 black box saves the last 5 runs to a flash `blackbox` partition ([firmware/partitions.csv:5](firmware/partitions.csv)). There is **no `course` partition**.
- **ESP32 runtime** ([firmware/src/esp32/main.cpp](firmware/src/esp32/main.cpp)):
  - control task: core 1, priority 6, 2 ms period; reads ADC and encoders, takes the gyro and ToF snapshot under a spinlock, then calls `Robot::step` and `Motors::apply` (lines 94–131);
  - sensors task: core 0, 2 ms period; gyro every pass, ToF every 10th pass (lines 25–28, 67–92);
  - service task: 20 ms; LED, console, black-box save after a run (lines 133–167);
  - esp_timer watchdog: coasts the motors if two 10 ms heartbeats are missed (lines 29–31, 170–183).
  Measured max tick time is 0.98 ms with no late ticks ([docs/IMPLEMENTATION.md:21](docs/IMPLEMENTATION.md)).
- On-robot history ([docs/IMPLEMENTATION.md:28-92](docs/IMPLEMENTATION.md)):
  - Step 1 motion was accepted: ±90° turns stopped at ±87.8°, and straight drives held heading within 1.2°.
  - Real line runs: three indoor runs followed the line but overshot at a 90° turn. One false finish was declared at a T junction. Outdoors, `scan_failed`, then the robot lost the line in sun.
  - Fixes made since: classify junctions only after passing them, prefer right at a tie, drop specks, a per-eye contrast rule, and pivot-in-place corners.
  - The owner then asked for faster reactions. The wheel-speed loop was **removed** in favour of direct duty with PD on position and its rate ([docs/IMPLEMENTATION.md:83-92](docs/IMPLEMENTATION.md)).
  - None of the later race code has been validated on the robot; it is only tested in simulation.

### Inferences
- **Code vs ARCHITECTURE drift:**
  - ARCHITECTURE §6.3 specifies a 500 Hz encoder wheel-speed loop with feedforward and PI; the code drives duty directly ([firmware/lib/robot/drive.cpp:21-31](firmware/lib/robot/drive.cpp) vs [docs/ARCHITECTURE.md:138](docs/ARCHITECTURE.md)).
  - ARCHITECTURE says to hold the gyro turn rate through junctions; the code holds the last duty ([race.cpp:184](firmware/lib/robot/race.cpp)).
  - Brick-width measurement is missing.
  - Run selection by LED, `course.*` and the `course` partition are missing.
  - The stale-IR limit is 3 cycles in the doc vs 10 ticks in code. The stall time is 300 ms vs 400 ms.
  - Several documented stop reasons (`sweep_no_turn`, `blocked`, `lost`) are not exact enum names.
- docs/decisions/README.md still lists ADR-023 as "chưa triển khai" (not implemented), although steps 1–4 are partly coded.

### Gaps
- Actual achieved line-following speed and lap times are not recorded anywhere.
- No log data exists in the repo to evaluate the PD gains.

## 5. Tests (firmware/test)

### Takeaway
Host "contract" tests drive a simulated plant: a first-order motor model with the measured floor gain, τ and dead band, plus assumed IR geometry. They cover line reading, motion, robot safety and race scenarios. They explicitly do not prove robot behaviour.

### Cited Findings
- Entry point: `run_line`, `run_motion`, `run_robot`, `run_race`. It prints "PASS (not a robot run)" ([firmware/test/test_main.cpp](firmware/test/test_main.cpp), [firmware/test/suites.hpp](firmware/test/suites.hpp)).
- Plant motor constants 22480/21009, τ 0.62/0.55, dead band 0.21/0.22; the controller-side model is deliberately weaker ([firmware/test/plant.hpp:15](firmware/test/plant.hpp), [firmware/test/plant.hpp:53-60](firmware/test/plant.hpp)).
- test_line: both polarities, a bright low-contrast floor, blind eyes, a floor near the rail, glare not flipping polarity, a scan that never crossed the line, position sign, shapes, split tracking, unusable eyes, bounded floor adaptation ([firmware/test/test_line.cpp:34-180](firmware/test/test_line.cpp)).
- test_motion: wheel sign, gyro integration and bias, shaken-robot bias, direct duty, stall ([firmware/test/test_motion.cpp:14-130](firmware/test/test_motion.cpp)).
- test_robot: go and turn accuracy, button stop in the same tick and during settling, debounce, stale gyro, wrong gyro sign, jammed wheel, low battery, nothing moving without a command, logged stop reason, stale ToF logged as missing, brake-then-release ([firmware/test/test_robot.cpp:23-253](firmware/test/test_robot.cpp)).
- test_race ([firmware/test/test_race.cpp:91-313](firmware/test/test_race.cpp)):
  - follows a line to the end on 4 lighting/floor variants;
  - poor starting placement;
  - left and right corners;
  - a crossing taken straight;
  - a T junction turned (right preferred) rather than read as the finish;
  - noisy outer eyes, a flickering eye, blind-eye tolerance, no line at all;
  - two-press start;
  - a filled-node corner;
  - a brick bypassed on a straight 1300 mm line;
  - follow-law shape;
  - button stop.

### Inferences
- There are no tests for curves, S-bends, speed profile, map, run 2 or landmark matching, because those features do not exist. The brick test covers only a brick on a straight line, not a brick near a corner or another line.

### Gaps
- Test results were not re-run in this read-only study. The last recorded green `just check`/`just sanitize` is 2026-11-10, before the current race code ([docs/IMPLEMENTATION.md:161](docs/IMPLEMENTATION.md)).

## 6. Open questions and gaps vs state-of-the-art line followers

### Takeaway
The architecture plans curvature feedforward, a κ-limited forward/backward speed profile, landmark re-anchoring and a wheel-speed loop, but none of these is in code. The current robot is a reactive, duty-driven PD follower with stop-and-pivot corners and a fixed-geometry brick detour. It has no run-2 advantage yet.

### Cited Findings (each gap with its current state)
- **Map recording / run 2 / speed profiling:** no `course.*`, no `course` partition, no κ(s), v(s) or landmark matching ([firmware/partitions.csv](firmware/partitions.csv); planned in [docs/PLAN.md:105-121](docs/PLAN.md), [docs/ARCHITECTURE.md:178-196](docs/ARCHITECTURE.md)).
- **Curvature feedforward:** steering is feedback only, PD on IR position ([race.cpp:8-13](firmware/lib/robot/race.cpp)). ω = v·κ is planned for run 2 only ([docs/ARCHITECTURE.md:139](docs/ARCHITECTURE.md)).
- **Gyro yaw-rate inner loop:** while following, the gyro is not used at all. It is used only in manoeuvres, the heading hold and the sign check ([maneuver.cpp:57-76](firmware/lib/robot/maneuver.cpp), [motion.cpp:133](firmware/lib/robot/motion.cpp)). Steering commands raw turn duty rather than a yaw rate.
- **Encoder wheel-velocity loops:** removed; there is only battery-scaled open-loop duty ([drive.cpp:21-31](firmware/lib/robot/drive.cpp), [docs/IMPLEMENTATION.md:83-92](docs/IMPLEMENTATION.md)). Wheel speed is estimated from a 20 ms window, used only for odometry and stall detection ([config.hpp:64-67](firmware/lib/robot/config.hpp)). The two wheels' differing dead bands go uncompensated ([docs/IMPLEMENTATION.md:92](docs/IMPLEMENTATION.md)).
- **Saturation handling:** per-wheel clamping with no turn-priority desaturation ([drive.cpp:30-31](firmware/lib/robot/drive.cpp)).
- **Speed selection:** a fixed cruise duty of 0.7 reduced by line offset and rate ([race.cpp:15-19](firmware/lib/robot/race.cpp)). There is no acceleration limiting and no look-ahead from a map.
- **Corners:** stop-and-pivot, with a 45 mm placeholder advance and ±95° probes ([race.cpp:186-237](firmware/lib/robot/race.cpp), [config.hpp:153-158](firmware/lib/robot/config.hpp)). There is no high-speed handling of 90° turns (e.g. slowing on approach from the map, or a skid-turn); the follower only detects corners after the line is lost.
- **Junctions:** open-loop held duty, not the gyro-rate hold in the design ([race.cpp:175-185](firmware/lib/robot/race.cpp)).
- **Marker-based localisation:** none. The real track has no side markers (the rules mention none); the plan uses corners, crossings and bricks as natural landmarks ([docs/ARCHITECTURE.md:194-195](docs/ARCHITECTURE.md)).
- **Pose estimation:** only s and ψ; no x/y odometry and no wheel-slip detection.
- **Obstacle bypass:** a fixed rectangle, 210 mm side × 600 mm forward × search of up to 300 mm ([config.hpp:166-179](firmware/lib/robot/config.hpp), [race.cpp:246-299](firmware/lib/robot/race.cpp)). Missing: ToF width measurement (planned in [docs/ARCHITECTURE.md:168](docs/ARCHITECTURE.md)), ToF guarding on the legs, and a distance window for reacquisition. Any dark reading is accepted as the line, which risks latching onto a different line segment and being judged as line-cutting ([race.cpp:258](firmware/lib/robot/race.cpp)). The research doc rejects this kind of timed or geometric FSM in favour of a feedback planner ([docs/research/esp32-firmware-restart.md:77-87](docs/research/esp32-firmware-restart.md)).
- **ToF detection range:** stops at 150 mm. Short mode with a 20 ms budget is polled every 20 ms ([config.hpp:166](firmware/lib/robot/config.hpp), [firmware/src/esp32/tof.cpp:37-39](firmware/src/esp32/tof.cpp)). Braking is by reverse duty ([race.cpp:256](firmware/lib/robot/race.cpp)). There is no speed-dependent approach slowdown, although ARCHITECTURE asks for one ([docs/ARCHITECTURE.md:167](docs/ARCHITECTURE.md)).
- **Line sensor front end:** no emitter modulation or ambient subtraction ([hardware/wiring.md:83](hardware/wiring.md)), and the readings are near the ADC rail without the planned divider ([docs/IMPLEMENTATION.md:36](docs/IMPLEMENTATION.md)). Outdoor failure has been observed ([docs/IMPLEMENTATION.md:44](docs/IMPLEMENTATION.md)). The 8 eyes have unknown pitch, and there is no look-ahead sensor.
- **Finish detection:** relies on line-end, since there is no finish-box sensor. Every line loss first goes through the corner probe, so the robot pivots ±95° before confirming the finish ([race.cpp:198-237](firmware/lib/robot/race.cpp)). This costs time, but the timer stops at the gate anyway ([docs/ARCHITECTURE.md:158](docs/ARCHITECTURE.md)).
- **Failure bounds in race mode:** the manoeuvre timeout is skipped ([robot.cpp:135](firmware/lib/robot/robot.cpp)). A stuck pivot relies on the stall detector only.
- **Dead config:** `kHoldIPerRadS` and `heading_integral_` are unused ([config.hpp:81](firmware/lib/robot/config.hpp), [maneuver.cpp:23](firmware/lib/robot/maneuver.cpp)).
- Open items listed in the docs:
  - the gyro sign is unverified ([config.hpp:23-26](firmware/lib/robot/config.hpp));
  - `kCornerAdvanceMm`, `kWideFraction` and the bypass sizes are placeholders ([docs/IMPLEMENTATION.md:76](docs/IMPLEMENTATION.md));
  - PD gains are not tuned on a real track ([docs/IMPLEMENTATION.md:92](docs/IMPLEMENTATION.md));
  - button-stop and USB-pull checks on the robot are pending ([docs/PLAN.md:46-47](docs/PLAN.md));
  - ADR-021 notes it is unclear whether self-calibration at the venue counts as an "adjustment" ([docs/decisions/ADR-021-field-adaptive-two-run-design.md:34](docs/decisions/ADR-021-field-adaptive-two-run-design.md));
  - a deviation list (`ESP_ERROR_CHECK` aborts, `just lint` not scanning `src/esp32`) ([docs/IMPLEMENTATION.md:163-171](docs/IMPLEMENTATION.md)).

### Inferences
- Priority opportunities, in order of likely impact on a 2-run contest:
  1. Make run 1 reliable on corners, junctions and bricks. PLAN's fallback criterion is run 1 finishing without bricks.
  2. Restore a fast inner loop. Either a wheel-speed loop with feedforward, or a gyro yaw-rate loop under the IR PD. The owner removed the speed loop because of lag, so a yaw-rate loop at 500 Hz with feedforward might keep responsiveness.
  3. Implement `course.*`: ψ(s) logging, κ(s), forward/backward v(s) with a_lat and a_brake measured at home, and landmark re-anchoring. Persist it to flash.
  4. Make the bypass robust with ToF-guarded legs, a reacquisition window and a width measurement.
  5. Fix the IR front end: add the divider, consider shading or a skirt for sunlight.
- The repo itself acknowledges the slow τ ≈ 0.6 s motor response as a physical limit. Mass reduction or motor/gearing choices would matter for top speed.

### Gaps
- Which commits or flash images ran at the 2026-11-11 qualifier, and with what result, is not recorded.
- No measured a_lat, a_brake or tyre μ, and no measured maximum straight-line speed on the floor.
