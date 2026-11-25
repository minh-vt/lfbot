# Control Algorithms and Two-Run Racing Strategies of Top Line-Follower / Robotrace Robots

Scope note: research done 2026-11-16 with ~20 web searches/fetches. Japanese Robotrace (NTF All-Japan) sources were the richest for two-run strategy; Western sources (Pololu, RobotChallenge, Polish LF) were thinner and mostly hobby-grade. Many top Japanese teams publish only in tech-data sheets and short blog posts, so loop rates and gains are rarely stated. Treat individual blog numbers as self-reported.

## 1. Line position estimation (centroid, calibration, lost line, crossings, markers)

### Takeaway
The standard in competition and reference libraries is per-sensor min/max normalization followed by a weighted centroid; lost-line handling "remembers the last side". Markers and crossings are handled by dedicated side sensors plus simple debounce/flag logic. No source found that documented quadratic/sub-pixel fits or sensor preview distance quantitatively on a top robot.

### Cited Findings
- Pololu QTR `readLine()`: position = Σ(1000·i·v_i)/Σv_i, range 0..1000·(N−1), designed to be monotonic for PID; `calibrate()` tracks per-sensor min/max and `readCalibrated()` maps to 0..1000; recommended calibration ~5 s sweeping over light/dark; for analog sensors average 4 ADC samples per sensor. — [Pololu QTR library docs](https://www.pololu.com/docs/0J19/all)
- Lost line: Pololu `readLine()` keeps returning the extreme value on the side where the line was last seen (e.g. 4000 for a 5-sensor array lost to the right), so the controller keeps turning toward it. — [Pololu QTR library docs](https://www.pololu.com/docs/0J19/all)
- Pololu example PD: `motorSpeed = KP*error + KD*(error-lastError)`, starting gains KP≈0.1, KD≈5 (KD ≫ KP because the difference term is small); negative speeds clamped to 0. — [Pololu QTR library docs](https://www.pololu.com/docs/0J19/all)
- Japanese Robotrace hobby robot (Savoia S.21M, 2022): 10 line sensors, calibration in software by sweeping across the line in 1 mm steps, each sensor normalized by its own min/max and rescaled 0–999; error = weighted (right sum − left sum) with higher weight on outer sensors. — [MasA_T blog, 2022 tracer](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
- Sensor counts on top Japanese robots: Aegis2024 (All-Japan 2023 winner) 16 IR line sensors via ADC; mikage 2.9 (2024) 14 IR sensors; Impact (RobotChallenge 2012 winner) 14 analog KTIR0711S. — [NTF TechRT68 2023](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html); [NTF TechRT51 2024](https://www.ntf.or.jp/mouse/micromouse2024/Robot/AllJapan/TechRT51.html); [elektroda Impact thread](https://www.elektroda.com/rtvforum/topic2325660.html)
- Marker/crossing handling (Kumamoto KOSEN tracer course code): crossing flag clears pending corner-marker detection; corner marker confirmed after side-sensor count >3–5 samples then return to 0; start/goal confirmed after >10 samples; goal detection only armed for the last few expected markers; right-only marker events used to reject crossings. — [cpu4edu tracer software](https://cpu4edu.net/articles/26004.shtml)
- Lost-line safety in same code: if all central sensors read nothing → motors off + beep; if |position| exceeds a threshold (24–28 of ±35), base speed is cut. — [cpu4edu tracer software](https://cpu4edu.net/articles/26004.shtml)
- Savoia S.21M: a designated line sensor triggering on a crossing sets a flag that blocks the goal-marker sensor while set. — [MasA_T blog](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
- Japanese Robotrace rules: arc radius ≥10 cm and ≥10 cm between curvature change points (markers sit at curvature change points); goal line 1 m behind start line. — [NTF rule summary via search](https://www.ntf.or.jp/mouse/rt_kitei.html); [NTF 2008 rule PDF](https://www.ntf.or.jp/archives/micromouse2008/rt_rule2008.pdf)
- Mapping target angular velocity directly from a "sum of sensor value × constant" causes discontinuities between sensors and oscillation (Japanese dev diary note). — [usamimi linetrace diary index](http://www.usamimi.info/~isys/cgi-bin/adiary/adiary.cgi/linetrace/) (claim surfaced in search summary; not verified on the specific page)

### Inferences
- A continuous, monotonic estimate (normalized centroid) matters more than the exact estimator; discontinuity is what drives oscillation at high gain.
- Marker detection robustness relies on temporal debounce + context (crossing flag, expected marker count) rather than better sensing.

### Gaps
- No primary source found with quantitative comparison of centroid vs quadratic/parabolic interpolation on a competition robot.
- No source quantified sensor-bar preview distance (sensor ahead of axle) vs stability on a top robot. The `_BIRD` robot uses a VCM-actuated "sensor steer" (sensor bar moved laterally) but gives no control numbers. — [_BIRD 2019](https://underbirdworks.blogspot.com/2020/01/2019-bird.html)

## 2. Steering control (PD, curvature feedforward, cascaded yaw-rate loops, model-based)

### Takeaway
Serious Japanese tracers use a cascade: line error → target angular velocity (yaw rate) → wheel-speed/voltage FF+FB, with gyro yaw-rate feedback; the I term is usually dropped. Adding curvature feedforward (κ) to the command is the main documented improvement over pure PD. MPC/LQR is essentially absent from documented competition robots.

### Cited Findings
- Standard Japanese structure: control center velocity v and angular velocity ω separately, each with FF + PID FB; ω measured by gyro if present, otherwise (v_R−v_L)/tread, which becomes wrong under slip, so a gyro is recommended. Method described as taught by senior club members and shared with micromouse. — [usamimi linetrace diary #072](http://www.usamimi.info/~isys/cgi-bin/adiary/adiary.cgi/linetrace/072)
- Wheel command law: ω_wheel_R = [v_t + K_pv(v_t−v_meas)]/r + (L/2r)·[ω_t + K_pω(ω_t−ω_meas)]; left wheel flips sign of ω term. — [Zenn roboticram control 1](https://zenn.dev/roboticram/articles/94fbd2171bd1fc)
- A high-school tracer reported improved line tracking after adding angular-velocity feedback to front-sensor-only control (search snippet). — [Ogaki Technical HS report](https://school.gifu-net.ed.jp/ogaki-ths/course/07_it/it_HP/pdf/R1/202003.pdf) (PDF redirected; claim only from search snippet)
- PD without I is the norm: Savoia S.21M uses Kp·e + Kd·de/dt producing a speed difference (m/s), I omitted; author says P alone can run the course. — [MasA_T blog](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
- Pololu "Suckbot": PD, no I, steering by L/R power difference; 30:1 micro metal gearmotors, 6× QTR-1RC. — [Pololu blog Suckbot](https://www.pololu.com/blog/491/nathans-line-following-robot-suckbot)
- Curvature feedforward family (autonomous RC time-trial, not Robotrace but directly relevant): κ_cmd = −Kp·y_e (P); κ_cmd = −Kp(y_e + Kd·sinΨ_e) (PD); κ_cmd = −Kp(y_e + Kd·sinΨ_e) + κ (with path curvature FF); closed-form Micaelli & Samson law with (1−κ·y_e) terms was reported fastest around the track. — [a1k0n, Fast line-following robots (2018)](https://a1k0n.net/2018/11/13/fast-line-following.html)
- Shortcut-running tracers (Japan) follow a virtual path with the Kanayama tracking law: v, ω from pose error with gains K_x, K_y, K_θ; author warns large gains make the robot spin and recommends ramping from 0. Pure pursuit mentioned as the common alternative. — [tutuikento diary, shortcut running](https://tutui.hatenablog.com/entry/2024/12/20/000000)
- Peer-reviewed MPC/curvature-aware works exist (CA-MPCC ICRA 2023, two-layer MPC) but for general mobile robots/vehicles, not line-follower competitions; a 2025 thesis notes MPC computational cost vs. a curvature-preview controller. — [Peverali thesis, Polimi 2025](https://www.politesi.polimi.it/retrieve/2940fdf5-f861-4653-be1b-349290918ca4/2025_10_Peverali.pdf); [Liu two-layer MPC](https://ira.lib.polyu.edu.hk/bitstream/10397/109621/1/Liu_Two-Layer_Model_Predictive.pdf)

### Inferences
- For a robot that cannot see far ahead (IR bar), the useful "model-based" upgrade is: run-1 = reactive PD on line error producing ω_target; run-2 = ω_target = v·κ_map(s) feedforward + PD correction on line error. This is the a1k0n law with κ taken from the map.
- MPC/LQR on line followers is speculative for competition use; no evidence of winners using it.

### Gaps
- Gain values (Kp, Kd, K_pω) for top Japanese robots are not published in the sources found.
- No documented loop rate for the winners (Aegis/APOLLON). Hobby examples: 1 ms main-loop wait in Kumamoto KOSEN code ([cpu4edu](https://cpu4edu.net/articles/26004.shtml)); a commercial tuning guide cites a 2 ms (~500 Hz) PID loop ([Zbotic tuning guide](https://zbotic.in/pid-line-follower-robot-tuning-speed-competition/), low-authority). 1 kHz is the de facto Japanese micromouse/tracer convention but I found no primary citation for a winning robot.

## 3. Wheel speed control (encoders, motor feedforward, traction limits)

### Takeaway
Japanese tracers use voltage feedforward from a DC-motor model (back-EMF·ω + R·I from target acceleration + friction) normalized by measured battery voltage, plus P(I) feedback on encoder speed. Traction (μ·g) bounds acceleration; suction is what lifts it.

### Cited Findings
- Motor voltage FF: V = K_E·η·G·ω + C_V·a + C_μ, with C_V = R·m·r/(2·G·K_T); duty = V/V_batt (battery measured by ADC); example η_M=0.7, η_G=0.95. Friction term C_μ is hard to identify. — [Zenn roboticram control 1](https://zenn.dev/roboticram/articles/94fbd2171bd1fc)
- Simpler empirical FF: V = K_A·a + K_V·v, K_A and K_V identified from voltage-vs-acceleration and voltage-vs-speed tests. — [usamimi #072](http://www.usamimi.info/~isys/cgi-bin/adiary/adiary.cgi/linetrace/072)
- Straight-line FF table design: m=0.1 kg robot, 2× Maxon DCX10, wheel r=6.3 mm; without downforce friction limit F=μmg gives a_max = 9.8 m/s² (μ=1); voltage limited to ±9 V; discrete RL-current model inverted to compute voltage per step. Author estimates 20 m from rest in ~2 s. — [Qiita: robotracer straight acceleration FF](https://qiita.com/controller/items/92e409850db1d311df61)
- Hardware of the winners: Aegis2024 2× Maxon DCX10L + Maxon ENX10 EASY optical encoders, LSM6DSR gyro, RX66T 160 MHz, 3S LiPo; mikage 2.9 Faulhaber 1717 + IE2H-4096 magnetic encoders, ADXRS620 gyro, 160 g. — [NTF TechRT68](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html); [NTF TechRT51](https://www.ntf.or.jp/mouse/micromouse2024/Robot/AllJapan/TechRT51.html)
- Practical traction issue: 8 mm tires on a 700 g robot slipped on R10 curves; fixed with wider tires. — [MasA_T blog](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
- Acceleration limited by wheelie (center of mass) on a high-school tracer, which also used 2000 mm/s max / 800 mm/s min speed settings. — [Ogaki THS report (snippet)](https://school.gifu-net.ed.jp/ogaki-ths/course/07_it/it_HP/pdf/R1/202003.pdf)

### Inferences
- For a robot without suction, plan a_long and a_lat well below 1 g (μ for rubber on the field is unknown; must be measured).
- Battery-voltage normalization of duty is a cheap, standard FF piece.

### Gaps
- No measured μ or lateral-g data for top robots found; no source on explicit slip detection (encoder vs gyro mismatch thresholds) in competition code.

## 4. Two-run / mapping strategies (Robotrace)

### Takeaway
Evolution in Japan: (a) marker-segmented section log + per-section speed (classic); (b) continuous distance-indexed curvature log + curvature-to-speed with forward/backward acceleration limits; (c) since ~2022–2023 the winners record a 2D XY map in run 1, optimize a shortcut path, and in later runs track a "virtual line" via dead-reckoning (encoder + gyro), with marker-distance correction. Winning times dropped to ~4.5 s on a ~36 m course (~8 m/s average).

### Cited Findings
- Classic section method: record cumulative encoder distance at each marker in run 1; classify section as straight/curve by peak gyro ω sampled at the section midpoint (because line sensors lead the marker sensor); straights: accelerate to v_max, cruise, decelerate to v_min at the next curve; curves at their allowed max speed. S.21M: v_max 2.9 m/s, a 4 m/s². — [MasA_T blog](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
  - Note: that blog's no-cruise formula v_max = √(2ax)+v_min is not the kinematic form; correct is v_peak² = v_min² + a·x for symmetric accel/decel over distance x (or v² = v0² + 2a·d per ramp). — derived, same source
- Distance correction: during run 2 snap odometry to the run-1 marker distance on each marker; only markers bounding curves R≥15 cm or straights ≥20 cm are used because short/gentle sections give false marker detections; uncorrected slip causes overshoot at higher speed. — [MasA_T blog](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
- Gyro bias: stationary averaging after startup transient reduced drift from −9.3°/min to −0.0053°/min (MPU-6050). — [MasA_T blog](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
- Encoder-only radius logging: section straight if |ΔR−ΔL|/(ΔR+ΔL) < 0.1; logs cDis[]/cRad[]; correction phases can be misclassified as straights; braking starts when remaining distance < f(speed difference). — [cpu4edu tracer software](https://cpu4edu.net/articles/26004.shtml)
- Curvature smoothing for course evaluation: curvature ρ(l) along distance changes abruptly, so a smoothed ρ_s(l) is used instead of raw constant-speed gyro log ω(l). — [Shachiku Hakase note (search snippet; page 404 at fetch)](https://note.com/shachiku_hakase/n/n9f9cf622a349)
- Curvature → speed: v = min(√(a_lat,max/|κ|), v_max); for |κ| < a_lat,max/v_max² use v_max; take the max curvature within a look-ahead window to brake before turns; a_lat,max calibrated by driving circles and comparing accelerometer vs v·ω. — [a1k0n](https://a1k0n.net/2018/11/13/fast-line-following.html). A ROS2 implementation adds safety factor 0.995 and reports that using single-point curvature (not window max) let measured lateral accel exceed the limit. — [OpenHUTB ros2 PR #202](https://github.com/OpenHUTB/ros2/pull/202) (low-authority, Chinese PR discussion)
- Speed profile shapes used: trapezoidal (constant accel), minimum-jerk, sinusoidal S-curve. — [学ロボの制御 slides](https://www.slideshare.net/slideshow/ss-126596414/126596414)
- Winners' strategy: Aegis2024 (Fujisawa, All-Japan 2023, 4.846 s): after run 1, builds a 2D map, computes the shortest path over the whole course by mathematical optimization, follows it as a virtual line; custom vibration-damping fan mount to reduce gyro noise for reliability; 5 m/s max, 10 m/s² accel, 1 m/s turning speed (registration data). — [NTF TechRT68 2023](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html)
- APOLLON (same entrant) won 2024 with 4.489 s; APOLLON 2025 reportedly 4.511 s on a 35.9 m course (avg ≈7.95 m/s) per a personal blog. — [NTF 2024 results](https://www.ntf.or.jp/mouse/micromouse2024/AllJapan/recode.html); [ak-rcroom Robotrace 2025](http://ak-rcroom.com/sakuhin/robotrace/robotrace_2025/robotrace_2025/) (blog; not verified against official record — TLS error at fetch)
- Shortcut running became mainstream among several entrants by 2024 (blog); 2022 tech data notes self-localization by wheel rotation + gyro accumulates large error. — [ak-rcroom RT2024](http://ak-rcroom.com/sakuhin/robotrace/robotrace_2024/robotrace_2024/); [NTF TechRT61 2022](https://www.ntf.or.jp/mouse/micromouse2022/AllJapan/robots/TechRT61.html) (both via search summary)
- Shortcut path generation (hobby): moving average over the logged x and y sequences to straighten wiggles; Kanayama tracking; failures seen: failsafe not triggering → robot spun, impact bent gyro, leaving the course at high speed. — [tutuikento diary](https://tutui.hatenablog.com/entry/2024/12/20/000000)
- Odometry from step counts + IMU heading: Δl = (Δs_R+Δs_L)/2·k, θ from trapezoidal gyro integration, x += Δl·sinθ, y += Δl·cosθ. — [MasA_T blog](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
- Western equivalent: Berkeley EE192 — first lap maps the track at max cornering speed, second lap plans using the map. — [EE192 Dis13](https://inst.eecs.berkeley.edu/~ee192/sp20/files/Dis13.pdf) (via search summary). Open-source example: StepTracer_Ray stores `search_info[]` to ROM, `fastrun.c` does second-run speed plan. — [StepTracer_Ray](https://github.yxlr123.workers.dev/thdtjdgur/StepTracer_Ray) (mirror URL; low-authority)

### Inferences
- Recommended pipeline for a 2-run contest (synthesis, not a single source): log (s, ω_gyro, v) at fixed distance step in run 1 → κ(s)=ω/v, smooth → v_lim(s)=min(v_max, √(a_lat/|κ|)) with window max → backward pass with a_dec, forward pass with a_acc → run 2 track v(s) with ω_ff = v·κ(s) + line-PD, correcting s at reliable landmarks (markers/crossings) and slowing if line error or landmark mismatch exceeds bounds.
- Full shortcut (virtual line) is only legal/useful where rules allow leaving the line; F-Maze-Solving rules must be checked before borrowing this.

### Gaps
- No source found describing cross-correlation/DTW-style localization of the run-2 trace against the run-1 log in Robotrace; marker snapping is the only documented method.
- Log sampling distance (e.g. every 10 mm) for winners not found.

## 5. Safety and robustness

### Takeaway
Documented practice is basic: speed cut when line error is large, motor stop when line is fully lost, marker filtering by plausibility, and multiple attempts with fallback. Little published on formal slip detection or adaptive escalation.

### Cited Findings
- Large error → reduce base speed; all sensors lost → stop + beep. — [cpu4edu](https://cpu4edu.net/articles/26004.shtml)
- Only "reliable" markers used for distance correction; false detections on short/gentle sections are a known failure. — [MasA_T blog](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
- Failsafe failure in shortcut mode leads to spinning/leaving course; gyro damage from impact. — [tutuikento diary](https://tutui.hatenablog.com/entry/2024/12/20/000000)
- Robotrace allows multiple attempts; _BIRD (2019 nationals) changed its run strategy and failed to finish runs 2 and 3 (27.712 s best, 32nd), illustrating risk of aggressive second runs. — [_BIRD 2019](https://underbirdworks.blogspot.com/2020/01/2019-bird.html)
- Gyro noise from the suction fan is a real reliability issue (Aegis uses a vibration-damping fan mount). — [NTF TechRT68](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html)
- Advice to raise max speed gradually and retune P/D each step (hobby guide, low authority). — [Pololu forum thread](https://forum.pololu.com/t/fast-line-follower-build/11956) (via search summary)

### Inferences
- Escalating speed scale between attempts (e.g. 70% → 85% → 100% of planned a_lat) is a reasonable design but speculative — no primary source documented a specific schedule.
- Slip can be detected by comparing encoder-derived ω with gyro ω and encoder-derived accel with IMU accel; speculative for this domain (no competition source).

### Gaps
- No published quantitative data on line-loss recovery maneuvers on top robots.

## 6. Breakthroughs and their quantified impact

### Takeaway
Suction/downforce + gyro/encoder course memory + (since ~2023) shortcut path optimization are the step changes. Quantified data points: suction robots list 10–20 m/s² acceleration vs ~4 m/s² for a non-suction hobby robot; All-Japan winning times ~4.5–4.8 s on ~36 m courses vs ~13–28 s for mid-field line-tracking robots.

### Cited Findings
- Suction robots' registration data: mikage 2.9 — 5 m/s, 20 m/s², 2 m/s turning, 160 g, 4WD, suction fan (but official 28.369 s, 24th in 2024); Aegis2024 — 5 m/s, 10 m/s², 1 m/s turning, winner 4.846 s. — [NTF TechRT51](https://www.ntf.or.jp/mouse/micromouse2024/Robot/AllJapan/TechRT51.html); [NTF TechRT68](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html)
  - Note: "turning speed" figures (1–2 m/s) look low against ~8 m/s average lap speed; registration data may be stale (NTF itself warns it can differ from contest-day robot).
- Non-suction comparison: S.21M 2.9 m/s, 4 m/s² ([MasA_T](https://masa-t.hateblo.jp/entry/2025/12/17/000100)); _BIRD 9 m/s theoretical, ~10 m/s² approx., no suction mentioned, 200 g ([_BIRD](https://underbirdworks.blogspot.com/2020/01/2019-bird.html)).
- Without downforce, friction caps longitudinal accel at μg ≈ 9.8 m/s² for μ=1. — [Qiita FF](https://qiita.com/controller/items/92e409850db1d311df61)
- Impact (RobotChallenge 2012 winner, 105 g): reversed EDF turbine (~11,000 RPM, ~4 A) for downforce to hold corners above ~2 m/s; average route speed 2.3–2.5 m/s, theoretical 3 m/s; STM32F103. — [elektroda Impact](https://www.elektroda.com/rtvforum/topic2325660.html)
- Counterexample: Pololu Suckbot (300 g, fan lifting ~140 g, 1.3 mm gap) averaged 0.93 m/s and the author thought it would have run as well without the fan at those speeds — suction helps only when the controller/speed exploits it. — [Pololu Suckbot](https://www.pololu.com/blog/491/nathans-line-following-robot-suckbot)
- Semreh (Brazil, ESP32, 8× QRE1113): added 4× 1103 brushless fans for downforce; 2nd IronCup 2024, 3rd RSM 2024; brushless downforce motors caused problems at RCX 2025; 1st place after a software update to motor/ESC control. No speed numbers. — [hackaday Semreh](https://hackaday.io/project/202208-semreh-advanced-line-follower-robot)
- Gyro: replaces slip-prone encoder ω, enabling yaw-rate loops and course logging; bias calibration critical (−9.3°/min → −0.0053°/min). — [usamimi #072](http://www.usamimi.info/~isys/cgi-bin/adiary/adiary.cgi/linetrace/072); [MasA_T](https://masa-t.hateblo.jp/entry/2025/12/17/000100)
- Shortcut path running (2D map + optimization + virtual line) is what distinguishes the 2023–2025 winners. — [NTF TechRT68](https://www.ntf.or.jp/mouse/micromouse2023/Robot/AllJapan/TechRT68.html)

### Inferences
- For a robot without suction and without known μ, the biggest gains available are (1) gyro yaw-rate inner loop, (2) curvature-indexed speed profile with forward/backward passes, (3) curvature FF in run 2; suction is a hardware decision outside a software rewrite.

### Gaps
- No controlled A/B (same robot, fan on/off) lap-time data found.
- No official loop-rate numbers for the winners; no data for Polish/European top LF robots beyond Impact (2012).
