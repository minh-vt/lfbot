# Obstacle avoidance and tricky elements in line-following competitions

Scope note: these notes are based on 4 primary rulebooks (Robotex 2026, RobotChallenge 2025, Lodz Sumo Challenge, RCJ Rescue Line 2025), one fully read world-champion RCJ code base and journal (Overengineering², 1st at RCJ 2024 Eindhoven), the Pololu 3pi wall-follower documentation, and this project's own rules (docs/manual.md, docs/fqa.csv). Public write-ups from speed-oriented "Line Follower Enhanced" teams turned out to be very sparse; see the Gaps sections.

## Rules context: what the obstacle and other elements actually look like

### Takeaway
The rulebooks all describe a single cuboid obstacle (a brick or wall) placed on a straight-ish section, plus a rejoin window of 20–50 cm. Several also guarantee a straight line after the obstacle and a minimum distance from field edges. This project's rules differ in ways that matter. Knocking over or dragging the obstacle is a DNF, and so is crossing a different line segment ("cắt line"), so a wide, sloppy bypass is directly penalised.

### Cited Findings
- **Robotex (2026 rules):** the obstacle or wall is a cuboid of max 25 x 15 x 10 cm, "not white", and may be any material. The robot must at least partly cover the line again within 30 cm after the obstacle. Hitting the obstacle is allowed but discouraged. After the obstacle there are at least 20 cm of uninterrupted line, and the robot must follow it. — [Robotex Line Following Enhanced 2026 PDF §7.2](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf)
- **Robotex, other elements:**
  - Line breaks of at most 10 cm, with at least 10 cm of unbroken line before a curve. Consecutive breaks are separated by at least 2 cm of line.
  - 25 cm of free space on both sides of the line, except at crossings and road splits. At crossings the robot must go straight.
  - Line width varies from 5 to 30 mm.
  - Road split: the shorter branch is marked by a 15x30 mm side marker at least 100 mm before the split, and the split angle is 30–60°.
  - Speed-limit zone (<0.5 m/s) marked by 15x60 mm side bars.
  - Driving off the track fails the attempt. Up to 5 rounds, plus a 5-minute finals window with unlimited attempts.
  - Source: [same PDF §4–7](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf)
- **RobotChallenge (rules revised Sept 2025):** "Line Follower Enhanced" uses a brick of about 25 cm long x 12 cm wide x 6.5 cm high. The robot must "detour around this brick and immediately return to the line". Leaving the line during the obstacle is allowed, but not returning to the intended path afterwards counts as off-line. There is a 10 cm gap, never on a curve, and the line continues straight after it. 3 attempts on a closed track; best time counts. — [RobotChallenge Line Follower rules PDF §3.2–3.4](https://www.robotchallenge.org.cn/doc/en/RC-LineFollower.pdf)
- **Lodz Sumo Challenge LF Enhanced:**
  - The wall is about 25 cm long, 6.5 cm wide and 12 cm high, and touching it is allowed.
  - "The route is constructed so that the robot can drive around the obstacle on its right side." The robot must be back on the route within 40 cm behind the obstacle.
  - Line breaks are at most 10 cm and never on curves; sequences of breaks are possible. Intersections are passed straight.
  - Other elements: inverted colours (up to 0.5 m of white line on black), a double line about 20 mm apart, forks with different obstacles on each branch, a bridge/tunnel, and changes in line width (19 mm → 38/10 mm).
  - Organisers choose the obstacles on competition day. Finals give 4 attempts.
  - Source: [Lodz LF rules PDF §2–3](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf)
- **Conflict on brick orientation:** RobotChallenge lists the brick as 12 cm wide x 6.5 cm high, which is a brick lying flat. Lodz lists 6.5 cm wide x 12 cm high, which is a brick on edge. The same physical brick appears in two orientations, so detection height and bypass width both change. — [RobotChallenge](https://www.robotchallenge.org.cn/doc/en/RC-LineFollower.pdf) vs [Lodz](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf)
- **RCJ Rescue Line 2025:**
  - Obstacles are bricks, blocks, weights or other heavy items at least 15 cm high, and may be fixed to the floor. Robots may move them, but a moved obstacle stays where it ends up "even if that prevents the robot from proceeding".
  - An obstacle occupies at most one line or tile and is never closer than 25 cm to the field edge or to inclined tiles. Each overcome obstacle scores 20 points.
  - Gaps are at most 20 cm, with at least 5 cm of straight line before them.
  - Intersections are always perpendicular and marked by 25x25 mm green markers placed just before them. With no marker the robot goes straight. Two markers mean a dead end, and the robot turns around.
  - The line is at least 10 cm from walls and from obstacles that are not on the robot's path.
  - Losing the line until the next tile, or reaching a line out of sequence, counts as a Lack of Progress.
  - Source: [RCJ Rescue Line 2025 rules draft §3.3–3.6, 5.5](https://junior.robocup.org/wp-content/uploads/2025/01/RCJRescueLine2025-draft-1.pdf)
- **This project (F-Maze-Solving):**
  - manual.md Điều 4.4 gives random obstacles of 10.5 x 6 cm, cuboids lying or standing.
  - FQA Q-13 overrides this with a brick of 205 x 95 x 95 mm, placed centred on the line at random positions chosen on competition day.
  - FQA Q-14: a light brush that slightly moves the obstacle is OK. Knocking it over or pushing/dragging it away is a DNF.
  - FQA Q-15: off-line time is measured by sensors. Leaving the line and then cutting across another line segment is a DNF.
  - Two runs per round; the best valid time counts.
  - Source: local `docs/manual.md`, `docs/fqa.csv` (Q-13, Q-14, Q-15)

### Inferences
- Many rulebooks guarantee a straight line after the obstacle (Robotex: 20 cm; RobotChallenge: immediately). This project's rules give no such guarantee, and the obstacle position is random. The bypass therefore has to cope with an obstacle near a curve, and the project manual and FQA give no geometric guarantees at all.
- The size conflict between manual.md (10.5 x 6 cm) and FQA Q-13 (20.5 x 9.5 x 9.5 cm) should be stated, not resolved silently. The bypass envelope should cover the larger one, and a robot that measures the obstacle (see below) need not assume either.
- Contact rules differ. Robotex and Lodz allow contact; this project tolerates a light brush but disqualifies on toppling or dragging. A "bump and push" strategy (e.g., [Arduino RobotRescue example](https://docs.arduino.cc/library-examples/robot-control-library/RobotRescue)) is therefore unacceptable here.

### Gaps
- Exact current RobotChallenge obstacle placement rules (distance to curves, side) are not stated in the PDF.
- No rulebook I read specifies obstacle colour or reflectivity, apart from Robotex's "not white". This matters for IR ToF/triangulation sensors.

## Detection: sensors, placement, distance, false positives, filtering

### Takeaway
The best-documented top team (Overengineering², RCJ 2024 world champion) tested camera, pressure switch, IR and ultrasonic options and chose three front IR distance sensors. The two outer sensors are angled inward so the robot can tell how it is oriented relative to the obstacle. Detection uses windowed time-averages, banded thresholds (min and max), a cooldown after other manoeuvres, and suppression on ramps. Their main false-positive problem came from the robot's own front panel, not from spectators.

### Cited Findings
- Overengineering² TDP: "After testing multiple methods, we determined that the most accurate one is infrared sensor based. We have 3 infrared sensors in the front of our robot, of which the outermost 2 are tilted inward, giving us the ability to tell how we stand in front of an obstacle." The rejected alternatives were camera, pressure switch and ultrasonic. — [Overengineering² repo, documents/documentation/Team Description Paper.pdf](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup)
- Their sensors were six Pololu IR distance sensors (irs17a) and one irsl16a, two on each side and two at the front for obstacle detection, plus two BNO055 IMUs. — [Overengineering² README / Engineering Journal](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup)
- **False positive from their own chassis:** "Sometimes very small distance values are output by the distance sensors in the front, even though there is nothing in front of them." The cause was a part of the front panel close to the sensor's field of view. The fix was to space the sensors further out with nuts, which "solved this problem, which we had been struggling with for a long time". Separately, a missing resistor on one sensor caused intermittent false detections. — [Engineering Journal pp. 48, 93](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf)
- **Detection predicate in their code** (`control.py`, `obstacle_detected()`):
  - Each sensor is averaged over a 0.25 s window, and an obstacle is flagged if any sensor falls inside a band, e.g. `20 < centre < 75` or `20 < side < 55` (raw units, likely mm).
  - The lower bound of 20 rejects zero or glitch readings.
  - Detection is gated by `obstacle_detect_cooldown`, which is set to 1.5 s after the seesaw manoeuvre, and is disabled on ramps unless a flag is set.
  - A separate, wider-band `obstacle_detected_again()` is used for re-checks.
  - Source: [robot_v.3/Python/main/control.py](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/robot_v.3/Python/main/control.py)
- **Pololu 3pi wall follower:**
  - It uses Sharp GP2Y0A21 analog sensors tilted slightly upward "so that they do not see the floor if the robot tips forward".
  - The sensors have a minimum range; once an obstacle is closer, the sensor "no longer detects that obstacle", and the robot can get stuck.
  - Suggested fixes: mount the sensors further back, or use the GP2Y0D810 (10 cm digital) sensor. A ≥10 µF capacitor at each sensor reduces supply noise from the sensors' current bursts.
  - Source: [Pololu 3pi Wall Follower docs](https://www.pololu.com/docs/0J26/all)
- An RCJ beginner guide (NSW Riley Rover) uses a front touch sensor or ultrasonic sensor: detect, reverse, navigate around, and reacquire within 30 cm. — [RoboCup NSW Riley Rover steps](https://sites.google.com/education.nsw.gov.au/robocupresources/line-rescue/riley-rover-steps) (search-snippet level; page not fully read)

### Inferences
- With a 95 mm-tall brick in this project, a forward sensor mounted low (roughly 20–60 mm above the floor) and tilted slightly up sees the brick in both orientations and should not see the floor. A 60 mm tall obstacle (the manual.md variant) would need the beam below 60 mm. This is my geometric reasoning, not a sourced figure.
- A banded threshold (ignore very small and very large readings) plus a short time-average is the documented filtering. The upper band limits the range, so the robot ignores walls and spectators beyond about 7.5 cm (Overengineering² tuning). For a faster robot the upper band must grow with speed: braking distance plus the time for the manoeuvre to start.
- Spectators and walls are mostly handled by placement and range limits in practice. RCJ guarantees 10 cm from line to walls and 25 cm from obstacle to edge, so a short-range sensor (<15 cm) naturally excludes them.

### Gaps
- I found no published detection distances or speeds for fast "LF Enhanced" robots (Robotex, RobotChallenge). No team blog or paper with numbers was located.
- I found no source quantifying ultrasonic versus IR ToF performance on dark or angled bricks in this context.

## Bypass manoeuvres: open-loop vs gyro/odometry vs wall-following; side choice; size assumptions; curves

### Takeaway
Documented practice, even from the world champion, is mostly open-loop timed motion plus sensor-based alignment. The robot centres on the obstacle with the front IR sensors and regulates its standoff distance into a 75–90 band. It then rotates until the centre sensor no longer sees the obstacle, and drives a fixed-radius curve for a set time until the line camera sees the line. Side choice is a predefined list (alternating L/R per obstacle count). The documented fix for overshoot was to insert short in-place turns. Wall-following with a side sensor is a known general technique, but I found no competition team documenting it for line obstacles.

### Cited Findings
- **Overengineering² journal (2024-01-16):** "Added orientation in front of any obstacle by turning either left or right until the obstacle is no longer detected by the center distance sensor. With this orientation, we can turn for another 0.3 seconds and then start driving a curve around the obstacle, which works with obstacles of any width now." — [Engineering Journal p. 93](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf)
- **Overengineering² journal (Hanover):** while circling in a curve, "the robot often drives too far behind the obstacle, where there no longer needs to be a line". The fix: "added a turn on the spot at a time interval of 0.3 seconds and then continue on the curve". — [Engineering Journal](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf)
- **Code-level sequence** (`turn_for_obstacle()` and states `obstacle_detected` → `obstacle_avoid` → `obstacle_orientate`), from [control.py](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/robot_v.3/Python/main/control.py):
  1. Back up briefly (0.15 s).
  2. If the centre sensor does not see the obstacle, rotate toward the nearer side until it does (5 s timeout).
  3. Regulate the centre distance into the 75–90 band (3 s timeout).
  4. Pick a direction from `obstacle_dir = ["l","r"]` indexed by `obstacle_count`. The side alternates per obstacle; this is a fixed policy and does not depend on sensing.
  5. Rotate until the centre sensor reads beyond 280 or nothing, then rotate 0.3 s more and drive straight 0.45 s.
  6. In `obstacle_avoid`, drive a max-steer arc for 1.6 s (3.5 s on ramp up, 1.0 s ramp down). Then repeatedly do a 0.1 s in-place correction toward the obstacle followed by 0.7 s timed arcs.
  7. If any step times out, `return_after_failed_obstacle()` turns back to the gyro heading recorded at detection and resumes line following.
  8. If image similarity stays above 0.88 for 15 frames while avoiding (the robot is stuck), it backs off.
  - Ramp variants use different timings.
- They used a gyro (two BNO055 averaged, because a single one "would shift randomly by around 5°") only for heading-return on failure and for turns. The arc itself is timed. — [Engineering Journal p. 36](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf); [control.py](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/robot_v.3/Python/main/control.py)
- **Simple tutorial pattern** (Microline Line Robot, RCJ exercise "Around obstacle"): stop when the front distance falls below a threshold, rotate in place 500 ms, drive a circular arc, then loop until the line is found. — search snippet of [Microline exercise](https://ns.microline.hr/HTML.ashx?id=350) (page fetch failed; content seen only via search summary)
- **Wall-following as a general technique:** the Pololu 3pi uses a PD law on a side Sharp sensor (`pd = P/3 + D*20` around a set point) and turns in place when the front sensor exceeds a threshold. — [Pololu 3pi Wall Follower](https://www.pololu.com/docs/0J26/all)
- **Side choice in the rules:** Lodz guarantees the right side is passable. RCJ guarantees 25 cm clearance from field edges but not which side is free. — [Lodz rules](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf); [RCJ 2025](https://junior.robocup.org/wp-content/uploads/2025/01/RCJRescueLine2025-draft-1.pdf)

### Inferences
- **Speculation:** wall-following the obstacle with a side distance sensor (closing the loop on the obstacle face) removes the size assumption. Combined with gyro heading, it would also bound the total turn: exit when the heading has turned about 180° relative to entry or the line is seen. This would replace the champion's timed arcs. It is untested here and comes from no competition source.
- A timed arc's radius depends on battery voltage, floor friction and robot mass. That explains why Overengineering² needed separate timings for ramps and an in-place correction hack. On an ESP32 robot with encoders, an arc closed on odometry (arc length and heading) is the direct fix. I found no team source confirming they did this.
- For this project, open-loop overshoot is doubly dangerous. A wide arc can cross another line segment (DNF per FQA Q-15), and a tight arc can drag the brick (DNF per Q-14). So the bypass radius needs a measured lower bound (clearance) and a bounded upper excursion.
- **Obstacle near a curve** (this project gives no straight-line guarantee): the rejoin may occur on a curve segment heading in a different direction, or the robot may meet a different line segment first. A heading-aware rejoin (accept only line hits whose expected heading is within some window of the pre-obstacle heading plus the planned turn) is a reasonable mitigation, but it is speculative.

### Gaps
- I found no documented use by a competition team of encoder or gyro-closed arcs for the bypass itself, and no side-sensor wall-follow around a line obstacle.
- I found no data on how fast LF Enhanced (speed-class) robots bypass, or how long they take.
- I found no source on choosing the side dynamically by sensing which side is free (for example with a sweeping or angled sensor), apart from the champion's "nearer side" centring step.

## Rejoin: re-detecting the line, angled crossing, false reacquisition, heading alignment

### Takeaway
The champion team documents three techniques:
1. A cooldown of about 0.45 s after the manoeuvre starts before any line detection counts.
2. A raised "minimum line size" threshold during the bypass (6500 px versus 3000 px normally), which rejects small fragments.
3. A dedicated orientate-after-obstacle routine: nudge forward, turn back, drive, counter-turn until the line is seen (3 s timeout), then continue.

For gaps they stop and re-square to the line angle using the camera, because gyro-angle turning failed on 45° gaps.

### Cited Findings
- **Rejoin code**, from [control.py](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/robot_v.3/Python/main/control.py):
  - On entering `obstacle_avoid`: `min_line_size.value = 6500` and `timer.set_timer("obstacke_cooldown", .45)`.
  - The state changes only when `line_detected and cooldown expired`. Then `min_line_size` is reset to 3000 and `orientate_after_obstacle(dir)` runs:
    1. Forward 0.1 s.
    2. Turn opposite to the bypass 0.4 s.
    3. Forward 0.75 s.
    4. Turn back, with a 3 s timeout, until the line is detected.
    5. Continue 0.35 s.
    6. If the line is lost, turn back.
  - The side distance-sensor buffers are then filled with "far" values (300) so the obstacle is not re-detected immediately.
- **Gap rejoin (journal 2024-01-09):** they detect a gap when the black area is below a threshold. They compute a "gap angle" from the min-area rectangle of the remaining line segment and reposition until that angle is 0°. In their words, the robot used to "turn based on its angle from the gyro sensor, but this approach won't work with gaps at 45°". — [Engineering Journal p. 86](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf)
- **Rejoin windows in the rules:** Robotex requires partial line coverage within 30 cm; Lodz requires being back within 40 cm; RobotChallenge says "immediately"; the RCJ beginner guide uses 30 cm. — [Robotex](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf); [Lodz](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf); [RobotChallenge](https://www.robotchallenge.org.cn/doc/en/RC-LineFollower.pdf); [NSW guide](https://sites.google.com/education.nsw.gov.au/robocupresources/line-rescue/riley-rover-steps)
- **RCJ out-of-sequence rule:** "a robot reaches a line that is not in the intended sequence" is a Lack of Progress. False reacquisition on the wrong segment is penalised explicitly. — [RCJ 2025 §5.5](https://junior.robocup.org/wp-content/uploads/2025/01/RCJRescueLine2025-draft-1.pdf)

### Inferences
- With an IR reflectance array (this project) instead of a camera, the equivalents of "min line size" are the ones `line_sensing` already reports: the cluster count and cluster width of the classification. While crossing the line at an angle, a single narrow cluster appears briefly and moves across the array. Requiring a minimum number of samples or a minimum travel while the line is seen, plus a cooldown in distance (not time), is the analogue. This is speculative.
- Crossing the line at an angle: the champion's approach is to overshoot slightly, then counter-turn until the line is centred. Given an IMU or odometry, an alternative is to drive until the array detects the line, record the heading, and rotate toward the pre-obstacle heading (or the curve-predicted heading) while keeping the line in the array. The champion did this for gaps with the camera; for the IR array it is inference.
- Re-detection of the same obstacle immediately after rejoining is a documented risk. They flush distance buffers to "far" and use cooldowns.

### Gaps
- I found no source quantifying false-reacquisition rates or rejoin angles.

## Mapping / two-run robots and obstacles

### Takeaway
I found no documented case of a two-run or mapping line follower that records obstacle positions in run 1 and plans a high-speed bypass in run 2. In the speed classes I read (Robotex, RobotChallenge, Lodz), runs are repeated attempts on a fixed track, often with unlimited practice in prelims. Any "learning" across attempts is presumably done by humans retuning between attempts. That is an inference; there is no source.

### Cited Findings
- Robotex: up to 5 rounds of one attempt each, and a 5-minute finals window with unlimited attempts. — [Robotex 2026 §6](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf)
- Lodz: unlimited attempts in prelims and 4 in finals; organisers choose obstacles on the day. — [Lodz rules](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf)
- RCJ: course knowledge from humans is forbidden during runs. The rules line about humans providing the robot "information about the field (e.g., location of obstacles...)" appears in the 2025 draft. — [RCJ 2025 draft](https://junior.robocup.org/wp-content/uploads/2025/01/RCJRescueLine2025-draft-1.pdf)
- This project: 2 runs per round, best valid time counts. No code reload or tuning after the track is revealed. — local `docs/manual.md` Điều 4.7, 5.1

### Inferences
- **Speculation:** for this project, a run-1 map could store the obstacle's along-track position (odometry distance since start or since the last landmark) and its measured width and side clearance. Run 2 could then start the bypass early at a planned speed instead of braking to a standoff distance. The risk is that the obstacle is not exactly where it was; FQA Q-14 says light brushes can move it slightly. So run 2 must still confirm with the range sensor before committing, and fall back to the run-1 behaviour.
- The run-2 planner must still respect "do not cut another line". The planned arc geometry must be checked against the mapped line geometry near the obstacle.

### Gaps
- I found no source documenting run-2 obstacle planning in any competition. This is the largest evidence gap.

## Failure modes, mitigations and testing practices

### Takeaway
Documented failure modes:
- false obstacle detection from the robot's own structure
- sensor hardware faults
- gyro drift (about 5°)
- overshooting behind the obstacle with timed arcs
- losing the line at angled gaps
- getting stuck
- obstacles on ramps
- minimum-range blind spots of IR rangers

Mitigations: mechanical relocation of the sensors, dual IMUs averaged, in-place correction turns, camera-based re-squaring, image-similarity stuck detection, timeouts with a heading-return fallback, and separate parameter sets for ramps. Testing is iterative on a physical practice course, logged in a dated engineering journal.

### Cited Findings
- **Self-detection of the front panel**, solved by spacing the sensors outward; a broken resistor on one IR sensor. — [Engineering Journal pp. 48, 93](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf)
- **Gyro shift of about 5°**, mitigated by adding a second BNO055 and averaging. — [Engineering Journal p. 36](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf)
- **Overshoot behind the obstacle**, fixed with periodic 0.3 s in-place turns during the arc. — [Engineering Journal (Hanover entry)](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf)
- **Stuck detection** via image structural similarity and gyro, which "is able to unstuck itself almost every time". Their final world-championship run "went poorly because of a obstacle in the evacuation zone". — [Engineering Journal pp. 112–113, conclusion](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf)
- **Every manoeuvre loop has a timeout** (3–5 s) and a fallback `return_after_failed_obstacle(start_angle)` that turns back to the heading recorded at detection. — [control.py](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/robot_v.3/Python/main/control.py)
- **IR minimum-range blind spot** can cause the robot to get stuck; mitigate with mounting position or a shorter-range sensor. — [Pololu 3pi Wall Follower](https://www.pololu.com/docs/0J26/all)
- **Testing practice:** a dated engineering journal recording "Issues and solutions" per day, practice-course runs, and a simulator. One team claims their world-championship-winning code was 90% written in a simulation (author's own claim, not verified). — [Overengineering² EJ](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/documents/documentation/Engineering%20Journal.pdf); [lucaSartore/Robocup-Rescue-Line-simulation](https://github.com/lucaSartore/Robocup-Rescue-Line-simulation)

### Inferences
- For this project, the failure modes map onto the existing architecture rules:
  - Timed arcs are field-dependent (friction, battery), so they belong to a field or robot profile, or should be replaced by encoder/gyro closure.
  - Raw range samples should be logged with timestamps, so detection can be tuned from blackbox logs.
- **Suggested bench regressions (speculative):**
  - the obstacle at various offsets from the line centre
  - the brick flat and on edge
  - the obstacle within one robot length before and after a 90° turn
  - a second line segment within the bypass envelope
  - a dark versus a light obstacle surface
  - a low battery

### Gaps
- I found no systematic failure statistics from competitions.

## Published algorithms / state machines / open-source code

### Takeaway
The one high-quality, fully public, award-winning reference is the Overengineering² repo (RCJ Rescue Line world champion 2024). It has an explicit obstacle state machine (`obstacle_detected` → `obstacle_avoid` → `obstacle_orientate` → `line_detected`), with timeouts and fallbacks. Academic and hobby "line follower with obstacle avoidance" papers are mostly stop-or-turn heuristics with little rigour.

### Cited Findings
- Overengineering² full code, TDP, Engineering Journal and poster. Results: 1st at RCJ World Championship 2024 Eindhoven, 1st at German Open 2024, and 3rd at the European Championship 2023. — [GitHub repo](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup); state machine in [control.py ~L1975–2075](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup/blob/main/robot_v.3/Python/main/control.py)
- Older archived versions are in the same repo (`archive/2023_Hannover`, `2023_Kroatien`), showing how the design evolved. — [repo tree](https://github.com/Overengineering-squared/Overengineering-squared-RoboCup)
- An RCJ Rescue Line simulator for developing code before the hardware is ready. — [lucaSartore/Robocup-Rescue-Line-simulation](https://github.com/lucaSartore/Robocup-Rescue-Line-simulation)
- A typical academic example (STM32): stop when an obstacle is within 15 cm, turn back toward the line, and resume when it is reacquired. — [ACM DL: STM32-Based Obstacle Avoiding Line Follower Robot](https://dl.acm.org/doi/10.1145/3723178.3723290) (search-snippet level)
- Pololu 3pi wall-follower sample code: a PD law on a side Sharp sensor. — [Pololu 0J26](https://www.pololu.com/docs/0J26/all)
- Rules repositories: [robocup-junior/rescue-rules](https://github.com/robocup-junior/rescue-rules/blob/master/rules_line.adoc)

### Inferences
- The champion's state machine is a good structural template (detect → align → bypass → reacquire → orient, with timeouts and a fallback). Its open-loop timings and Python `sleep()` calls conflict with this project's no-blocking control-loop rule. Reuse the structure, not the code. Note also that the project rules (manual Điều 3.9) forbid copying other teams' code.

### Gaps
- I found no open-source code from speed-class LF Enhanced winners (Robotex, RobotChallenge, Lodz).
- Some sources were seen only through search snippets: the Microline exercise page (connection refused on fetch), the NSW Riley Rover guide and the ACM paper. Treat them as lower confidence.
