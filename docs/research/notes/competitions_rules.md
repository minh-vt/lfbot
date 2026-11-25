# Line-following robot competitions: rules, scoring, track specs and the strategies they reward

Scope note: emphasis is on formats similar to F-Maze-Solving (black 2 cm line on white, random box obstacles 10.5 x 6 cm, max 2 runs per round, best valid time counts, secret track revealed on the day, one 3-minute reconnaissance window). The F-Maze rules themselves come from the project's local copy `docs/manual.md` / `docs/fqa.csv` (cited as local files, not URLs). Several rulebooks are PDFs; numbers below were extracted from the PDF text with `pdftotext`, not from search snippets, unless stated.

## Which competitions exist and what are their rules

### Takeaway
The closest international analogues to F-Maze are the "Line Follower Enhanced" family (Robotex, RobotChallenge, Polish SUMO Challenge/Robotic Arena-style events): fastest-time races on a black-on-white line with gaps, a box/wall obstacle that must be circumnavigated and a forced line re-acquisition distance. Japan's Robotrace is the canonical "explore then exploit" race (multiple runs, best lap counts, curvature markers) but has no obstacles. RoboCup Junior Rescue Line has obstacles/gaps/intersections but is point-scored, not timed, and explicitly bans pre-mapping.

### Cited Findings

**F-Maze-Solving (FPT University / CLB F-Chip, HCMC, 2026) – local rules**
- Robot max 16 x 16 cm measured ready-to-start; must not change size while running; max 12 V nominal (3S 12.6 V accepted); fully autonomous except start button — `docs/manual.md` Dieu 3 (local file /home/minhvt/Projects/lfbot/docs/manual.md); FAQ Q-01, Q-08 in `docs/fqa.csv`.
- No suction fan; all floor contacts must be rolling wheels — `docs/manual.md` Dieu 3.8; FAQ Q-02.
- Track secret, revealed on competition day; practice track published beforehand "will not be the same as the real track"; line 2 cm wide, black on white; same track for all teams within a round, changes only between rounds (qualifier → final) — `docs/manual.md` Dieu 4.1–4.3; FAQ Q-03.
- "Random" rectangular-box obstacles 10.5 x 6 cm, placed lying or standing — `docs/manual.md` Dieu 4.4.
- One reconnaissance ("dò thử") opportunity of up to 3 minutes before the official runs — `docs/manual.md` Dieu 4.5.
- Robots impounded after check-in; no code reload or modification after the real track is published — `docs/manual.md` Dieu 4.7; FAQ Q-04.
- Max 2 runs per round, 3 min each; ranking by best valid time; DNF teams ranked below all finishers; tie-break = second-best run, then run-off — `docs/manual.md` Dieu 5.1–5.2, 6, 7.
- Leaving the line to find the way is not a foul, but leaving the line and cutting across it ("cắt line") voids the run — `docs/manual.md` Dieu 5.5, Dieu 8; FAQ Q-05.
- Practice (07:00–08:00, unlimited runs, parameter/code edits allowed but not hardware changes) — `docs/manual.md` Dieu 3.5, 5.3.
- Dates: qualifier 11/11/2026, final 2/12/2026 at FPT University HCMC — `docs/manual.md` Dieu 10.

**Robotex International – Line Following Enhanced (rules updated 08.02.2026)**
- Field white synthetic sheets 3–100 m²; line 15 mm black; minimum turning radius 0; 25 cm free space either side of the line; perpendicular crossings for ≥20 cm, robot must go straight at crossings — [Robotex LFE 2026 PDF](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf)
- Robot max 30 x 30 x 30 cm, 3 kg, ≤24 V; must always cover the line while following it, otherwise race failed — same source.
- Format: up to 5 rounds, 1 attempt each; top 10% (up to 25%) to finals; in finals each team has 5 minutes for unlimited attempts, best counts; max lap 3 min; must start within 3 s of the start command — same source.
- Obstacles: line breaks ≤10 cm (≥10 cm unbroken line before a curve, ≥2 cm line between consecutive breaks); cuboid obstacle/wall max 25 x 15 x 10 cm, robot must drive around and "cover the line at least partially within the 30 cm following the obstacle"; hitting allowed but not recommended; obstacle not white; ≥20 cm straight line after it — same source.
- Other features: swing (≥50 cm long, fulcrum ≤8 cm), mountain (≤15 cm high, must not be bypassed), line width varying 5–30 mm, road split (shorter branch marked by 15 x 30 mm mark ≥100 mm before the split, 30–60° split), speed-limit zone before finish (<0.5 m/s, marked by two 15 x 60 mm lines; violation adds zone time) — same source.
- Revision history: road split and speed-limit zone added Aug 2021; "track loop" removed June 2024; all paragraphs updated Feb 2026 — same source.
- Lighting/IR noise at the arena is uneven; organisers recommend sensor covers and testing under direct sunlight — same source §8.1.

**RobotChallenge – Line Follower / Line Follower Enhanced (China edition, revised 15 Sep 2025)**
- 3 rounds; shortest time counts; 3-min limit (timeouts recorded as 03:00:01); electronic timing over one closed lap — [RC-LineFollower.pdf](https://www.robotchallenge.org.cn/doc/en/RC-LineFollower.pdf)
- Mat 2.8 x 4.5 m, white with 15 mm black line; no intersections; parallel segment spacing ≥15 cm; line ≥15 cm from mat edge; minimum arc radius 7.5 cm; angles ≥90° may exist; robot ≤30 x 30 cm, ≤3 kg — same source.
- Off-line = entire projection leaves the line; spinning in place or reversing direction also counts as off-line — same source §2.2.6.
- Enhanced props: one 10 cm gap, not on a curve, straight continuation; brick obstacle ≈25 x 12 x 6.5 cm, robot "must detour around this brick and immediately return"; seesaw ≥50 cm, axle <12 cm; going off-line is allowed during obstacle navigation, but must return to the intended path afterwards — same source §3.2.6.E, §3.4.
- The original Austrian RobotChallenge site says the enhanced version must "additionally overcome unexpected obstacles" (search snippet only) — [robotchallenge.org.cn competition page](https://robotchallenge.org.cn/competition-LineFollower.html)

**Polish SUMO Challenge (Łódź University of Technology) – Line Follower / LF Enhanced / LF Turbo / LF Enhanced3D**
- Preliminaries: unlimited attempts within a scheduled window, best time qualifies; finals: 4 attempts, best counts; 3-min attempt limit — [SUMO Challenge LF rules](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf)
- Line ≈19 mm black on white HDF boards; closed route, possibly >1 lap; robot must fit on A4 sheet, weight unlimited; LF classic bans downforce devices, LF Turbo allows turbines — same source.
- Enhanced obstacles: line breaks ≤10 cm not on curves (sequences possible); wall ≈25 x 6.5 x 12 cm (L x W x H), touching allowed, track built so robot can pass on the right, must rejoin within 40 cm behind the obstacle; intersections (go straight); inverted colours ≤0.5 m; double line ≈20 mm apart; fork with different obstacles on each branch; bridge/tunnel (16 cm clearance); hill ≤15 cm; swing; loop ≤30 cm diameter (≤3 passes); line width 10–38 mm — same source.
- Organisers choose which obstacles appear on the day and may remove ones that prove too hard — same source §3d.

**Japan – All Japan Micromouse Contest, Robotrace class (NTF)**
- Black surface, white line 1.9 cm, max course length 60 m; straights and arcs, min arc radius 10 cm, curvature-change points >10 cm apart; crossings at 90±5° with 25 cm straight before/after — [NTF Robotrace rules (EN)](https://ntf.or.jp/mouse/rule/kitei_trace-EN.html)
- Start/goal markers on the right of the line; corner markers on the left wherever curvature changes; partial 5° bank allowed; arcs of continuously varying curvature may appear — same source.
- Robot ≤25 x 25 x 20 cm, no suction; 3 minutes and max 3 runs; fastest lap is official; robot must stop automatically in the start-goal area for ≥2 s or the time isn't recorded; failure if the projection fully leaves the line or the robot stops >2 s — same source.
- Operators may not give the robot course information after the course is opened to the public, nor change course data via switches during the contest — same source rule 3-8.
- Older (2011) English rules: same 1.9 cm line, 10 cm min radius, up to 3 runs in 3 minutes, shortest lap recorded; start/goal gate 40 x 25 cm — [NTF 2011 Robotrace rules](https://ntf.or.jp/mouse/micromouse2011/rulerobotrace-EN.html)

**UK Micromouse (UKMARS) Line Follower – Robotrace-derived**
- White 10 mm (±2 mm) line on black; min radius 75 mm; straights ≤6 m; right-angle crossovers only; turn markers on the left at each curvature change, 20 mm long, line width, centred on the change, offset 25 mm; start/finish markers on the right; finish 250–500 mm behind start — [UKMARS LF rules](https://ukmars.org/contests/contest-rules/line-follower/)
- Typically 3–5 min performance period with 3–4 attempts; handler may change batteries and program parameters between runs — same source.
- A 2024 IEEE (TU Eindhoven student branch) LF rule set uses a 19 mm line with 19 x 40 mm markers 40 mm from the track (search snippet; not opened) — [IEEE TU/e LF rules v1.1](https://ieee.tue.nl/wp-content/uploads/2024/04/IEEE__Linefollwer_Competition_Rules_v1_1.pdf)

**RoboCup Junior Rescue Line (2026 rules, last updated 2026-01-28)**
- 30 x 30 cm tiles; black line 1–2 cm; gaps ≤20 cm on straights with ≥5 cm straight line before each; line ≥10 cm from edges/walls; red goal strip 25 x 300 mm — [RCJ Rescue Line 2026 draft](https://junior.robocup.org/wp-content/uploads/2026/01/RCJRescueLine2026-draft.pdf)
- Obstacles: bricks/blocks/weights ≥15 cm tall, may be fixed to the floor, may be moved by the robot and stay where moved, not closer than 25 cm to field edge, max one line/tile; speed bumps ≤1 cm; debris ≤3 mm; green 25 x 25 mm intersection markers; dead end = two green marks — same source.
- 8 minutes per game including calibration; calibration does not count as pre-mapping; "pre-mapping" by humans disqualifies; "any pre-mapped type of dead reckoning" is prohibited; organisers say the field layout will not be revealed beforehand — same source.
- Scoring: 10 pts intersection/dead end, 20 pts seesaw, 20 pts per obstacle, 10 pts gap, 10 pts ramp tile, 10 pts speed bump; tile points decay with attempts (5/3/1/0 per tile); lack of progress → restart at last checkpoint; exit bonus = 60 − 5 × (LoPs); victim-rescue multipliers ×1.4 — same source §5.6.
- Final ranking = 0.6 × mean of normalized field scores (worst round dropped) + 0.2 rubrics + 0.2 technical challenge — same source §6.1.
- July 2025 judge-training slides confirm gap/obstacle placement rules and that a pushed obstacle stays moved — [RCJ 2025 Judge Training Line](https://rescue-community-website.pages.dev/rules/2025/JudgeTraining_Line.pdf)

**RoboGames (USA) Line Following**
- Search snippet reports a 3-minute limit, 500 g weight limit and a white paper course 44" x 96"; the same snippet also states an arena "111 cm wide and 2448 cm long", which is inconsistent with 44" x 96" (≈112 x 244 cm) and looks like a garbled 244.8 cm. Not verified; the page's TLS certificate had expired when fetched — [RoboGames LF rules](https://robogames.net/rules/line-following.php)

**Other events surfaced (rules exist, not opened in detail)**
- Turkey MEB robot competitions (line follower basic), ITURO (ITU), METU UORG line follower — [MEB 2023](https://robot.meb.gov.tr/yukleme/Robot2023/11_Cizgi_izleyen_Temel_en.pdf), [ITURO 2025](https://ituro.org/assets/images/kategoriler/%C3%87izgi%C4%B0zleyenEng25.pdf), [METU UORG 2022](https://robot.metu.edu.tr/assets/2022%20UORG%20Line%20Follower%20Category%20Rules.pdf)
- Robotic Arena / TSI Latvia, KTU Lithuania "Robots Intellect", Ukraine NENC — [TSI](https://robotic.tsi.lv/robotic/rules/linefollower), [KTU](https://robotsintellect.ktu.edu/wp-content/uploads/sites/416/2022/10/Linijos-sekimas-EN.pdf), [NENC](https://nenc.gov.ua/wp-content/uploads/2025/10/RulesRegulations-Line-Follower.docx)
- Malaysia Petrosains RBTX "Speed Rivals" tracer — [Petrosains 2024](https://petrosains.com.my/rbtx-challenge-2024/Assets/comp/tracer/221024_RnR_24_Speed_Rivals.pdf)

**Vietnam**
- Searches found no public rulebook for a student "dò line + vật cản" contest other than the local F-Maze manual. Found: HCMUTE "Xe tự hành 2025" (image-processing/AI autonomous car, teams of 3–5, final planned 12/7/2025) — [HCMUTE plan](https://stiao.hcmute.edu.vn/Resources/Docs/SubDomain/stiao/Kế%20hoạch%20-%20Xe%20tự%20hành%202025.pdf); Quảng Ngãi high-school robot contest with a line-following task — [Quảng Ngãi 2025](https://lienhiephoi.quangngai.gov.vn/items/doc/334.pdf); Robocon Việt Nam and WRO (Robotacon WRO 2025) are task/field games, not line races — [VnEconomy](https://vneconomy.vn/doi-vo-dich-robocon-viet-nam-2025-se-dai-dien-thi-dau-quoc-te-tai-mong-co.htm), [Tạp chí Công Thương](https://tapchicongthuong.vn/robotacon-wro-2025--toa-sang-tai-nang-robot-viet-nam-huong-den-chung-ket-quoc-te-tai-singapore-171936.htm)

### Inferences
- F-Maze's obstacle (10.5 x 6 cm footprint, "lying or standing") is much smaller than the Enhanced-family walls (25 x 6.5–15 cm) and it is placed "randomly", while Robotex/SUMO Challenge guarantee ≥20 cm straight after the obstacle and a re-acquisition window (30 cm / 40 cm). F-Maze states neither, so detour geometry cannot rely on a guaranteed straight exit.
- F-Maze's "ra ngoài vạch để tìm đường không phạm quy, nhưng cắt line thì loại lượt" is close to RobotChallenge 3.2.6.E (off-line allowed during obstacle navigation, must return to intended path) and the SUMO Challenge "shortening the track is not allowed".
- F-Maze's 16 x 16 cm limit is far tighter than the 25–30 cm limits elsewhere; the Polish "fit on A4" is the next closest.

### Gaps
- Original Austrian RobotChallenge (robotchallenge.org) LF Enhanced rulebook not retrieved; numbers above are from the China edition, which may differ.
- RoboGames full rule text not readable (expired certificate); details unverified.
- No public rule source found for "Cuộc đua số" (that contest is an autonomous-car/camera contest by FPT, not line following, from memory; not verified this session) or any other Vietnamese line+obstacle student contest.
- No other contest named "F-Maze" was found online.

## Track specs (line width, colour, radius, crossings, gaps, markers, obstacles, timing)

### Takeaway
Two families: Japanese/UK "Robotrace" (white ~10–19 mm line on black, min radius 7.5–10 cm, left-side curvature markers, right-side start/goal markers, crossings) and European "Line Follower" (black 15–19 mm on white, min radius 7.5 cm or even 0, no markers except Robotex road-split/speed-zone marks). Obstacles are cuboids 6.5–15 cm high with a guaranteed line re-acquisition zone.

### Cited Findings
| Contest | Line | Min radius | Crossings | Gaps | Markers | Obstacle | Limit/run |
|---|---|---|---|---|---|---|---|
| F-Maze 2026 | 20 mm black on white | not stated | not stated | not stated | goal box only | 10.5 x 6 cm box, random, lying/standing | 3 min, 2 runs |
| Robotex LFE 2026 | 15 mm black on white, 5–30 mm variable | 0 | yes, go straight | ≤10 cm | road-split 15x30 mm, speed-zone 15x60 mm | ≤25x15x10 cm, rejoin ≤30 cm | 3 min |
| RobotChallenge LFE 2025 | 15 mm black on white | 7.5 cm | none | one 10 cm, straight | none | ≈25x12x6.5 cm | 3 min, 3 rounds |
| SUMO Challenge LFE | ≈19 mm black on white, 10–38 mm | n/s | yes, go straight | ≤10 cm, not on curves | none | ≈25x6.5x12 cm, rejoin ≤40 cm | 3 min |
| NTF Robotrace | 19 mm white on black | 10 cm | 90±5°, 25 cm straight around | none | start/goal right, curvature left | none | 3 min, 3 runs |
| UKMARS LF | 10±2 mm white on black | 75 mm | 90° | none | turn markers 20 mm, offset 25 mm | none | 3–5 min, 3–4 runs |
| RCJ Rescue Line 2026 | 10–20 mm black on white tiles | n/s | green-marked intersections | ≤20 cm | green 25x25 mm, silver/black zone strips | ≥15 cm tall, may be fixed | 8 min game |

Sources: F-Maze `docs/manual.md`; [Robotex](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf); [RobotChallenge](https://www.robotchallenge.org.cn/doc/en/RC-LineFollower.pdf); [SUMO Challenge](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf); [NTF](https://ntf.or.jp/mouse/rule/kitei_trace-EN.html); [UKMARS](https://ukmars.org/contests/contest-rules/line-follower/); [RCJ 2026](https://junior.robocup.org/wp-content/uploads/2026/01/RCJRescueLine2026-draft.pdf)

- Timing hardware: Robotex optical gate, 3 mm beam at 3 cm height, robot body must block it — [Robotex](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf); NTF transmission photoelectric sensors with axis ~1 cm above floor — [NTF](https://ntf.or.jp/mouse/rule/kitei_trace-EN.html)
- Lighting: NTF will not adjust lighting on request — [NTF](https://ntf.or.jp/mouse/rule/kitei_trace-EN.html); SUMO Challenge warns of varying lighting and forbids shading the track — [SUMO Challenge](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf); Robotex allows lighting to change between attempts and warns of IR noise — [Robotex](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf)

### Inferences
- Every timed contest above uses a 3-minute cap; this is effectively a standard.
- Contests that want robots to learn the course (Robotrace, UKMARS) supply curvature markers; obstacle contests do not, so mapping there relies on odometry/IMU and line-event signatures.
- Lighting is unreliable across all organisers, which supports per-field calibration rather than fixed thresholds (consistent with the project's AGENTS.md rule on field profiles).

### Gaps
- NTF marker physical dimensions are in figures not present in the English HTML text.
- F-Maze minimum radius, crossings, gaps and obstacle-to-line clearances are not stated in the manual/FAQ excerpt read.

## How scoring rewards strategy

### Takeaway
Almost all timed contests rank by best single valid time (not sum), so the incentive is: bank a safe run, then gamble on a fast one. F-Maze (2 runs, best counts, tie-break by second-best run) fits this. RCJ is the exception (points, decay with retries, worst round dropped).

### Cited Findings
- Best-of-runs: F-Maze best valid of ≤2 runs, tie-break by second run — `docs/manual.md` Dieu 5.1, 7.1; RobotChallenge best of 3 rounds — [RobotChallenge](https://www.robotchallenge.org.cn/doc/en/RC-LineFollower.pdf); NTF fastest lap of ≤3 runs — [NTF](https://ntf.or.jp/mouse/rule/kitei_trace-EN.html); SUMO Challenge finals best of 4 — [SUMO Challenge](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf); Robotex finals unlimited attempts in 5 min, best counts — [Robotex](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf)
- Penalties are run-voiding rather than time-additive: off-line / reversing / spinning in place voids the run in RobotChallenge; driving off track fails the attempt in Robotex; NTF fails the run if projection leaves the line or robot stops >2 s; F-Maze voids the run on "cắt line" or manual intervention — sources as above.
- Time-additive penalty exists only in Robotex speed-limit zone (zone time added if >0.5 m/s) — [Robotex](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf)
- Obstacle contact: allowed (Robotex "not recommended"; SUMO Challenge allowed); RCJ allows moving obstacles — sources above. F-Maze manual only penalises humans touching obstacles (warning, then DQ) — `docs/manual.md` Dieu 8; robot contact is not addressed in the excerpt read.
- RCJ: retries cost points (tile points 5→3→1→0; exit bonus −5 per LoP) — [RCJ 2026](https://junior.robocup.org/wp-content/uploads/2026/01/RCJRescueLine2026-draft.pdf)
- NTF requires automatic stop in the goal area for ≥2 s, else no time — [NTF](https://ntf.or.jp/mouse/rule/kitei_trace-EN.html); F-Maze finish = wheel/body touches goal box — `docs/manual.md` Dieu 4.6.

### Inferences
- With best-of-2 and a DNF-ranked-last rule, the dominant F-Maze strategy is: run 1 prioritises a guaranteed valid finish (and gathers map data), run 2 spends the margin on speed. A team with one fast run beats a team with two medium runs.
- Because penalties void runs instead of adding time, aggressive corner-cutting or obstacle shortcuts have binary risk; a learned speed profile should keep margin where run-1 data showed tracking error.

### Gaps
- F-Maze does not say whether the 3-minute reconnaissance counts as a run, nor whether the robot may store data across reconnaissance → run 1 → run 2 (manual says no code reload after track reveal, not that on-board memory must be cleared).

## Winning times / records

### Takeaway
I did not find reliable, citable recent winning times for Robotrace, RobotChallenge or Robotex LF in this session.

### Cited Findings
- NTF publishes per-year records/pamphlets (e.g. MM2023 record PDF, held 17–18 Feb 2024), but the PDF exceeded the fetch size limit and search snippets did not include Robotrace times — [NTF MM2023 record](https://www.ntf.or.jp/mouse/micromouse2023/MM2023recode.pdf), [MM2023 pamphlet](https://www.ntf.or.jp/alljapan2023/wp-content/uploads/2024/02/MM2023パンフレットweb.pdf)
- Pololu LVBots 2015 learning robot: no lap times reported; motors already at 83%, straightaway boost made little difference — [Pololu blog](https://www.pololu.com/blog/486/davids-line-following-robot-that-learns-the-course)

### Inferences
- None drawn; no figures to compare.

### Gaps
- Robotrace winning lap times/speeds, RobotChallenge and Robotex LF records: not retrieved (large PDFs / not in snippets). Next step would be downloading the NTF record PDF locally and grepping "ロボトレース".

## What rule features enable "learn in run 1, exploit in run 2"

### Takeaway
Learning is enabled by (a) multiple runs on the identical course with best-time ranking, (b) the robot retaining on-board state between runs, (c) course features the robot can localise against (markers, crossings, start/goal), and (d) a ban on humans feeding course data, which forces the learning to be autonomous. RCJ deliberately disables it (field changes, pre-mapped dead reckoning banned).

### Cited Findings
- Robotrace: 3 runs on the same course, fastest lap counts; operators may not input course data after the course opens or change it via switches — i.e. only robot-acquired data is legal — [NTF](https://ntf.or.jp/mouse/rule/kitei_trace-EN.html)
- Robotrace markers on the left at every curvature change and start/goal markers on the right give the robot segment boundaries — [NTF](https://ntf.or.jp/mouse/rule/kitei_trace-EN.html); UKMARS copies this (20 mm markers offset 25 mm) — [UKMARS](https://ukmars.org/contests/contest-rules/line-follower/)
- An mbed Robotrace kit manual describes recording course configuration in memory on the first run and running faster on the second using those records, with start/goal markers giving position — [mbed robot tracer manual](https://os.mbed.com/media/uploads/hayama/mbedrobotracer-manual-english.pdf) (search snippet)
- Pololu LVBots robot: lap 1 logs encoder+gyro X/Y and heading every 100 ticks, detects lap end geometrically, laps 2–3 boost on straights while PID still tracks the line — [Pololu blog](https://www.pololu.com/blog/486/davids-line-following-robot-that-learns-the-course)
- A university lab report used run 1 to measure curvature (7 segments at low speed) and run 2 to drive faster — [lab3 report](https://yycho0108.github.io/Academics/assets/poe/lab3_report.pdf) (search snippet)
- UKMARS lets the handler change program parameters between runs — [UKMARS](https://ukmars.org/contests/contest-rules/line-follower/); F-Maze forbids any code reload/modification after track reveal — `docs/manual.md` Dieu 4.7, so all learning must be on-board and automatic.
- F-Maze: same track for all teams within a round, ≤2 runs, plus one 3-min reconnaissance — `docs/manual.md` Dieu 4.3, 4.5, 5.1.
- RCJ prohibits "any pre-mapped type of dead reckoning" and organisers may change tiles/obstacles between rounds — [RCJ 2026](https://junior.robocup.org/wp-content/uploads/2026/01/RCJRescueLine2026-draft.pdf)
- Enhanced-family obstacles are fixed during a round (Robotex/SUMO Challenge describe placement constraints; SUMO organisers pick obstacles on the day) — [Robotex](https://robotex.international/wp-content/uploads/2026/02/Line-Following-Enhanced-ENG-1.pdf), [SUMO Challenge](https://www.skaner.p.lodz.pl/sumochallenge/files/en/linefollower.pdf); RCJ obstacles moved by a robot stay moved — [RCJ](https://junior.robocup.org/wp-content/uploads/2026/01/RCJRescueLine2026-draft.pdf)

### Inferences
- F-Maze has the key enablers (same course across runs, best-time ranking, on-board autonomy) but lacks Robotrace-style curvature markers, so a run-1 map must be built from odometry/IMU plus line events (corners, gaps, obstacle encounters) and re-localised in run 2 against those events.
- Because F-Maze potentially gives three passes (reconnaissance + 2 runs), the reconnaissance could serve as the mapping pass if the rules let the robot keep memory; this needs an organiser answer (see Gaps under scoring).
- Obstacles in F-Maze are "random" but fixed for the round, so obstacle positions learned in run 1 are valid for run 2 unless someone touches them (human touching is penalised).
- Run-voiding penalties mean the exploit run should fall back to reactive line-following when the map disagrees with sensors (map drift), as in the Pololu design where PID always stays in control.

### Gaps
- No primary team write-up (Japanese Robotrace blogs) was opened this session; specific Robotrace speed-profile techniques (e.g. curvature-based velocity limits, acceleration figures) are not cited here.
- Whether obstacle positions in F-Maze can differ between run 1 and run 2 within a round is not explicitly stated in the manual.
