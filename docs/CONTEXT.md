# LF-Bot Domain Model (SSoT)

Ubiquitous language for the LF-Bot autonomous racing platform.

---

## 1. Track & Trajectory Domain

- **`RawTrace`**: Sequence of discrete spatial samples $(x, y, \theta, s)$ recorded during Run 1 mapping from odometry and optical sensors.
- **`TrackCorridor`**: Geometrically bounded lateral space around the line centerline ($[-w_{\text{left}}, w_{\text{right}}] \le \pm w_{\text{max}}$, configurable up to $120\text{ mm}$, default $80\text{ mm}$ via HUD `"Max Dev"`), allowing the optimizer to relax curvature across track boundaries while respecting competition limits.
- **`RacingTrajectory`**: Continuous spatial profile containing $(x, y, \theta, \kappa, v^*)$ optimized for minimum traversal time via curvature relaxation and late-apex exit acceleration.
- **`TrajectoryPoint`**: Atomic waypoint on a trajectory storing spatial coordinates, tangent angle, curvature $\kappa$, and target velocity $v^*$.
- **`LateApex`**: Geometric transformation shifting the turn-in point outward and apex point toward the exit of a corner, maximizing exit radius and allowing earlier full-throttle acceleration.
- **`CorridorRelaxation`**: Constrained spatial optimization minimizing $\int \kappa^2 ds$ subject to lateral deviation bounds $|n(s)| \le w_{\text{dev}}$ and jerk damping.
- **`GlobalTrajectoryOptimizer`**: Minimum-curvature Convex Quadratic Program (QP) optimizing the global racing line $(x^*, y^*)$ across the track subject to corridor boundaries and smooth obstacle repulsive fields.

---

## 2. Vehicle Dynamics Domain

- **`FrictionEnvelope`**: Bounded elliptical acceleration capability $(a_{\text{lat}}^2 + a_{\text{long}}^2 \le (\mu g)^2)$.
- **`GripVelocity` ($v_{\text{grip}}$)**: Maximum cornering speed before tire saturation: $v_{\text{grip}} = \sqrt{\frac{\mu g}{|\kappa| + \epsilon}}$.
- **`ContinuousKinodynamicLimit` ($v_{\text{diff\_limit}}$)**: Closed-form differential drive saturation speed bound derived from track width $W_{\text{base}}$ and outer wheel speed limit:
  $$v_{\text{diff\_limit}} = \frac{v_{\text{top}}}{1.0 + \left(\frac{v_{\text{top}}}{v_{\text{safe}}}\right) |\kappa| W_{\text{base}}}$$
- **`ForwardBackwardPass`**: Two-pass dynamic programming algorithm enforcing motor torque acceleration limits forward and tire friction braking limits backward.
- **`KinematicFeedforward`**: Exact steering offset required to maintain circular motion at speed $v$ and curvature $\kappa$: $\text{steer}_{ff} = \frac{1}{2} v \cdot \kappa \cdot W_{\text{base}}$.
- **`CounterRotation`**: Symmetrical wheel speed capability $[-v_{\max}, +v_{\max}]$ in differential tracking allowing the inner wheel to actively reverse, enabling tight pivot turns ($R \to 0$) without understeer spinouts.

---

## 3. Sensing & Estimation Domain

- **`Centroid`**: Weighted optical line deviation estimate calculated from analog sensor array reflectivities.
- **`OpticalDynamicBraking`**: Run 1 continuous dynamic deceleration based on optical centroid deviation:
  $$v(e) = \frac{V_{\text{map}}}{1.0 + \left(\frac{|e|}{20.0\text{mm}}\right)^2}$$
  guaranteeing automatic crawl speeds through sharp $90^\circ$ corners on unmapped tracks without arbitrary piecewise if-else thresholds.
- **`SensorCone`**: Spatial field-of-view representation of hardware sensors:
  - **IR Footprint**: Circular ground area ($\varnothing 5\text{ mm}$) sampled via $3 \times 3$ area integration on track bitmap.
  - **ToF Cone**: Dense 21-ray active conical sector casting spanning $25^\circ$ FoV, returning shortest hit distance and boolean `has_hit` flag to distinguish obstacle reflection from empty range.
- **`Odometry`**: Dead-reckoning state estimator integrating wheel encoder pulses and yaw change $d\theta = \frac{ds_r - ds_l}{W_{\text{base}}}$.
- **`CalibratedOdometryScale`**: Hardware-grounded distance-per-pulse factor (`meters_per_tick_scale = distance / ticks`), replacing speculative wheel diameter and gear ratio constants.

---

## 4. Control & Orchestration Domain

- **`TacticalBehaviorArbiter`**: Deep domain strategy module adjudicating tactical state, managing optical line loss dead-reckoning momentum (< 300 ms), triggering Run 1 continuous $C^2$ obstacle evasion chicanes, and executing emergency halts.
- **`TrajectoryFollower`**: Low-level tracking module combining feedforward trajectory tracking with feedback optical error rejection.
- **`RobotCoordinator`**: Thin system orchestrator coordinating mode transitions (Idle, Run 1 Map, Run 2 Sprint) and peripheral lifecycle.
- **`StanleyTracker`**: Non-linear path tracking controller combining lookahead cross-track error $e_{\text{fa}}$, heading alignment error $\psi_e$, and kinematic feedforward offset to command smooth wheel differential speeds.
- **`RelayAutoTuner`**: Closed-loop limit-cycle feedback tuner measuring ultimate period $T_u$ and oscillation amplitude to compute converged PID gains via Åström-Hägglund rules.

---

## 5. Race Arbitration Domain

- **`SimEngine`**: Headless discrete physics simulation engine operating on memory buffers and 2D bitmap textures at 500 Hz without graphical dependencies.
- **`RaceArbiter`**: Independent track-side digital steward judging race validity based on ground-truth physical state, completely isolated from robot internal firmware state.
- **`SectorGate`**: Directed spatial checkpoint line segment $(\mathbf{p}_1, \mathbf{p}_2, \mathbf{n})$ that must be traversed sequentially in the forward normal direction $(\vec{v} \cdot \mathbf{n} > 0)$.
- **`FinishBox`**: Bounded rectangular terminal zone ($160 \times 160\text{ mm}$ at $x=400, y=1300\text{ mm}$) per Competition Article 4.6, in which the vehicle body/wheels must arrive and come to a controlled halt ($v \le 0.05\text{ m/s}$).
- **`InfractionCode`**: Authoritative disqualification penalty code (`kObstacleCollision`, `kOutOfBounds`, `kLineLostTimeout`, `kWrongWay`, `kGateSkipped`, `kTimeLimitExceeded`, `kPhysicsAnomaly`).
- **`LapVerdict`**: Immutable certification record generated by `RaceArbiter` indicating official lap time, status (`kCompleted` vs `kDisqualified`), cleared sector count, and specific infraction details.

