# Quy ước đặt tên

Lấy cảm hứng từ Linux: tên **"boring, obvious"**, nói **việc robot làm**, không nói **lý thuyết đằng sau**.

## Nguyên tắc

1. **Hàm = động từ việc làm**: `steer_to_line`, `plan_speed`, `avoid_obstacle`, `drive_motors`.
2. **Struct = danh từ miền (CamelCase)**: `Readings`, `RacePlan`, `DriveCommand`, `Tuning`.
3. **Lý thuyết (PD, feedforward, backward-pass, curvature, μ) nằm trong comment/implementation — không ra tên public.**
4. Không chữ tắt trừ khi phổ thông (`ir`, `tof`, `enc`, `adc`).
5. Chọn tên "boring" hơn tên "clever".

## Map cũ → mới

| Cũ (lý thuyết/mechanism) | Mới (việc robot làm) |
|---|---|
| `SensorFrame` | `Readings` |
| `TrackPlan` | `RacePlan` |
| `MotorCommand` | `DriveCommand` |
| `KernelState` | `DriveState` |
| `TuneConfig` | `Tuning` |
| `resolve_line_error` | `resolve_line_offset` |
| `line_error` (field) | `line_offset` |
| `steer_pd` | `steer_to_line` |
| `steer_ff` (field) / `steer_at` | `planned_steer` / `planned_steer_at` |
| `throttle_profile` (hàm) / `throttle_max[]` / `throttle_at` | `choose_speed` / `speed_limit[]` / `speed_limit_at` |
| `bake()` / `baked` | `plan_speed()` / `ready` |
| `corner_slowdown` | `corner_limit` |
| `decel_per_tick` | `brake_limit` |
| `evasion_bias` / `evasion_slowdown` | `avoid_strength` / `avoid_slowdown` |
| `steer_evasion` | `avoid_obstacle` |
| `mix_differential` | `drive_motors` |
| `e_hat` / `confidence` | `line_estimate` / `line_certainty` |
| `curvature[t]` | `line_curve[t]` |
