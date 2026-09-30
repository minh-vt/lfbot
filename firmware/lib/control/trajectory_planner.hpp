#pragma once

#include <vector>

#include "config/robot_config.hpp"
#include "config/types.hpp"

namespace lf {

struct ObstacleGeometry {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float width_mm = 105.0f;
  float height_mm = 60.0f;
  float angle_deg = 0.0f;
};

class TrajectoryPlanner {
public:
  TrajectoryPlanner();
  explicit TrajectoryPlanner(const EnvironmentConfig& env, const ProfilerConfig& profiler);

  // Ham chuyen tiep da thuc bac 5 C2-continuous (0 tai u=0, 1 tai u=1, dao ham 1 va 2 triet tieu
  // tai bien)
  [[nodiscard]] static float quintic_step(float u);

  // Ham hinh chuong bac 5 C2-continuous (0 tai u=0 va u=1, dinh cuc dai = 1 tai u=0.5, dao ham 1 va
  // 2 triet tieu tai bien)
  [[nodiscard]] static float quintic_bump(float u);

  // Sinh quy dao dua toi uu muot ma tu vet chay tho (Run 1) va danh sach vat can xoay goc
  [[nodiscard]] std::vector<TrajectoryPoint> generate_racing_line(
    const std::vector<TrajectoryPoint>& raw_path,
    const std::vector<ObstacleGeometry>& obstacles) const;

  // Tinh toan ho so van toc gioi han ma sat bam duong theo do cong C2
  void profile_speeds(std::vector<TrajectoryPoint>& path) const;

  [[nodiscard]] const EnvironmentConfig& get_env() const { return env_; }
  [[nodiscard]] const ProfilerConfig& get_profiler() const { return profiler_; }

private:
  EnvironmentConfig env_;
  ProfilerConfig profiler_;
};

}  // namespace lf
