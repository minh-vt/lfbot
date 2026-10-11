#pragma once

#include <cstdint>

#include "drive/types.hpp"
#include "sim_config.hpp"
#include "track_asset.hpp"

namespace lf::sim {

// Vật lý vi sai + cảm biến (physical ↔ normalized), adapter cho drive.
class WorldPhysics {
public:
  void reset(const TrackManifest& manifest);
  void step(const lf::drive::DriveCommand& cmd, float dt);
  void sample(lf::drive::Readings& frame, float dt, const TrackAsset& track) const;

  [[nodiscard]] float x() const { return x_; }
  [[nodiscard]] float y() const { return y_; }
  [[nodiscard]] float theta() const { return theta_; }
  [[nodiscard]] float v() const { return v_; }
  [[nodiscard]] float w() const { return w_; }

private:
  [[nodiscard]] float ray_tof(float ox, float oy, float base_theta, float offset) const;

  float x_ = 0.0f;
  float y_ = 0.0f;
  float theta_ = 0.0f;
  float v_ = 0.0f;
  float w_ = 0.0f;
  float wheel_ticks_l_ = 0.0f;  // tích lũy float (tránh cắt cụt tick)
  float wheel_ticks_r_ = 0.0f;
  TrackManifest manifest_;
};

}
