#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <race_trial.hpp>
#include <robot.hpp>
#include <robot_geometry.hpp>
#include <simulation.hpp>
#include <string>
#include <string_view>
#include <track_asset.hpp>

namespace {

// Separating-axis gap is a conservative lower bound on Euclidean clearance.
// Positive means separated; non-positive means overlap/contact. This belongs
// to the evaluator only: neither the controller nor its sensors receive it.
float measure_gap(const lf::sim::Pose& pose, const lf::sim::Obstacle& obstacle,
                  const lf::Calibration& calibration) {
  const float c = std::cos(pose.yaw_rad);
  const float s = std::sin(pose.yaw_rad);
  const float oc = std::cos(obstacle.angle_rad);
  const float os = std::sin(obstacle.angle_rad);
  const std::array<lf::Point2, 4> axes = {{{c, s}, {-s, c}, {oc, os}, {-os, oc}}};
  float gap = -std::numeric_limits<float>::infinity();
  for (const auto& axis : axes) {
    const float robot_radius = calibration.half_length_mm * std::abs(axis.x * c + axis.y * s) +
                               calibration.half_width_mm * std::abs(-axis.x * s + axis.y * c);
    const float obstacle_radius = 0.5F * obstacle.width_mm * std::abs(axis.x * oc + axis.y * os) +
                                  0.5F * obstacle.height_mm * std::abs(-axis.x * os + axis.y * oc);
    const float separation =
      std::abs((pose.x_mm - obstacle.x_mm) * axis.x + (pose.y_mm - obstacle.y_mm) * axis.y);
    gap = std::max(gap, separation - robot_radius - obstacle_radius);
  }
  return gap;
}

bool verify_gap() {
  auto calibration = lf::sim::simulated_calibration();
  calibration.half_length_mm = 2.0F;
  calibration.half_width_mm = 1.0F;
  const lf::sim::Pose pose;
  return std::abs(measure_gap(pose, {5.0F, 0.0F, 2.0F, 2.0F, 0.0F}, calibration) - 2.0F) < 1e-4F &&
         std::abs(measure_gap(pose, {0.0F, 5.0F, 2.0F, 2.0F, 0.0F}, calibration) - 3.0F) < 1e-4F &&
         std::abs(measure_gap(pose, {3.0F, 0.0F, 2.0F, 2.0F, 0.0F}, calibration)) < 1e-4F &&
         measure_gap(pose, {0.0F, 0.0F, 2.0F, 2.0F, 0.78539816F}, calibration) < 0.0F &&
         std::abs(measure_gap(pose, {5.0F, 0.0F, 4.0F, 2.0F, 1.57079633F}, calibration) - 2.0F) <
           1e-4F;
}

const char* describe(lf::sim::VerdictKind verdict) {
  switch (verdict) {
    case lf::sim::VerdictKind::kPending:
      return "pending";
    case lf::sim::VerdictKind::kCompleted:
      return "completed";
    case lf::sim::VerdictKind::kDnf:
      return "dnf";
  }
  return "unknown";
}

struct RunMetrics {
  double path_mm = 0.0;
  float gap_lower_bound_mm = std::numeric_limits<float>::infinity();
  unsigned blocked_ticks = 0;
  unsigned overlap_ticks = 0;
  unsigned outside_ticks = 0;
  unsigned steering_reversals = 0;
  int previous_turn = 0;

  void record(const lf::sim::WorldObservation& world, const lf::sim::Pose& previous,
              const lf::MotorOutput& output, const lf::sim::TrackAsset& track,
              const lf::Calibration& calibration) {
    path_mm += std::hypot(world.pose.x_mm - previous.x_mm, world.pose.y_mm - previous.y_mm);
    blocked_ticks += world.blocked ? 1U : 0U;
    bool overlap = false;
    for (const auto& obstacle : track.obstacles) {
      const float gap = measure_gap(world.pose, obstacle, calibration);
      gap_lower_bound_mm = std::min(gap_lower_bound_mm, gap);
      overlap = overlap || gap < 0.0F;
    }
    overlap_ticks += overlap ? 1U : 0U;
    const float c = std::cos(world.pose.yaw_rad);
    const float s = std::sin(world.pose.yaw_rad);
    const float radius_x =
      calibration.half_length_mm * std::abs(c) + calibration.half_width_mm * std::abs(s);
    const float radius_y =
      calibration.half_length_mm * std::abs(s) + calibration.half_width_mm * std::abs(c);
    const bool outside =
      world.pose.x_mm - radius_x < 0.0F || world.pose.x_mm + radius_x > track.arena_width_mm ||
      world.pose.y_mm - radius_y < 0.0F || world.pose.y_mm + radius_y > track.arena_height_mm;
    outside_ticks += outside ? 1U : 0U;
    const float turn = output.right_duty - output.left_duty;
    const int sign = static_cast<int>(turn > 1e-5F) - static_cast<int>(turn < -1e-5F);
    if (sign != 0) {
      steering_reversals += previous_turn != 0 && sign != previous_turn ? 1U : 0U;
      previous_turn = sign;
    }
  }

  void print(const std::string& name, unsigned requested_run, bool started,
             const lf::sim::WorldObservation& world, const lf::RaceStatus& status,
             bool exhausted) const {
    std::cout << name << ',' << requested_run << ',' << (started ? "yes" : "no") << ','
              << (started ? describe(world.result.verdict) : "not_started") << ','
              << static_cast<unsigned>(status.reason) << ','
              << (started ? world.result.elapsed_us : 0U) << ',' << path_mm << ',' << blocked_ticks
              << ',' << overlap_ticks << ',' << outside_ticks << ',' << steering_reversals << ',';
    if (started && std::isfinite(gap_lower_bound_mm)) {
      std::cout << gap_lower_bound_mm;
    } else {
      std::cout << "NA";
    }
    // Raster assets have no ordered-line labels yet. A finish-gate verdict
    // alone cannot establish legal navigation.
    std::cout << ",NA," << (exhausted ? "yes" : "no") << ',' << world.pose.x_mm << ','
              << world.pose.y_mm << '\n';
  }
};

class Evaluation final : public lf::RobotSession {
public:
  Evaluation(const lf::sim::TrackAsset& track, lf::sim::Simulation& simulation,
             const lf::Calibration& calibration)
    : track_(track), simulation_(simulation), calibration_(calibration) {}

  void run(lf::Robot& robot) noexcept override {
    lf::sim::RaceTrial trial(robot, simulation_);
    // Evaluator watchdog, not another referee. Exhaustion is a failure even
    // if an engine change accidentally stops enforcing the 180-second rule.
    constexpr unsigned kMaxTicks = 180000000U / lf::sim::kControlStepUs + 2U;
    for (unsigned requested_run = 1; requested_run <= 2; ++requested_run) {
      RunMetrics metrics;
      const auto before_start = trial.summary().started;
      auto previous = simulation_.observe().pose;
      auto request = lf::RaceRequest::kStart;
      for (unsigned tick = 0; tick < kMaxTicks; ++tick) {
        const auto& frame = trial.step(request);
        request = lf::RaceRequest::kNone;
        if (trial.summary().started == before_start) {
          break;
        }
        // A start repositions the world; teleportation is not travelled path.
        if (tick == 0) {
          previous = {track_.start_x_mm, track_.start_y_mm, track_.start_yaw_rad, 0.0F};
        }
        const auto& world = simulation_.observe();
        metrics.record(world, previous, frame.output, track_, calibration_);
        previous = world.pose;
        if (!trial.running()) {
          break;
        }
      }
      const bool exhausted = trial.running();
      if (exhausted) {
        (void)trial.step(lf::RaceRequest::kAbort);
      }
      const bool started = trial.summary().started > before_start;
      const auto& world = simulation_.observe();
      const auto& status = robot.observe().status;
      const bool completed = started && world.result.verdict == lf::sim::VerdictKind::kCompleted;
      const bool checks_passed = completed && !exhausted && metrics.blocked_ticks == 0 &&
                                 metrics.overlap_ticks == 0 && metrics.outside_ticks == 0;
      passed_ = passed_ && checks_passed;
      metrics.print(track_.name, requested_run, started, world, status, exhausted);
    }
  }

  void disable_motors() noexcept override {}
  [[nodiscard]] bool passed() const noexcept { return passed_; }

private:
  const lf::sim::TrackAsset& track_;
  lf::sim::Simulation& simulation_;
  lf::Calibration calibration_;
  bool passed_ = true;
};

// Diagnostics intentionally report observations rather than asserting the old
// plant is correct. They are reproducible before/after evidence for S0.
void diagnose_plant() {
  const auto calibration = lf::sim::simulated_calibration();
  lf::sim::TrackAsset track;
  track.start_x_mm = 1000.0F;
  track.start_y_mm = 1000.0F;
  for (const float heading : {0.0F, 1.57079633F, 3.14159265F, -1.57079633F}) {
    track.start_yaw_rad = heading;
    lf::sim::WorldPhysics world(calibration);
    world.reset(track);
    world.advance({1.0F, 0.0F, 100000, lf::MotorMode::kDrive}, 0, 100000, track);
    const auto& pose = world.pose();
    const float right_displacement = -(pose.x_mm - track.start_x_mm) * std::sin(heading) +
                                     (pose.y_mm - track.start_y_mm) * std::cos(heading);
    std::cout << "right_turn,heading_rad=" << heading
              << ",right_displacement_mm=" << right_displacement << '\n';
  }
  track.start_yaw_rad = 0.0F;
  lf::sim::WorldPhysics motor(calibration);
  motor.reset(track);
  motor.advance({1.0F, 1.0F, 2000, lf::MotorMode::kDrive}, 0, 2000, track);
  std::cout << "motor,first_2ms_speed_mm_s=" << motor.pose().speed_mm_s << '\n';
  motor.advance({1.0F, 1.0F, 102000, lf::MotorMode::kDrive}, 2000, 102000, track);
  const auto before_brake = motor.pose();
  motor.advance({0.0F, 0.0F, 202000, lf::MotorMode::kBrake}, 102000, 202000, track);
  std::cout << "brake,initial_speed_mm_s=" << before_brake.speed_mm_s
            << ",next_100ms_travel_mm=" << motor.pose().x_mm - before_brake.x_mm << '\n';
  track.obstacles.assign(1, {1300.0F, 1000.0F, 205.0F, 95.0F, 0.0F});
  lf::sim::WorldPhysics contact(calibration);
  contact.reset(track);
  contact.advance({1.0F, 1.0F, 1000000, lf::MotorMode::kDrive}, 0, 1000000, track);
  const auto before_contact = contact.sense(1000000, track);
  const auto blocked_pose = contact.pose();
  contact.advance({1.0F, 1.0F, 2000000, lf::MotorMode::kDrive}, 1000000, 2000000, track);
  const auto after_contact = contact.sense(2000000, track);
  std::cout << "contact,body_travel_mm=" << contact.pose().x_mm - blocked_pose.x_mm
            << ",left_encoder_ticks="
            << after_contact.wheel_counters[0].value - before_contact.wheel_counters[0].value
            << ",blocked=" << contact.blocked() << '\n';
}

}  // namespace

int main(int argc, char* const* argv) {
  if (!verify_gap()) {
    std::cerr << "evaluation clearance self-check failed\n";
    return 2;
  }
  if (argc == 2 && std::string_view(argv[1]) == "--self-test") {
    std::cout << "evaluation clearance: separated/contact/overlap/rotated PASS\n";
    return 0;
  }
  if (argc == 2 && std::string_view(argv[1]) == "--diagnose") {
    diagnose_plant();
    return 0;
  }
  if (argc != 1) {
    std::cerr << "usage: evaluate_race [--self-test|--diagnose] (from firmware/)\n";
    return 2;
  }
  std::cout << "track,requested_run,started,verdict,stop_reason,elapsed_us,path_mm,blocked_ticks,"
               "overlap_ticks,outside_ticks,steering_reversals,gap_lower_bound_mm,"
               "line_order_check,budget_exhausted,end_x_mm,end_y_mm\n";
  bool passed = true;
  for (const auto* manifest : {"assets/track_vongloai.json", "assets/track_chungket.json"}) {
    lf::sim::TrackAsset track;
    std::string error;
    if (!track.load(manifest, error)) {
      std::cerr << manifest << ": " << error << '\n';
      return 2;
    }
    const auto calibration = lf::sim::simulated_calibration();
    lf::sim::Simulation simulation(calibration);
    simulation.use_track(track);
    Evaluation session(track, simulation, calibration);
    lf::run_robot(calibration, session);
    passed = passed && session.passed();
  }
  if (passed) {
    std::cerr << "execution checks passed, but route-order and map-gain acceptance are not "
                 "implemented yet\n";
    return 3;
  }
  return 1;
}
