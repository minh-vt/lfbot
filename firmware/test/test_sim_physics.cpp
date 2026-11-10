#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <robot_geometry.hpp>
#include <simulation.hpp>
#include <track_asset.hpp>
#include <world_physics.hpp>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

lf::sim::TrackAsset empty_track() {
  lf::sim::TrackAsset track;
  track.start_x_mm = 1000.0F;
  track.start_y_mm = 1000.0F;
  track.arena_width_mm = 3000.0F;
  track.arena_height_mm = 3000.0F;
  return track;
}

void verify_right_turn() {
  auto track = empty_track();
  for (const float heading : {0.0F, 1.57079633F, 3.14159265F, -1.57079633F}) {
    track.start_yaw_rad = heading;
    lf::sim::WorldPhysics world;
    world.reset(track);
    world.advance({1.0F, 0.0F, 100000, lf::MotorMode::kDrive}, 0, 100000, track);
    const auto& pose = world.pose();
    const float right = -(pose.x_mm - track.start_x_mm) * std::sin(heading) +
                        (pose.y_mm - track.start_y_mm) * std::cos(heading);
    require(right > 0.0F, "left wheel faster must move toward body-right in raster y-down");
  }
}

void verify_ir_handedness() {
  auto track = empty_track();
  track.start_x_mm = 128.0F;
  track.start_y_mm = 128.0F;
  track.arena_width_mm = 256.0F;
  track.arena_height_mm = 256.0F;
  track.image_width = 256;
  track.image_height = 256;
  track.grayscale.resize(std::size_t{256} * 256U);
  for (const float heading : {0.0F, 1.57079633F, 3.14159265F, -1.57079633F}) {
    track.start_yaw_rad = heading;
    for (std::size_t y = 0; y < 256; ++y) {
      for (std::size_t x = 0; x < 256; ++x) {
        const float left = (static_cast<float>(x) - 128.0F) * std::sin(heading) -
                           (static_cast<float>(y) - 128.0F) * std::cos(heading);
        track.grayscale[y * 256U + x] = left > 0.0F ? 0 : 255;
      }
    }
    lf::sim::WorldPhysics world;
    world.reset(track);
    const auto readings = world.sense(0, track);
    for (std::size_t i = 0; i < lf::kIrSensorCount; ++i) {
      const bool body_left = lf::sim::kSimulatedRobot.ir_positions_mm[i].y > 0.0F;
      const bool dark = readings.ir[i].value < 1.0F;
      require(dark == body_left, "body-left IR channels sample the actual left half-plane");
    }
  }
}

void verify_response_and_braking() {
  const auto track = empty_track();
  lf::sim::WorldPhysics world;
  world.reset(track);
  world.advance({1.0F, 1.0F, 2000, lf::MotorMode::kDrive}, 0, 2000, track);
  require(world.pose().speed_mm_s > 0.0F && world.pose().speed_mm_s < 10.0F,
          "a motor must accelerate from rest, not jump to its steady speed");
  world.advance({1.0F, 1.0F, 1000000, lf::MotorMode::kDrive}, 2000, 1000000, track);
  require(std::abs(world.pose().speed_mm_s - 500.0F) < 0.1F,
          "nominal motor converges to its declared steady speed");
  auto coast = world;
  const float before_x = world.pose().x_mm;
  world.advance({0.0F, 0.0F, 1010000, lf::MotorMode::kBrake}, 1000000, 1010000, track);
  require(world.pose().x_mm > before_x && world.pose().speed_mm_s > 0.0F,
          "braking preserves residual travel and takes finite time");
  world.advance({0.0F, 0.0F, 1200000, lf::MotorMode::kBrake}, 1010000, 1200000, track);
  const float brake_travel = world.pose().x_mm - before_x;
  require(world.pose().speed_mm_s == 0.0F && brake_travel > 30.0F && brake_travel < 33.0F,
          "nominal braking distance agrees with v squared over twice deceleration");
  coast.advance({0.0F, 0.0F, 1200000, lf::MotorMode::kCoast}, 1000000, 1200000, track);
  require(coast.pose().x_mm - before_x > 90.0F && coast.pose().speed_mm_s > 440.0F,
          "coasting has finite drag and travels farther than braking");

  auto split = coast;
  auto whole = coast;
  const lf::MotorOutput expires{0.6F, 0.6F, 1201000, lf::MotorMode::kDrive};
  whole.advance(expires, 1200000, 1202000, track);
  split.advance(expires, 1200000, 1201000, track);
  const lf::MotorOutput disabled;
  split.advance(disabled, 1201000, 1202000, track);
  require(std::abs(whole.pose().x_mm - split.pose().x_mm) < 1e-3F &&
            std::abs(whole.pose().speed_mm_s - split.pose().speed_mm_s) < 1e-3F,
          "deadline inside a tick splits active drive from passive residual motion");
}

void verify_contact_and_signed_encoders() {
  auto track = empty_track();
  track.obstacles.assign(1, {1300.0F, 1000.0F, 205.0F, 95.0F, 0.0F});
  lf::sim::WorldPhysics world;
  world.reset(track);
  world.advance({1.0F, 1.0F, 1000000, lf::MotorMode::kDrive}, 0, 1000000, track);
  const auto before = world.sense(1000000, track);
  const auto pose = world.pose();
  world.advance({1.0F, 1.0F, 2000000, lf::MotorMode::kDrive}, 1000000, 2000000, track);
  const auto after = world.sense(2000000, track);
  require(world.blocked() && std::abs(world.pose().x_mm - pose.x_mm) < 0.5F,
          "contact blocks chassis displacement");
  require(after.wheel_counters[0].value - before.wheel_counters[0].value > 100U,
          "a blocked chassis can have spinning wheels and advancing encoders");
  world.advance({1.0F, 0.0F, 2200000, lf::MotorMode::kDrive}, 2000000, 2200000, track);
  require(!world.collided(track), "collision rollback includes yaw as well as translation");

  track.obstacles.clear();
  world.reset(track);
  world.advance({-1.0F, -1.0F, 1000000, lf::MotorMode::kDrive}, 0, 1000000, track);
  const auto reverse = world.sense(1000000, track);
  require(world.pose().x_mm < track.start_x_mm && reverse.wheel_counters[0].value > 0x80000000U,
          "reverse wheel rotation is signed travel represented through unsigned counter wrap");
}

void verify_delayed_acquisitions() {
  auto track = empty_track();
  track.obstacles.assign(1, {1300.0F, 1000.0F, 205.0F, 95.0F, 0.0F});
  lf::sim::WorldParameters parameters;
  parameters.ir.delay_us = 2000;
  parameters.encoders.delay_us = 2000;
  parameters.tof.delay_us = 4000;
  lf::sim::WorldPhysics world(lf::sim::simulated_calibration(), parameters);
  world.reset(track);
  const auto initial = world.sense(0, track);
  require(initial.ir[0].status == lf::ReadingStatus::kMissing &&
            initial.tof[0].status == lf::ReadingStatus::kMissing,
          "a delivery delay does not fabricate a fresh initial acquisition");
  for (std::uint64_t time = 2000; time <= 20000; time += 2000) {
    world.advance({0.5F, 0.5F, time, lf::MotorMode::kDrive}, time - 2000, time, track);
    (void)world.sense(time, track);
  }
  const auto acquired = world.sense(20000, track);
  require(acquired.tof[0].status == lf::ReadingStatus::kMissing &&
            acquired.ir[0].time_us == 18000 && acquired.wheel_counters[0].time_us == 18000,
          "delivery time is independent of acquisition time for every sensor source");
  track.obstacles.front().x_mm += 200.0F;
  (void)world.sense(22000, track);
  const auto delivered = world.sense(24000, track);
  require(delivered.tof[0].status == lf::ReadingStatus::kValid &&
            delivered.tof[0].time_us == 20000 && delivered.tof[0].sequence == 1 &&
            delivered.tof[0].value < 150.0F,
          "a delayed ToF reports the old observed surface, not the world at delivery");
  const auto reused = world.sense(24000, track);
  require(reused.ir[0].sequence == delivered.ir[0].sequence &&
            reused.tof[0].sequence == delivered.tof[0].sequence,
          "re-reading at the same clock does not reacquire or relabel samples");
  world.reset(track);
  const auto reset = world.sense(1000000, track);
  require(reset.ir[0].status == lf::ReadingStatus::kMissing &&
            reset.tof[0].status == lf::ReadingStatus::kMissing,
          "reposition clears all pending and delivered samples from the previous run");
  const auto next = world.sense(1002000, track);
  require(next.ir[0].time_us == 1000000 && next.ir[0].sequence == 1,
          "a new run has only acquisitions from its new sensor origin");
}

void verify_delayed_turn() {
  auto track = empty_track();
  track.obstacles.assign(1, {1300.0F, 1000.0F, 205.0F, 95.0F, 0.0F});
  const auto calibration = lf::sim::simulated_calibration();
  lf::sim::WorldParameters parameters;
  parameters.tof.delay_us = 4000;
  lf::sim::WorldPhysics delayed(calibration, parameters);
  lf::sim::WorldPhysics reference(calibration);
  delayed.reset(track);
  reference.reset(track);
  (void)delayed.sense(0, track);
  (void)reference.sense(0, track);
  lf::SensorSample<float> at_acquisition;
  for (std::uint64_t time = 2000; time <= 24000; time += 2000) {
    const lf::MotorOutput turn{1.0F, 0.0F, time, lf::MotorMode::kDrive};
    delayed.advance(turn, time - 2000, time, track);
    reference.advance(turn, time - 2000, time, track);
    (void)delayed.sense(time, track);
    const auto packet = reference.sense(time, track);
    if (time == 20000) {
      at_acquisition = packet.tof[0];
    }
  }
  const auto delivered = delayed.sense(24000, track).tof[0];
  require(delivered.status == lf::ReadingStatus::kValid && delivered.time_us == 20000 &&
            delivered.value == at_acquisition.value && delayed.pose().yaw_rad > 0.0F,
          "a delayed return during turning retains its acquisition-pose surface distance");
}

void verify_outages_and_no_return() {
  auto track = empty_track();
  lf::sim::WorldParameters parameters;
  parameters.tof.dropout_begin_us = 40000;
  parameters.tof.dropout_end_us = 80000;
  lf::sim::WorldPhysics world(lf::sim::simulated_calibration(), parameters);
  world.reset(track);
  (void)world.sense(0, track);
  const auto first = world.sense(20000, track);
  require(first.tof[0].status == lf::ReadingStatus::kNoReturn && first.tof[0].value == 4000.0F,
          "no return is distinct from a measured surface and retains the fixture range limit");
  const auto outage = world.sense(60000, track);
  require(outage.tof[0].time_us == first.tof[0].time_us &&
            outage.tof[0].sequence == first.tof[0].sequence &&
            outage.ir[0].sequence > first.ir[0].sequence,
          "a channel outage retains its old sample while other channels keep acquiring");
  const auto resumed = world.sense(80000, track);
  require(resumed.tof[0].sequence == first.tof[0].sequence + 1 && resumed.tof[0].time_us == 80000,
          "recovery does not fabricate acquisitions from the missing interval");
  const auto backwards = world.sense(79000, track);
  require(backwards.ir[0].status == lf::ReadingStatus::kInvalid,
          "a backwards acquisition clock is rejected without mutating the stream");
  require(world.sense(80000, track).tof[0].sequence == resumed.tof[0].sequence,
          "a rejected clock does not consume a sequence");
  track.obstacles.assign(1, {5082.5F, 1000.0F, 20.0F, 20.0F, 0.0F});
  const auto boundary = world.sense(100000, track);
  require(boundary.tof[0].status == lf::ReadingStatus::kValid && boundary.tof[0].value == 4000.0F,
          "a hit at the range boundary is not confused with a no-return scalar of equal value");
}

void verify_noise_and_traction() {
  auto track = empty_track();
  track.image_width = 32;
  track.image_height = 32;
  track.grayscale.assign(std::size_t{32} * 32U, 128);
  track.obstacles.assign(1, {1300.0F, 1000.0F, 205.0F, 95.0F, 0.0F});
  const auto calibration = lf::sim::simulated_calibration();
  lf::sim::WorldParameters parameters;
  parameters.ir_noise_fraction = 0.02F;
  parameters.tof_noise_mm = 5.0F;
  parameters.noise_seed = 17;
  lf::sim::WorldPhysics noisy(calibration, parameters);
  lf::sim::WorldPhysics replay(calibration, parameters);
  lf::sim::WorldPhysics nominal(calibration);
  noisy.reset(track);
  replay.reset(track);
  nominal.reset(track);
  for (std::uint64_t time = 0; time <= 100000; time += 2000) {
    const auto a = noisy.sense(time, track);
    const auto b = replay.sense(time, track);
    const auto reference = nominal.sense(time, track);
    require(a.ir[0].value == b.ir[0].value && a.tof[0].value == b.tof[0].value,
            "seeded noise replays deterministically");
    require(std::abs(a.ir[0].value - reference.ir[0].value) <= 5.1001F &&
              std::abs(a.tof[0].value - reference.tof[0].value) <= 5.0001F,
            "noise stays inside its declared IR full-scale and ToF millimetre bounds");
    require(noisy.sense(time, track).ir[0].value == a.ir[0].value,
            "delivery/re-reading does not draw noise again");
  }
  track.obstacles.clear();
  parameters.traction = {0.85F, 0.85F};
  lf::sim::WorldPhysics slipping(calibration, parameters);
  slipping.reset(track);
  slipping.advance({1.0F, 1.0F, 1000000, lf::MotorMode::kDrive}, 0, 1000000, track);
  const auto encoders = slipping.sense(1000000, track);
  const auto wheel_travel = static_cast<float>(encoders.wheel_counters[0].value);
  const float body_travel = slipping.pose().x_mm - track.start_x_mm;
  require(std::abs(body_travel / wheel_travel - 0.85F) < 0.005F,
          "traction mismatch changes body motion but not axle counters");
}

void verify_queue_boundary() {
  const auto track = empty_track();
  lf::sim::WorldParameters parameters;
  parameters.ir.delay_us = parameters.ir.period_us * (lf::sim::kSensorQueueCapacity - 1U);
  lf::sim::WorldPhysics world(lf::sim::simulated_calibration(), parameters);
  world.reset(track);
  for (std::uint64_t time = 0; time <= 100000; time += parameters.ir.period_us) {
    const auto sample = world.sense(time, track).ir[0];
    if (time < parameters.ir.delay_us) {
      require(sample.status == lf::ReadingStatus::kMissing,
              "a full-delay queue does not publish samples early");
    } else {
      require(sample.status == lf::ReadingStatus::kValid &&
                sample.time_us == time - parameters.ir.delay_us &&
                sample.sequence == (time - parameters.ir.delay_us) / parameters.ir.period_us + 1U,
              "the largest supported delay preserves every acquisition through queue wrap");
    }
  }
}

void verify_parameter_bounds() {
  auto parameters = lf::sim::WorldParameters{};
  require(lf::sim::validate_world_parameters(parameters), "nominal plant is valid");
  parameters.ir.period_us = 0;
  require(!lf::sim::validate_world_parameters(parameters), "zero acquisition period rejected");
  parameters = {};
  parameters.ir.delay_us = parameters.ir.period_us * lf::sim::kSensorQueueCapacity;
  require(!lf::sim::validate_world_parameters(parameters),
          "delivery delay beyond the fixed queue contract rejected before acquisition");
  lf::sim::Simulation invalid(lf::sim::simulated_calibration(), parameters);
  const auto track = empty_track();
  invalid.use_track(track);
  require(!invalid.prepare_run(), "invalid plant configuration cannot start a run");
  parameters = {};
  parameters.max_wheel_speed_mm_s = std::numeric_limits<float>::max();
  const lf::sim::WorldPhysics too_fast(lf::sim::simulated_calibration(), parameters);
  require(!too_fast.parameters_valid(),
          "a plant that exceeds integration/counter resolution is rejected before motion");
}

}  // namespace

int main() {
  verify_right_turn();
  verify_ir_handedness();
  verify_response_and_braking();
  verify_contact_and_signed_encoders();
  verify_delayed_acquisitions();
  verify_delayed_turn();
  verify_outages_and_no_return();
  verify_noise_and_traction();
  verify_queue_boundary();
  verify_parameter_bounds();
  std::cout << "sim physics: direction/motors/contact/acquisition/noise/traction/config PASS\n";
}
