#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <robot_geometry.hpp>
#include <simulation.hpp>
#include <string>
#include <track_asset.hpp>
#include <world_physics.hpp>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

bool nearly_equal(float a, float b) {
  return std::abs(a - b) < 1e-3F;
}

void verify_assets() {
  lf::sim::TrackAsset track;
  std::string error;
  const bool loaded = track.load("assets/track_vongloai.json", error);
  if (!loaded) {
    std::cerr << "track_vongloai load: [" << error << "]\n";
    std::abort();
  }
  require(track.image_width > 0 && track.image_height > 0, "PNG decoded without Raylib");
  require(track.grayscale.size() == static_cast<std::size_t>(track.image_width) *
                                      static_cast<std::size_t>(track.image_height),
          "bounded grayscale pixels available for renderer upload");
  require(track.obstacles.size() == 2, "manifest obstacle count");
  require(track.obstacles.front().width_mm == 105.0F,
          "fixture geometry is imported, not BOM truth");
  const auto original_name = track.name;
  require(!track.load("assets/does-not-exist.json", error), "missing manifest rejected");
  require(track.name == original_name, "failed load preserves current track");
}

void verify_sensor_positions() {
  lf::sim::TrackAsset track;
  track.image_width = 256;
  track.image_height = 256;
  track.arena_width_mm = 256.0F;
  track.arena_height_mm = 256.0F;
  track.start_x_mm = 128.0F;
  track.start_y_mm = 128.0F;
  track.grayscale.resize(std::size_t{256} * 256U);
  for (unsigned y = 0; y < 256; ++y) {
    for (unsigned x = 0; x < 256; ++x) {
      track.grayscale[y * 256U + x] = static_cast<std::uint8_t>((x + 3U * y) % 256U);
    }
  }
  const auto& positions = lf::sim::kSimulatedRobot.ir_positions_mm;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    const auto& position = positions[index];
    const auto& opposite = positions[positions.size() - 1U - index];
    require(std::abs(position.x - opposite.x) < 1e-4F && std::abs(position.y + opposite.y) < 1e-4F,
            "IR geometry is symmetric and retains channel order");
    require(std::abs(position.x) <= lf::sim::kSimulatedRobot.half_length_mm &&
              std::abs(position.y) <= lf::sim::kSimulatedRobot.half_width_mm,
            "IR geometry fits inside the simulated chassis");
  }
  for (const float heading : {0.0F, 1.57079633F, 3.14159265F, -1.57079633F}) {
    track.start_yaw_rad = heading;
    lf::sim::WorldPhysics world;
    world.reset(track);
    const auto sensors = world.sense(2000, track);
    for (std::size_t index = 0; index < positions.size(); ++index) {
      const auto& position = positions[index];
      const float x =
        track.start_x_mm + position.x * std::cos(heading) + position.y * std::sin(heading);
      const float y =
        track.start_y_mm + position.x * std::sin(heading) - position.y * std::cos(heading);
      // Channel values come from an analog spot, not one pixel: assert the
      // fused value stays inside the neighbourhood it integrated.
      float lowest = 255.0F;
      float highest = 0.0F;
      const float radius = lf::sim::kSimulatedRobot.ir_spot_radius_mm;
      for (int row = -4; row <= 4; ++row) {
        for (int column = -4; column <= 4; ++column) {
          const float dx = static_cast<float>(column) * radius / 4.0F;
          const float dy = static_cast<float>(row) * radius / 4.0F;
          if (dx * dx + dy * dy > radius * radius) {
            continue;
          }
          const auto sample = static_cast<float>(track.sample(x + dx, y + dy));
          lowest = std::min(lowest, sample);
          highest = std::max(highest, sample);
        }
      }
      require(sensors.ir[index].value >= lowest && sensors.ir[index].value <= highest,
              "IR channel is fused over its spot neighbourhood at the curved pose");
    }
  }
}

void verify_world() {
  lf::sim::TrackAsset track;
  std::string error;
  if (!track.load("assets/track_chungket.json", error)) {
    std::cerr << "track_chungket load: " << error << '\n';
    std::abort();
  }
  track.obstacles.assign(1,
                         {track.start_x_mm + 800.0F, track.start_y_mm, 205.0F, 95.0F, 0.52359878F});
  lf::sim::WorldPhysics world;
  world.reset(track);
  auto readings = world.sense(0, track);
  require(readings.tof[0].status == lf::ReadingStatus::kMissing,
          "front ToF starts unavailable until its sampling period");
  readings = world.sense(20000, track);
  require(readings.tof[0].status == lf::ReadingStatus::kValid,
          "front ToF updates at its independent 50 Hz cadence");
  require(readings.tof[0].value > 0.0F && readings.tof[0].value < 4000.0F,
          "front range intersects a rotated-box surface");
  require(readings.ir[0].sequence == 2 && readings.ir[7].sequence == 2,
          "curved IR channels retain independent sequence identity");
  const auto first_tof = readings.tof[0];
  track.obstacles.front().x_mm += 200.0F;
  const auto reused = world.sense(22000, track).tof[0];
  require(reused.value == first_tof.value && reused.sequence == first_tof.sequence &&
            reused.time_us == first_tof.time_us,
          "reused ToF sample keeps its value, identity, and acquisition time");
  const auto updated = world.sense(40000, track).tof[0];
  require(updated.sequence == first_tof.sequence + 1 && updated.value != first_tof.value,
          "ToF sees the changed world only at its next acquisition");
  track.obstacles.clear();
  const auto clear_range = world.sense(60000, track).tof[0];
  require(nearly_equal(clear_range.value, lf::sim::kSimulatedRobot.tof_range_mm),
          "a clear ToF field ends at the finite configured range");
  const float sensor_x = track.start_x_mm + lf::sim::kSimulatedRobot.tof_forward_mm;
  const float range_limit = lf::sim::kSimulatedRobot.tof_range_mm;
  track.obstacles.assign(1,
                         {sensor_x + range_limit + 100.0F, track.start_y_mm, 20.0F, 20.0F, 0.0F});
  require(nearly_equal(world.sense(80000, track).tof[0].value, range_limit),
          "ToF ignores obstacles beyond its range");
  track.obstacles.front().x_mm = sensor_x + 510.0F;
  require(nearly_equal(world.sense(100000, track).tof[0].value, 500.0F),
          "ToF reports surface distance from the sensor, in millimetres");
  const auto untouched = world.pose();
  const lf::MotorOutput expired{1.0F, 1.0F, 1000, lf::MotorMode::kDrive};
  world.advance(expired, 10000, 12000, track);
  require(world.pose().x_mm == untouched.x_mm, "expired output produces no forward displacement");
  track.obstacles.assign(1, {track.start_x_mm, track.start_y_mm, 205.0F, 95.0F, 0.78539816F});
  require(world.collided(track), "oriented brick contacts the robot footprint");

  // Separation exists on the axes the old two-axis test never projected onto.
  lf::sim::TrackAsset separated;
  separated.start_x_mm = 500.0F;
  separated.start_y_mm = 500.0F;
  separated.start_yaw_rad = 0.0F;
  separated.obstacles.assign(1, {500.0F, 1000.0F, 100.0F, 100.0F, 0.0F});
  lf::sim::WorldPhysics clear_world;
  clear_world.reset(separated);
  require(!clear_world.collided(separated),
          "a box far along y does not collide with an unrotated robot");
  separated.obstacles.assign(1, {500.0F, 571.0F, 100.0F, 100.0F, 0.0F});
  require(clear_world.collided(separated), "overlapping footprints still collide");
}

void verify_referee() {
  lf::sim::TrackAsset track;
  track.finish_min_x_mm = 520.0F;
  track.finish_max_x_mm = 530.0F;
  track.finish_min_y_mm = -1000.0F;
  track.finish_max_y_mm = 1000.0F;
  lf::sim::Referee referee;
  referee.start_run(1000, 1);
  const auto early = referee.check(track, {445.0F, 0.0F, 0.0F, 300.0F}, 2000);
  require(early.verdict == lf::sim::VerdictKind::kPending,
          "a footprint outside the finish gate is not completed");
  const auto crossed = referee.check(track, {450.0F, 0.0F, 0.0F, 300.0F}, 3000);
  require(crossed.verdict == lf::sim::VerdictKind::kCompleted,
          "finish contact completes immediately, without a minimum travelled distance");
  require(crossed.elapsed_us == 2000 && crossed.run == 1,
          "referee measures from its own start time and run identity");
  const auto after_finish = referee.check(track, {600.0F, 0.0F, 0.0F, 300.0F}, 4000);
  require(
    after_finish.verdict == lf::sim::VerdictKind::kCompleted && after_finish.elapsed_us == 2000,
    "a terminal verdict is never rewritten by later poses");
}

void verify_tuning_config() {
  require(lf::validate_calibration(lf::sim::simulated_calibration()),
          "the simulated calibration is valid");
  require(lf::validate_race_tuning(lf::RaceTuning{}), "the default tuning plan is valid");
  require(!lf::validate_race_tuning([](lf::RaceTuning t) {
    t.robot.slip_curvature[2] = t.robot.slip_curvature[1];
    return t;
  }(lf::RaceTuning{})),
          "validation rejects a non-ascending slip table");
  require(!lf::validate_race_tuning([](lf::RaceTuning t) {
    t.track.clearance_margin = -1.0F;
    return t;
  }(lf::RaceTuning{})),
          "validation rejects a negative clearance margin");
}

void verify_engine() {
  lf::sim::TrackAsset track;
  std::string error;
  require(track.load("assets/track_vongloai.json", error), error.c_str());
  lf::sim::Simulation simulation;
  simulation.use_track(track);
  const auto start = simulation.observe();
  require(!simulation.running() && simulation.run() == 0,
          "a fresh session is idle at its first observation");
  require(nearly_equal(start.pose.x_mm, track.start_x_mm), "world starts at the start gate");
  (void)simulation.advance({});
  require(simulation.observe().time_us == lf::sim::kControlStepUs,
          "idle advance does not run the clock");

  require(simulation.prepare_run(), "prepare requires a bound track");
  const auto prepared = simulation.observe();
  simulation.start_run();
  require(simulation.running() && simulation.run() == 1, "start commits the prepared run");

  const std::uint64_t t0 = prepared.time_us;
  const lf::MotorOutput nan{
    std::numeric_limits<float>::quiet_NaN(),
    0.5F,
    t0 + 2000,
    lf::MotorMode::kDrive,
  };
  const auto rejected = simulation.advance(nan);
  require(rejected.time_us == t0 + 2000, "an accepted tick advances the logical clock once");
  require(nearly_equal(rejected.pose.x_mm, prepared.pose.x_mm),
          "non-finite duty is treated as a stopped motor, not as motion");

  const lf::MotorOutput forward{1.0F, 1.0F, rejected.time_us + 2000, lf::MotorMode::kDrive};
  const auto moved = simulation.advance(forward);
  require(moved.pose.x_mm > rejected.pose.x_mm, "a drive output moves the simulated robot");
  require(moved.measurements.time_us == moved.time_us,
          "published measurements carry the observed time");
  const auto sequence = moved.measurements.ir[0].sequence;
  (void)simulation.observe();
  require(simulation.observe().measurements.ir[0].sequence == sequence,
          "reading the snapshot does not acquire new samples");
  require(simulation.observe().tick == moved.tick, "reading the snapshot does not run a tick");

  simulation.stop_run();
  require(!simulation.running(), "stop ends the run");
  require(simulation.result().verdict == lf::sim::VerdictKind::kDnf,
          "an unjudged stopped run is a DNF, not a completion");
  const auto stopped_time = simulation.observe().time_us;

  require(simulation.prepare_run(), "a second run can be prepared after a verdict");
  require(simulation.observe().time_us == stopped_time + lf::sim::kControlStepUs,
          "preparing a run keeps the session clock monotonic and fresh");
  require(nearly_equal(simulation.observe().pose.x_mm, track.start_x_mm),
          "preparing a run repositions the world at the start gate");
  require(simulation.result().verdict == lf::sim::VerdictKind::kPending,
          "preparing a run clears the previous verdict");
  simulation.start_run();
  require(simulation.run() == 2, "the second start is run two");
  require(simulation.observe().measurements.wheel_counters[0].value == 0U,
          "the repositioned world restarts its wheel counters");
}

}  // namespace

int main() {
  verify_assets();
  verify_tuning_config();
  verify_sensor_positions();
  verify_world();
  verify_referee();
  verify_engine();
  std::cout << "sim runtime: assets/sensors/world/referee/engine contracts PASS\n";
  return 0;
}
