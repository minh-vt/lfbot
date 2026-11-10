#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <drive.hpp>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <robot_geometry.hpp>
#include <route.hpp>
#include <track_asset.hpp>
#include <world_physics.hpp>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

lf::RobotTuning exact_wheels() {
  lf::RobotTuning tuning;
  tuning.wheel_travel_error = 0.0F;
  return tuning;
}

lf::SensorReadings wheels(float left, float right, float elapsed, std::uint32_t sequence) {
  lf::SensorReadings readings;
  readings.elapsed_time = elapsed;
  readings.wheel_positions[0] = {left, 0.0F, sequence, lf::ReadingStatus::kValid};
  readings.wheel_positions[1] = {right, 0.0F, sequence, lf::ReadingStatus::kValid};
  return readings;
}

void verify_unobserved_motion_profile() {
  lf::RaceGeometry geometry;
  geometry.wheel_resolution = 0.001F;
  geometry.wheel_acceleration = 2.208F;
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  const float pi = std::acos(-1.0F);
  route.record_readings(wheels(pi, pi, 2.0F * pi, 2));
  const auto& pose = route.locate_robot();
  // Right rolls pi, then left rolls pi: two half-circles, endpoint (0,2).
  // Each phase can use 1-cos(2t): continuous velocity, |acceleration| <= 2.
  require(std::hypot(pose.position.x, pose.position.y - 2.0F) <= pose.position_error,
          "same endpoint counts do not imply a constant-curvature motion profile");
}

void verify_delivery_jitter() {
  const lf::RaceGeometry geometry;
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  auto packet = wheels(0.0F, 0.0F, 1.0F, 1);
  for (auto& wheel : packet.wheel_positions) {
    wheel.age = 1.0F;
  }
  route.record_readings(packet);
  packet = wheels(1.0F, 1.0F, 1.0F, 2);
  for (auto& wheel : packet.wheel_positions) {
    wheel.age = 1.0F;
  }
  route.record_readings(packet);
  packet = wheels(2.0F, 2.0F, 0.5F, 3);
  for (auto& wheel : packet.wheel_positions) {
    wheel.age = 0.5F;
  }
  route.record_readings(packet);
  packet = wheels(3.0F, 3.0F, 1.0F, 4);
  for (auto& wheel : packet.wheel_positions) {
    wheel.age = 0.5F;
  }
  route.record_readings(packet);
  require(route.locate_robot().time == 3.0 && route.locate_robot().position.x == 3.0F,
          "delivery jitter cannot permanently stop contiguous acquired wheel epochs");
}

void verify_motion() {
  const lf::RaceGeometry geometry;
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  require(route.has_position(), "initial acquisition anchors a local body frame");
  route.record_readings(wheels(1.0F, 1.0F, 1.0F, 2));
  require(std::abs(route.locate_robot().position.x - 1.0F) < 1e-6F &&
            route.locate_robot().position.y == 0.0F,
          "equal wheel travel moves forward in the estimated body chart");
  route.record_readings(wheels(1.5F, 2.5F, 1.0F, 3));
  const auto& pose = route.locate_robot();
  require(std::abs(pose.position.x - 1.0F - std::sin(1.0F)) < 1e-6F &&
            std::abs(pose.position.y - 1.0F + std::cos(1.0F)) < 1e-6F &&
            std::abs(pose.heading - 1.0F) < 1e-6F,
          "right wheel faster gives the exact positive-left circular arc");
  const auto motion = route.motion();
  require(motion.distance == 1.0F && motion.angle == 1.0F,
          "motion is reported once in dimensionless length and radians");
  const auto past = route.locate_robot(1.0);
  require(past.has_value() && past->position.x == 1.0F && past->position.y == 0.0F,
          "turning later does not attach a past sensor to the current pose");
  require(past.has_value() && past->position_error == 0.0F && past->heading_error == 0.0F,
          "an exact stored acquisition does not acquire interpolation uncertainty");
  require(!route.locate_robot(1.5).has_value(),
          "an unacquired epoch is unsupported without an additional motion model");
  require(!route.locate_robot(2.01).has_value() && !route.locate_robot(-0.01).has_value(),
          "the estimator does not fabricate a future or pre-run acquisition pose");
}

void verify_asynchronous_wheels() {
  const lf::RaceGeometry geometry;
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  auto readings = wheels(1.0F, 0.0F, 1.0F, 2);
  readings.wheel_positions[1] = {0.0F, 1.0F, 1, lf::ReadingStatus::kValid};
  route.record_readings(readings);
  require(route.locate_robot().time == 0.0 && route.motion().distance == 0.0F,
          "a new left encoder cannot turn a reused right encoder into simultaneous motion");
  readings = wheels(2.0F, 1.0F, 1.0F, 3);
  readings.wheel_positions[1] = {1.0F, 1.0F, 2, lf::ReadingStatus::kValid};
  route.record_readings(readings);
  require(route.locate_robot().time == 1.0 && route.locate_robot().position.x == 1.0F &&
            route.locate_robot().heading == 0.0F,
          "different delivery times are aligned at the actual shared acquisition epoch");
  readings = wheels(3.0F, 2.0F, 1.0F, 4);
  readings.wheel_positions[1] = {2.0F, 1.0F, 3, lf::ReadingStatus::kValid};
  route.record_readings(readings);
  require(route.locate_robot().time == 2.0 && route.locate_robot().position.x == 2.0F,
          "a delayed stream keeps progressing on its acquisition clock");

  auto bounded_geometry = geometry;
  bounded_geometry.wheel_acceleration = 1.0F;
  lf::Route unsupported(bounded_geometry, tuning);
  unsupported.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  readings = wheels(1.0F, 0.5F, 1.0F, 2);
  readings.wheel_positions[1].age = 0.5F;
  unsupported.record_readings(readings);
  require(unsupported.locate_robot().time == 0.0 && !unsupported.locate_robot(0.5).has_value(),
          "noncoincident wheel acquisitions do not imply constant velocity inside their intervals");
  unsupported.record_readings(wheels(2.0F, 2.0F, 1.0F, 3));
  require(unsupported.locate_robot(2.0).has_value() &&
            unsupported.locate_robot().profile_error > 0.0F &&
            !unsupported.locate_robot(1.0).has_value(),
          "a recovered endpoint carries profile uncertainty, not a fabricated missing acquisition");
}

void verify_reposition_and_missing_origin() {
  const lf::RaceGeometry geometry;
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  route.record_readings(wheels(1.0F, 1.0F, 1.0F, 2));
  route.prepare_run();
  require(!route.has_position() && !route.locate_robot(1.0).has_value(),
          "reposition clears prior-run wheel and pose history");
  auto readings = wheels(1.0F, 1.0F, 20.0F, 1);
  readings.wheel_positions[0].age = 0.1F;
  readings.wheel_positions[1].age = 0.1F;
  route.record_readings(readings);
  require(!route.has_position() && route.rejected_samples() == 2,
          "the new run excludes delayed acquisitions from before reposition");
  route.record_readings(wheels(1.0F, 1.0F, 0.1F, 2));
  require(!route.has_position(),
          "missing the initial origin cannot be repaired by assuming later motion was zero");
  route.prepare_run();
  route.record_readings(wheels(100.0F, 100.0F, 20.0F, 3));
  route.record_readings(wheels(101.0F, 101.0F, 1.0F, 4));
  require(route.locate_robot().position.x == 1.0F && route.locate_robot().wheel_position == 101.0F,
          "reposition rebases the chart without assuming cumulative axle positions reset");
  route.reset();
  require(!route.has_position(), "full reset clears the live estimate");
}

void verify_bounds_and_history() {
  lf::RaceGeometry geometry;
  geometry.wheel_resolution = 0.001F;
  const lf::RobotTuning tuning;
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, std::numeric_limits<std::uint32_t>::max()));
  route.record_readings(wheels(1.0F, 1.0F, 1.0F, 0));
  const float position_error = route.locate_robot().position_error;
  require(position_error >= 0.15F && route.locate_robot().heading_error >= 0.3F,
          "free-rolling traction stress is represented in the conditional error budget");
  route.record_readings(wheels(1.0F, 1.0F, 1.0F, 1));
  require(route.locate_robot().position_error == position_error,
          "stationary samples do not re-add the same endpoint quantization error");
  for (std::uint32_t i = 2; i < 100; ++i) {
    route.record_readings(wheels(static_cast<float>(i), static_cast<float>(i), 1.0F, i));
  }
  require(!route.locate_robot(0.0).has_value() && route.locate_robot(98.0).has_value(),
          "expired pose history is rejected instead of aliased to the first or last entry");
  const auto old = route.locate_robot();
  auto readings = wheels(200.0F, 200.0F, 0.0F, 101);
  readings.wheel_positions[0].age = 90.0F;
  readings.wheel_positions[1].age = 90.0F;
  route.record_readings(readings);
  require(route.locate_robot().position.x == old.position.x && route.rejected_samples() == 2,
          "out-of-order acquisitions cannot rewrite the pose history");
  readings = wheels(200.0F, 200.0F, 0.0F, 102);
  readings.wheel_positions[0].age = std::numeric_limits<float>::quiet_NaN();
  readings.wheel_positions[1].value = std::numeric_limits<float>::infinity();
  route.record_readings(readings);
  require(route.rejected_samples() == 4, "nonfinite encoder observations are rejected");
}

void verify_reverse_and_wrap() {
  const lf::RaceGeometry geometry;
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  route.record_readings(wheels(-1.0F, -1.0F, 1.0F, 2));
  require(route.locate_robot().position.x == -1.0F && route.locate_robot().wheel_distance == 1.0F,
          "reverse position and absolute axle distance are different facts");
  route.record_readings(wheels(-2.5F, 0.5F, 1.0F, 3));
  route.record_readings(wheels(-2.75F, 0.75F, 1.0F, 4));
  const auto after_wrap = route.locate_robot(3.0);
  const float expected_heading = std::remainder(3.5F, 2.0F * std::acos(-1.0F));
  require(after_wrap.has_value() && std::abs(after_wrap->heading - expected_heading) < 1e-6F,
          "heading wrapping retains the acquired orientation across pi");
}

void verify_travel_budget() {
  lf::RaceTuning tuning;
  require(lf::validate_race_tuning(tuning), "the declared fixture travel budget is valid");
  for (const float invalid : {-0.01F, 1.01F, std::numeric_limits<float>::quiet_NaN()}) {
    tuning.robot.wheel_travel_error = invalid;
    require(!lf::validate_race_tuning(tuning), "travel-error fraction must be finite in [0,1]");
  }
}

void verify_against_plant(float heading, float traction) {
  lf::sim::TrackAsset track;
  track.arena_width_mm = 10000.0F;
  track.arena_height_mm = 10000.0F;
  track.start_x_mm = 5000.0F;
  track.start_y_mm = 5000.0F;
  track.start_yaw_rad = heading;
  const auto calibration = lf::sim::simulated_calibration();
  lf::sim::WorldParameters parameters;
  parameters.encoders.delay_us = 2000;
  parameters.traction = {traction, traction};
  lf::sim::WorldPhysics world(calibration, parameters);
  world.reset(track);
  lf::Drive drive(calibration);
  lf::RaceGeometry geometry;
  geometry.wheel_resolution = calibration.wheel_mm_per_tick / calibration.wheelbase_mm;
  geometry.wheel_acceleration =
    std::max(calibration.drive_acceleration_mm_s2, calibration.brake_deceleration_mm_s2) *
    calibration.wheelbase_mm /
    (calibration.reference_speed_mm_s * calibration.reference_speed_mm_s);
  geometry.ir_interval =
    calibration.ir_period_s * calibration.reference_speed_mm_s / calibration.wheelbase_mm;
  auto tuning = exact_wheels();
  tuning.wheel_travel_error = 1.0F - traction;
  lf::Route route(geometry, tuning);
  lf::SensorReadings readings;
  auto fault = lf::StopReason::kNone;
  require(drive.read_sensors(world.sense(0, track), readings, fault), "initial sensor packet");
  route.record_readings(readings);
  for (std::uint64_t time = 2000; time <= 4000000; time += 2000) {
    const auto truth_at_acquisition = world.pose();
    const bool first_turn = time <= 2000000;
    const lf::MotorOutput output{
      first_turn ? 0.6F : 0.9F,
      first_turn ? 0.9F : 0.6F,
      time,
      lf::MotorMode::kDrive,
    };
    world.advance(output, time - 2000, time, track);
    require(drive.read_sensors(world.sense(time, track), readings, fault), "delayed sensor packet");
    route.record_readings(readings);
    require(route.has_position(), "the delayed initial epoch anchors the estimator");
    const auto& pose = route.locate_robot();
    const double acquired_time = static_cast<double>(time - 2000) * 1e-6 *
                                 calibration.reference_speed_mm_s / calibration.wheelbase_mm;
    require(std::abs(pose.time - acquired_time) <=
              4.0 * std::numeric_limits<float>::epsilon() * (1.0 + acquired_time),
            "the estimate is stamped at delayed acquisition, not current delivery");
    const float dx = truth_at_acquisition.x_mm - track.start_x_mm;
    const float dy = truth_at_acquisition.y_mm - track.start_y_mm;
    const float x = (dx * std::cos(heading) + dy * std::sin(heading)) / calibration.wheelbase_mm;
    const float y = (dx * std::sin(heading) - dy * std::cos(heading)) / calibration.wheelbase_mm;
    const float heading_error = std::abs(std::remainder(
      pose.heading + truth_at_acquisition.yaw_rad - heading, 2.0F * std::acos(-1.0F)));
    require(std::hypot(pose.position.x - x, pose.position.y - y) <= pose.position_error &&
              heading_error <= pose.heading_error,
            "free-rolling truth at acquisition stays inside the declared conditional bounds");
  }
}

lf::RaceGeometry map_geometry() {
  lf::RaceGeometry geometry;
  geometry.ir_spot_radius = 0.03F;
  geometry.ir_interval = 1.0F;  // synthetic acquisition cadence, not delivery cadence
  geometry.line_half_width = 0.07F;
  geometry.lookahead = 0.4F;
  geometry.tof_forward = 0.5F;
  geometry.tof_half_fov = 0.2F;
  geometry.tof_error = 0.04F;
  geometry.obstacle_radius = 0.9F;
  for (std::size_t i = 0; i < lf::kIrSensorCount; ++i) {
    geometry.ir_position[i] = {0.4F, (static_cast<float>(i) - 3.5F) * 0.06F};
  }
  return geometry;
}

void illuminate(lf::SensorReadings& readings, std::uint32_t sequence,
                std::initializer_list<std::size_t> black, float age = 0.0F) {
  for (auto& sample : readings.ir) {
    sample = {0.0F, age, sequence, lf::ReadingStatus::kValid};
  }
  for (const auto i : black) {
    readings.ir[i].value = 1.0F;
  }
}

void verify_line_map() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  auto readings = wheels(0.0F, 0.0F, 0.0F, 1);
  illuminate(readings, 1, {4, 5, 6});
  route.record_readings(readings);
  require(std::abs(route.latest_line().position.x - 0.4F) < 1e-6F &&
            std::abs(route.latest_line().position.y - 0.09F) < 1e-6F,
          "line evidence uses geometric spot coordinates, not a normalized coverage moment");
  readings = wheels(1.0F, 1.0F, 1.0F, 2);
  illuminate(readings, 1, {4, 5, 6}, 1.0F);
  route.record_readings(readings);
  require(
    route.coverage().line_acquisitions == 1 && route.line_size() == 1 && route.path_size() == 2,
    "wheel travel and repeated IR do not manufacture new observed line geometry");
  for (std::uint32_t i = 2; i <= 6; ++i) {
    readings = wheels(static_cast<float>(i), static_cast<float>(i), 1.0F, i + 1U);
    illuminate(readings, i, {});
    if (i == 3) {
      illuminate(readings, i, {0, 7});
    } else if (i == 4) {
      illuminate(readings, i, {0, 1, 2, 3, 4, 5, 6, 7});
    } else if (i >= 5) {
      illuminate(readings, i, {3, 4});
    }
    route.record_readings(readings);
    if (i == 2) {
      require(route.latest_line().evidence == lf::RouteEvidence::kGap,
              "no usable black response is a gap, not an extrapolated measured line");
    } else if (i <= 4) {
      require(route.latest_line().evidence == lf::RouteEvidence::kAmbiguous,
              "disjoint clusters and an all-black fan retain ambiguity");
    }
  }
  auto query = route.locate_robot();
  query.position = {2.4F, 0.0F};
  require(!route.locate_line(query).supported, "a hidden gap is not bridged as observed geometry");
  query.position = {5.9F, 0.0F};
  const auto match = route.locate_line(query);
  require(match.supported && match.begin_order >= 5.0F && route.coverage().has_gap &&
            route.coverage().has_ambiguity && route.coverage().incomplete,
          "line order, ambiguity and prefix coverage remain distinct from path distance");
}

void verify_pending_map() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  lf::SensorReadings readings;
  illuminate(readings, 1, {3, 4});
  route.record_readings(readings);
  require(route.coverage().line_acquisitions == 0, "IR waits for an encoder acquisition pose");
  readings = wheels(0.0F, 0.0F, 1.0F, 1);
  for (auto& wheel : readings.wheel_positions) {
    wheel.age = 1.0F;
  }
  illuminate(readings, 1, {3, 4}, 1.0F);
  route.record_readings(readings);
  require(route.coverage().line_acquisitions == 1 && route.latest_line().position.x == 0.4F,
          "pending IR uses its old epoch and is processed only once");
  readings = wheels(1.0F, 1.0F, 1.0F, 2);
  for (auto& wheel : readings.wheel_positions) {
    wheel.age = 1.0F;
  }
  illuminate(readings, 2, {3, 4});
  route.record_readings(readings);
  readings = wheels(2.0F, 2.0F, 1.0F, 3);
  for (auto& wheel : readings.wheel_positions) {
    wheel.age = 1.0F;
  }
  illuminate(readings, 2, {3, 4}, 1.0F);
  route.record_readings(readings);
  require(route.line_point(route.line_size() - 1U).break_before && route.coverage().has_gap,
          "missing acquisitions create a support break even when black returns later");
}

void verify_discarded_line_break() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  auto packet = wheels(0.0F, 0.0F, 0.0F, 1);
  illuminate(packet, 1, {3, 4});
  route.record_readings(packet);
  packet = wheels(1.0F, 1.0F, 1.0F, 2);
  illuminate(packet, 2, {3, 4}, 0.5F);
  route.record_readings(packet);
  packet = wheels(2.0F, 2.0F, 1.0F, 3);
  illuminate(packet, 3, {3, 4}, 1.0F);
  route.record_readings(packet);
  auto query = route.locate_robot();
  query.position = {0.9F, 0.0F};
  require(
    route.line_point(route.line_size() - 1U).break_before && !route.locate_line(query).supported,
    "a rejected pending IR epoch breaks the next retained line segment");
}

void verify_obstacle_map() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  auto readings = wheels(0.5F, 1.5F, 1.0F, 2);
  readings.tof[0] = {1.0F, 1.0F, 1, lf::ReadingStatus::kValid};
  route.record_readings(readings);
  require(route.obstacle_size() == 1, "one delayed range creates occupied evidence");
  const auto before = route.locate_obstacle(0, route.locate_robot());
  const float radius = 1.0F * std::tan(0.2F) + 0.04F;
  require(before.centre.x == 1.5F && before.centre.y == 0.0F && before.radius >= radius,
          "a delayed hit stays at its acquisition pose and covers the cone it measured");
  readings = wheels(0.5F, 1.5F, 1.0F, 3);
  readings.tof[0] = {10.0F, 0.0F, 2, lf::ReadingStatus::kNoReturn};
  route.record_readings(readings);
  require(route.obstacle_size() == 1 && route.coverage().range_acquisitions == 2,
          "a fresh no-return after leaving FOV does not delete occupied evidence");
  const auto retained = route.locate_obstacle(0, route.locate_robot());
  require(retained.centre.x == before.centre.x && retained.centre.y == before.centre.y,
          "turning the robot does not move a remembered world region");
  const auto count = route.coverage().range_acquisitions;
  readings = wheels(0.5F, 1.5F, 1.0F, 4);
  readings.tof[0] = {10.0F, 1.0F, 2, lf::ReadingStatus::kNoReturn};
  route.record_readings(readings);
  require(route.coverage().range_acquisitions == count, "reused range never multiplies evidence");
}

void verify_obstacle_union_and_gauge() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  auto readings = wheels(0.0F, 0.0F, 0.0F, 1);
  readings.tof[0] = {1.0F, 0.0F, 1, lf::ReadingStatus::kValid};
  route.record_readings(readings);
  const auto first = route.locate_obstacle(0, route.locate_robot());
  readings = wheels(0.0F, 0.0F, 1.0F, 2);
  readings.tof[0] = {2.5F, 0.0F, 2, lf::ReadingStatus::kValid};
  route.record_readings(readings);
  // Two patches 1.5 L0 apart with these cone widths do not overlap, and a scalar
  // range cannot prove they are the same body, so they stay two records. Merging
  // them into one disk wide enough to contain both would forbid the gap between
  // them without having measured anything there.
  const float second_radius = 2.5F * std::tan(0.2F) + 0.04F;
  const auto second = route.locate_obstacle(1, route.locate_robot());
  require(route.obstacle_size() == 2 && second.centre.x == 3.0F && second.radius >= second_radius &&
            route.locate_obstacle(0, route.locate_robot()).radius >= first.radius,
          "a separated patch is its own record, never folded into a wider disk");

  // Patches that do overlap are one uncertain region and the union is preserved:
  // the record must cover both measured patches, never their intersection.
  lf::Route union_route(geometry, tuning);
  readings = wheels(0.0F, 0.0F, 0.0F, 1);
  readings.tof[0] = {1.0F, 0.0F, 1, lf::ReadingStatus::kValid};
  union_route.record_readings(readings);
  const auto near_first = union_route.locate_obstacle(0, union_route.locate_robot());
  readings = wheels(0.0F, 0.0F, 1.0F, 2);
  readings.tof[0] = {1.2F, 0.0F, 2, lf::ReadingStatus::kValid};
  union_route.record_readings(readings);
  const auto combined = union_route.locate_obstacle(0, union_route.locate_robot());
  const float near_second_radius = 1.2F * std::tan(0.2F) + 0.04F;
  require(
    union_route.obstacle_size() == 1 &&
      std::hypot(combined.centre.x - near_first.centre.x, combined.centre.y - near_first.centre.y) +
          near_first.radius <=
        combined.radius + 1e-5F &&
      std::hypot(combined.centre.x - 1.7F, combined.centre.y) + near_second_radius <=
        combined.radius + 1e-5F,
    "an overlapping patch preserves both possible occupied regions, never an intersection");

  const lf::RobotTuning uncertain;
  lf::Route gauge(geometry, uncertain);
  gauge.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  readings = wheels(5.0F, 5.0F, 1.0F, 2);
  readings.tof[0] = {1.0F, 0.0F, 1, lf::ReadingStatus::kValid};
  gauge.record_readings(readings);
  const auto current = gauge.locate_robot();
  const auto local = gauge.locate_obstacle(0, current);
  require(local.radius < current.position_error,
          "same-acquisition obstacle and robot share a gauge, not two independent global errors");
  gauge.prepare_run();
  gauge.record_readings(wheels(100.0F, 100.0F, 20.0F, 1));
  // Cross-run transport adds both gauges instead of cancelling them, and that
  // cost is carried as uncertainty because the stored shape is still the
  // observed envelope.
  require(gauge.locate_obstacle(0, gauge.locate_robot()).uncertainty >=
            local.uncertainty + current.position_error,
          "a different run does not silently cancel unknown map-to-pose error");
}

void verify_obstacle_capacity() {
  auto geometry = map_geometry();
  geometry.tof_half_fov = 0.01F;  // Narrow-beam unit fixture to separate 16 envelopes.
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  for (std::uint32_t i = 0; i < lf::Route::kObstacleCapacity; ++i) {
    const float angle = static_cast<float>(i) * 2.0F * std::acos(-1.0F) /
                        static_cast<float>(lf::Route::kObstacleCapacity);
    auto packet = wheels(-0.5F * angle, 0.5F * angle, 1.0F, i + 2U);
    packet.tof[0] = {10.0F, 0.0F, i + 1U, lf::ReadingStatus::kValid};
    route.record_readings(packet);
  }
  require(route.obstacle_size() == lf::Route::kObstacleCapacity && !route.coverage().overflow,
          "the declared number of disjoint envelopes fits exactly");
  auto packet = wheels(-std::acos(-1.0F), std::acos(-1.0F), 1.0F, 100);
  packet.tof[0] = {20.0F, 0.0F, 100, lf::ReadingStatus::kValid};
  route.record_readings(packet);
  require(route.obstacle_size() == lf::Route::kObstacleCapacity && route.coverage().overflow,
          "a new object beyond capacity signals exhaustion without replacing an old object");
}

void verify_rejected_newer_ir() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  auto packet = wheels(0.0F, 0.0F, 0.0F, 1);
  illuminate(packet, 1, {3, 4});
  route.record_readings(packet);
  packet = wheels(0.0F, 0.0F, 1.0F, 1);
  for (auto& wheel : packet.wheel_positions) {
    wheel.age = 1.0F;
  }
  illuminate(packet, 2, {3, 4});
  route.record_readings(packet);
  packet = wheels(1.0F, 1.0F, 0.5F, 2);
  for (auto& wheel : packet.wheel_positions) {
    wheel.age = 0.5F;
  }
  illuminate(packet, 3, {3, 4});
  packet.ir[3].value = 2.0F;
  route.record_readings(packet);
  packet = wheels(2.0F, 2.0F, 0.5F, 3);
  illuminate(packet, 4, {3, 4});
  route.record_readings(packet);
  auto query = route.locate_robot();
  query.position = {1.9F, 0.0F};
  require(
    route.line_point(route.line_size() - 1U).break_before && !route.locate_line(query).supported,
    "an older pending observation cannot consume a later rejected acquisition's break");
  packet = wheels(3.0F, 3.0F, 1.0F, 4);
  illuminate(packet, 5, {3, 4});
  route.record_readings(packet);
  query = route.locate_robot();
  query.position = {2.9F, 0.0F};
  require(
    !route.line_point(route.line_size() - 1U).break_before && route.locate_line(query).supported,
    "valid acquisitions beyond the rejected epoch resume support without erasing its gap");
}

void verify_profile_transport() {
  auto geometry = map_geometry();
  geometry.wheel_acceleration = 2.208F;
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  auto packet = wheels(0.0F, 0.0F, 0.0F, 1);
  packet.tof[0] = {1.0F, 0.0F, 1, lf::ReadingStatus::kValid};
  route.record_readings(packet);
  const auto before = route.locate_obstacle(0, route.locate_robot());
  const float pi = std::acos(-1.0F);
  route.record_readings(wheels(pi, pi, 2.0F * pi, 2));
  const auto after = route.locate_obstacle(0, route.locate_robot());
  // Gauge error says where the record could be, not how large the object is, so
  // it is carried as uncertainty rather than folded into the observed envelope.
  const auto expected = static_cast<float>(before.uncertainty + route.locate_robot().profile_error);
  require(after.uncertainty >= expected,
          "transport includes unobserved motion-profile uncertainty after the anchor");
}

void verify_relative_error_precision() {
  auto geometry = map_geometry();
  geometry.wheel_acceleration = 2.208F;
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  auto packet = wheels(0.0F, 0.0F, 100.0F, 2);
  packet.tof[0] = {1.0F, 0.0F, 1, lf::ReadingStatus::kValid};
  route.record_readings(packet);
  const auto before = route.locate_obstacle(0, route.locate_robot());
  for (std::uint32_t i = 1; i <= 10000; ++i) {
    const float position = static_cast<float>(i) * 0.001F;
    route.record_readings(wheels(position, position, 0.01F, i + 2U));
  }
  const auto after = route.locate_obstacle(0, route.locate_robot());
  // The same split as above: tiny increments after a large earlier bound must
  // survive, and they belong to the gauge uncertainty, not to the observed size.
  require(after.uncertainty > before.uncertainty + 0.001F,
          "common-gauge cancellation must retain small new error after a large earlier bound");
}

void verify_ir_delivery_jitter() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  struct Arrival {
    float interval;
    float position;
    float age;
    std::uint32_t sequence;
  };
  const std::array<Arrival, 5> arrivals = {
    Arrival{0.0F, 0.0F, 0.0F, 1}, Arrival{1.0F, 0.0F, 1.0F, 1}, Arrival{1.0F, 1.0F, 1.0F, 2},
    Arrival{0.5F, 2.0F, 0.5F, 3}, Arrival{1.0F, 3.0F, 0.5F, 4},
  };
  for (const auto& arrival : arrivals) {
    auto packet = wheels(arrival.position, arrival.position, arrival.interval, arrival.sequence);
    for (auto& wheel : packet.wheel_positions) {
      wheel.age = arrival.age;
    }
    illuminate(packet, arrival.sequence, {3, 4}, arrival.age);
    route.record_readings(packet);
  }
  require(route.coverage().line_acquisitions == 4 && !route.coverage().has_gap &&
            !route.line_point(route.line_size() - 1U).break_before,
          "IR support breaks use acquisition cadence, not changing delivery latency");
}

void verify_map_retention() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  auto readings = wheels(0.0F, 0.0F, 0.0F, 1);
  illuminate(readings, 1, {3, 4});
  readings.tof[0] = {1.0F, 0.0F, 1, lf::ReadingStatus::kValid};
  route.record_readings(readings);
  readings = wheels(1.0F, 1.0F, 1.0F, 2);
  illuminate(readings, 2, {3, 4});
  route.record_readings(readings);
  const auto line_size = route.line_size();
  const auto path_size = route.path_size();
  const auto path_end = route.coverage().recorded_distance;
  route.prepare_run();
  readings = wheels(100.0F, 100.0F, 20.0F, 1);
  illuminate(readings, 1, {4, 5, 6});
  readings.tof[0] = {1.0F, 0.5F, 1, lf::ReadingStatus::kValid};
  route.record_readings(readings);
  require(route.line_size() == line_size && route.path_size() == path_size &&
            route.obstacle_size() == 1 && route.rejected_samples() > 0 &&
            route.coverage().recorded_distance == path_end && route.coverage().incomplete,
          "run two preserves the recorded prefix and rejects pre-reposition range");
  route.reset();
  require(route.line_size() == 0 && route.path_size() == 0 && route.obstacle_size() == 0,
          "full session reset clears map and retained path");
}

void verify_crossing_correspondence() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  auto readings = wheels(0.0F, 0.0F, 0.0F, 1);
  illuminate(readings, 1, {3, 4});
  route.record_readings(readings);
  float left = 0.0F;
  float right = 0.0F;
  std::uint32_t sequence = 1;
  const auto move = [&](float travel, float turn) {
    left += travel - 0.5F * turn;
    right += travel + 0.5F * turn;
    ++sequence;
    auto packet = wheels(left, right, 1.0F, sequence);
    illuminate(packet, sequence, {3, 4});
    route.record_readings(packet);
  };
  const float quarter_turn = 0.5F * std::acos(-1.0F);
  move(2.0F, 0.0F);
  move(0.0F, quarter_turn);
  move(2.0F, 0.0F);
  move(0.0F, quarter_turn);
  move(2.0F, 0.0F);
  move(0.0F, quarter_turn);
  move(3.0F, 0.0F);
  move(0.0F, quarter_turn);
  move(3.0F, 0.0F);
  move(0.0F, quarter_turn);
  move(2.0F, 0.0F);
  move(0.0F, quarter_turn);
  move(4.0F, 0.0F);
  auto query = route.locate_robot();
  query.position = {0.0F, 1.0F};
  const auto match = route.locate_line(query);
  require(match.supported && match.ambiguous && match.end_order > match.begin_order,
          "a crossing retains every plausible ordered interval, not the nearest branch");
  require(query.wheel_distance > query.path_distance,
          "wheel rotation, nominal body arc and line acquisition order are different coordinates");
}

float segment_distance(lf::Point2 point, lf::Point2 a, lf::Point2 b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  const float squared = dx * dx + dy * dy;
  const float fraction =
    squared > 0.0F ? std::clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / squared, 0.0F, 1.0F)
                   : 0.0F;
  return std::hypot(point.x - a.x - fraction * dx, point.y - a.y - fraction * dy);
}

void verify_curve_bound() {
  auto geometry = map_geometry();
  geometry.ir_spot_radius = 0.01F;
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  std::array<lf::Point2, 4000> raw{};
  float left = 0.0F;
  float right = 0.0F;
  for (std::uint32_t i = 0; i < raw.size(); ++i) {
    const float angle = 0.006F * std::sin(static_cast<float>(i) * 0.03F);
    left += 0.004F - 0.5F * angle;
    right += 0.004F + 0.5F * angle;
    route.record_readings(wheels(left, right, 0.01F, i + 2U));
    raw[i] = route.locate_robot().position;
  }
  require(route.path_size() < 1000 && !route.coverage().overflow,
          "bounded fitting covers a long smooth trace instead of spending a node every tick");
  for (const auto point : raw) {
    float nearest = std::numeric_limits<float>::infinity();
    for (std::size_t i = 1; i < route.path_size(); ++i) {
      nearest = std::min(nearest, segment_distance(point, route.path_point(i - 1U).position,
                                                   route.path_point(i).position));
    }
    require(nearest <= geometry.ir_spot_radius + 1e-4F,
            "all original stations remain inside the declared simplification corridor");
  }
}

void verify_map_overflow() {
  const auto geometry = map_geometry();
  const auto tuning = exact_wheels();
  lf::Route route(geometry, tuning);
  route.record_readings(wheels(0.0F, 0.0F, 0.0F, 1));
  float left = 0.0F;
  float right = 0.0F;
  const float quarter_turn = 0.5F * std::acos(-1.0F);
  for (std::uint32_t i = 0; i < lf::Route::kCurveCapacity + 4U; ++i) {
    const float turn = (i % 2U == 0) ? quarter_turn : -quarter_turn;
    left -= 0.5F * turn;
    right += 0.5F * turn;
    route.record_readings(wheels(left, right, 1.0F, 2U * i + 2U));
    left += 1.0F;
    right += 1.0F;
    route.record_readings(wheels(left, right, 1.0F, 2U * i + 3U));
  }
  require(route.path_size() == lf::Route::kCurveCapacity && route.coverage().overflow,
          "finite storage reports exhaustion instead of collapsing the tail into the last node");
  const auto before = route.path_point(route.path_size() - 1U);
  route.record_readings(wheels(left + 10.0F, right + 10.0F, 1.0F, 10000));
  const auto after = route.path_point(route.path_size() - 1U);
  require(before.position.x == after.position.x && before.position.y == after.position.y &&
            before.path_distance == after.path_distance,
          "overflow freezes the recorded prefix rather than aliasing later motion");
  lf::Route pending(geometry, tuning);
  for (std::uint32_t i = 0; i <= lf::Route::kPendingCapacity; ++i) {
    lf::SensorReadings packet;
    packet.elapsed_time = 1.0F;
    illuminate(packet, i + 1U, {3, 4});
    pending.record_readings(packet);
  }
  require(pending.coverage().overflow && pending.coverage().line_acquisitions == 0,
          "pending acquisition work also has a hard budget and cannot invent poses");
}

}  // namespace

int main() {
  verify_rejected_newer_ir();
  verify_profile_transport();
  verify_relative_error_precision();
  verify_ir_delivery_jitter();
  verify_unobserved_motion_profile();
  verify_delivery_jitter();
  verify_discarded_line_break();
  verify_line_map();
  verify_pending_map();
  verify_obstacle_map();
  verify_obstacle_union_and_gauge();
  verify_obstacle_capacity();
  verify_map_retention();
  verify_crossing_correspondence();
  verify_curve_bound();
  verify_map_overflow();
  for (const float heading : {0.0F, 1.57079633F, 3.14159265F, -1.57079633F}) {
    verify_against_plant(heading, 1.0F);
    verify_against_plant(heading, 0.85F);
  }
  verify_motion();
  verify_asynchronous_wheels();
  verify_reposition_and_missing_origin();
  verify_bounds_and_history();
  verify_reverse_and_wrap();
  verify_travel_budget();
  std::cout << "route estimator: alignment/history/reposition/conditional bounds PASS\n";
}
