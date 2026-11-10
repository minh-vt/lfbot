#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "robot_data.hpp"

namespace lf {

// Internal, dimensionless competition geometry. Robot builds this from
// calibration/domain data; neither Race nor Route sees the simulator.
struct RaceGeometry {
  std::array<Point2, kIrSensorCount> ir_position;  // x forward, y left
  float obstacle_radius = 0.0F;
  float body_radius = 0.0F;
  float tof_forward = 0.0F;
  float tof_half_fov = 0.0F;
  float lookahead = 0.0F;
  float sensor_span_sum = 0.0F;
  float sensor_span_half = 0.0F;  // |y| of the outermost IR sensor
  float wheel_resolution = 0.0F;
  float ir_spot_radius = 0.0F;
  float line_half_width = 0.0F;
  float tof_error = 0.0F;
  float wheel_acceleration = 0.0F;  // zero only in ideal constant-profile unit fixtures
  float ir_interval = 0.0F;
  // Nominal drive model used to realize a speed demand as motor duty.
  float duty_scale = 0.0F;          // V0 per unit duty at steady state
  float motor_response = 0.0F;      // T0 first-order response
  float drive_acceleration = 0.0F;  // L0/T0^2 commandable
  float brake_deceleration = 0.0F;  // L0/T0^2 active brake
};

// What a scalar range did observe: half the cone width at that range plus the
// range error, i.e. a patch on the surface. The object's own extent is not part
// of the measurement, so it is not part of the envelope: inflating the patch by
// an assumed diameter puts the near face's owner behind the robot's own body and
// forbids every approach to the object that was just measured.
inline float observed_patch_radius(const RaceGeometry& geometry, float range) noexcept {
  return range * std::tan(geometry.tof_half_fov) + geometry.tof_error;
}

struct RoutePose {
  double time = 0.0;           // T0 since this run's first decision
  double profile_error = 0.0;  // cumulative: retain small increments for relative queries
  double travel_bound = 0.0;   // includes unobserved reversals within intervals
  Point2 position;
  float heading = 0.0F;         // positive left; not raster-world yaw
  float wheel_position = 0.0F;  // mean normalized cumulative axle position
  float wheel_distance = 0.0F;  // sum of absolute axle net increments, not route s
  float path_distance = 0.0F;   // nominal body arc; not true progress certified by encoders
  float position_error = 0.0F;
  float heading_error = 0.0F;
};

struct RouteMotion {
  float distance = 0.0F;
  float angle = 0.0F;
};

enum class RouteEvidence : std::uint8_t { kUnknown, kLine, kGap, kAmbiguous, kTracedPath };

struct RoutePoint {
  Point2 position;
  float path_distance = 0.0F;  // body arc at acquisition, NOT distance along hidden line
  float error = 0.0F;
  float fit_error = 0.0F;
  std::uint32_t order = 0;  // acquisition order, not a metric line coordinate
  RouteEvidence evidence = RouteEvidence::kUnknown;
  bool break_before = false;
};

struct RouteObstacle {
  Point2 centre;
  float radius = 0.0F;       // observed occupied envelope, not a measured brick centre
  float uncertainty = 0.0F;  // pose-gauge error of `centre`, not part of the shape
  RoutePose anchor;
  std::uint16_t run = 0;
};

struct RouteCorrespondence {
  float begin_order = 0.0F;
  float end_order = 0.0F;
  float begin_distance = 0.0F;
  float end_distance = 0.0F;
  bool supported = false;
  bool ambiguous = false;
};

struct RouteCoverage {
  float recorded_distance = 0.0F;
  std::size_t line_acquisitions = 0;
  std::size_t range_acquisitions = 0;
  bool has_gap = false;
  bool has_ambiguity = false;
  bool overflow = false;
  // A finite recorded prefix never certifies the rest of an unknown course.
  bool incomplete = true;
};

// Private core implementation. Estimates are conditional on the declared
// free-rolling error budget; encoders alone cannot certify chassis motion.
class Route {
public:
  static constexpr std::size_t kHistoryCapacity = 64;
  static constexpr std::size_t kCurveCapacity = 1024;
  static constexpr std::size_t kObstacleCapacity = 16;
  static constexpr std::size_t kPendingCapacity = 32;

  Route(const RaceGeometry& geometry, const RobotTuning& tuning) noexcept;
  void reset() noexcept;
  void prepare_run() noexcept;
  void record_readings(const SensorReadings& sensors) noexcept;
  [[nodiscard]] bool has_position() const noexcept { return has_position_; }
  [[nodiscard]] const RoutePose& locate_robot() const noexcept { return pose_; }
  [[nodiscard]] std::optional<RoutePose> locate_robot(double acquisition_time) const noexcept;
  [[nodiscard]] RouteMotion motion() const noexcept { return motion_; }
  [[nodiscard]] std::size_t rejected_samples() const noexcept { return rejected_samples_; }
  [[nodiscard]] const RouteCoverage& coverage() const noexcept { return coverage_; }
  [[nodiscard]] const RoutePoint& latest_line() const noexcept { return latest_line_; }
  [[nodiscard]] std::size_t line_size() const noexcept { return line_.size; }
  [[nodiscard]] const RoutePoint& line_point(std::size_t i) const noexcept {
    return line_.points[i];
  }
  [[nodiscard]] std::size_t path_size() const noexcept { return path_.size; }
  [[nodiscard]] const RoutePoint& path_point(std::size_t i) const noexcept {
    return path_.points[i];
  }
  [[nodiscard]] std::size_t obstacle_size() const noexcept { return obstacle_size_; }
  [[nodiscard]] RouteObstacle locate_obstacle(std::size_t i, const RoutePose& pose) const noexcept;
  [[nodiscard]] RouteCorrespondence locate_line(const RoutePose& pose) const noexcept;

private:
  template <typename T>
  struct History {
    std::array<T, kHistoryCapacity> entries{};
    std::size_t begin = 0;
    std::size_t size = 0;

    void append(const T& entry) noexcept {
      if (size == kHistoryCapacity) {
        begin = (begin + 1U) % kHistoryCapacity;
        --size;
      }
      entries[(begin + size) % kHistoryCapacity] = entry;
      ++size;
    }
    [[nodiscard]] const T& at(std::size_t index) const noexcept {
      return entries[(begin + index) % kHistoryCapacity];
    }
  };

  struct WheelReading {
    double time = 0.0;
    float position = 0.0F;
  };

  struct Curve {
    std::array<RoutePoint, kCurveCapacity> points{};
    std::size_t size = 0;
    float direction = 0.0F;
    float min_angle = 0.0F;
    float max_angle = 0.0F;
    float max_radius = 0.0F;
  };

  struct PendingReading {
    double time = 0.0;
    std::array<float, kIrSensorCount> values{};
    bool range = false;
    bool break_before = false;
    ReadingStatus status = ReadingStatus::kValid;
  };

  void reset_estimate() noexcept;
  void record_path(float interval_travel) noexcept;
  [[nodiscard]] std::optional<double> locate_acquisition(float age) const noexcept;
  void record_wheel(std::size_t index, const SensorObservation& sample) noexcept;
  void queue_line(const SensorReadings& sensors) noexcept;
  void mark_line_gap(double first, double last) noexcept;
  void queue_range(const SensorObservation& sample) noexcept;
  void queue_reading(const PendingReading& sample) noexcept;
  void process_readings() noexcept;
  void record_line(const PendingReading& sample, const RoutePose& pose) noexcept;
  void record_obstacle(const PendingReading& sample, const RoutePose& pose) noexcept;
  [[nodiscard]] bool record_point(Curve& curve, RoutePoint point) const noexcept;
  [[nodiscard]] float transport_error(const RouteObstacle& obstacle, const RoutePose& pose,
                                      float reach) const noexcept;
  [[nodiscard]] std::optional<float> locate_wheel(std::size_t index, double time) const noexcept;
  void update_position(double time, const std::array<float, 2>& wheels) noexcept;

  double time_ = 0.0;
  std::size_t rejected_samples_ = 0;
  std::size_t obstacle_size_ = 0;
  std::size_t pending_size_ = 0;
  double last_ir_time_ = 0.0;
  double last_range_time_ = 0.0;
  double first_line_break_time_ = 0.0;
  double last_line_break_time_ = 0.0;
  RouteCoverage coverage_;
  RoutePose pose_;
  std::array<RouteObstacle, kObstacleCapacity> obstacles_{};
  std::array<PendingReading, kPendingCapacity> pending_{};
  std::array<History<WheelReading>, 2> wheels_;
  History<RoutePose> poses_;
  Curve line_;
  Curve path_;
  float wheel_resolution_;
  float travel_error_;
  float ir_noise_bound_;
  float fit_tolerance_;
  std::uint32_t range_sequence_ = 0;
  std::uint32_t line_order_ = 0;
  RouteMotion motion_;
  std::array<std::uint32_t, 2> wheel_sequences_{};
  std::array<float, 2> previous_wheels_{};
  RoutePoint latest_line_;
  std::array<std::uint32_t, kIrSensorCount> ir_sequences_{};
  RaceGeometry geometry_;
  std::uint16_t run_ = 0;
  bool first_packet_ = true;
  bool has_position_ = false;
  bool has_ir_sequence_ = false;
  bool has_range_sequence_ = false;
  bool line_break_ = false;
  std::array<bool, 2> has_wheel_sequence_{};
};

}  // namespace lf
