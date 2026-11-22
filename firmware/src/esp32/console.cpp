#include "console.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace lf::esp32 {
namespace {

constexpr std::size_t kDefaultLogLines = 300;

void report(const char* text) {
  static_cast<void>(std::fwrite(text, 1, std::strlen(text), stdout));
  static_cast<void>(std::fflush(stdout));
}

std::array<char, 64> line{};
std::size_t length = 0;

const char* phase_name(Phase phase) {
  switch (phase) {
    case Phase::kIdle:
      return "idle";
    case Phase::kSettling:
      return "settling";
    case Phase::kRunning:
      return "running";
    case Phase::kReady:
      return "ready";
    case Phase::kStopped:
      return "stopped";
    case Phase::kFinished:
      return "finished";
  }
  return "?";
}

const char* race_name(RaceState state) {
  switch (state) {
    case RaceState::kScan:
      return "scan";
    case RaceState::kReturn:
      return "return";
    case RaceState::kReady:
      return "ready";
    case RaceState::kFollow:
      return "follow";
    case RaceState::kCorner:
      return "corner";
    case RaceState::kConfirmGo:
      return "confirm_go";
    case RaceState::kBypass:
      return "bypass";
    case RaceState::kFinished:
      return "finished";
    case RaceState::kFailed:
      return "-";
  }
  return "?";
}

const char* shape_name(LineShape shape) {
  switch (shape) {
    case LineShape::kNone:
      return "none";
    case LineShape::kLine:
      return "line";
    case LineShape::kSplit:
      return "split";
    case LineShape::kWide:
      return "wide";
  }
  return "?";
}

const char* reason_name(StopReason reason) {
  switch (reason) {
    case StopReason::kNone:
      return "none";
    case StopReason::kDone:
      return "done";
    case StopReason::kButton:
      return "button";
    case StopReason::kStall:
      return "stall";
    case StopReason::kBattery:
      return "battery";
    case StopReason::kGyroStale:
      return "gyro_stale";
    case StopReason::kGyroDisagrees:
      return "gyro_disagrees";
    case StopReason::kManeuverTimeout:
      return "maneuver_timeout";
    case StopReason::kNotStill:
      return "not_still";
    case StopReason::kScanFailed:
      return "scan_failed";
    case StopReason::kIrStale:
      return "ir_stale";
    case StopReason::kLineLost:
      return "line_lost";
    case StopReason::kWideTooLong:
      return "wide_too_long";
    case StopReason::kBypassLost:
      return "bypass_lost";
    case StopReason::kFinish:
      return "finish";
  }
  return "?";
}

void print_status(const Snapshot& s, const Diagnostics& d) {
  std::array<char, 512> text{};
  std::snprintf(text.data(), text.size(),
                "phase=%s reason=%s ready=%d\n"
                "s=%.1f mm psi=%.1f deg rate=%.3f rad/s bias=%.4f rad/s bat=%.2f V\n"
                "cps=[%.0f,%.0f]\n"
                "tick_max=%" PRIu32 " us late=%" PRIu32 " watchdog_cuts=%" PRIu32
                " adc_ovf=%" PRIu32 " adc_restarts=%" PRIu32 " motors=%" PRIu32 "\n",
                phase_name(s.phase), reason_name(s.reason), s.ready ? 1 : 0,
                static_cast<double>(s.s_mm), static_cast<double>(s.psi_deg),
                static_cast<double>(s.yaw_rate_rad_s), static_cast<double>(s.gyro_bias_rad_s),
                static_cast<double>(s.battery_v), static_cast<double>(s.cps[0]),
                static_cast<double>(s.cps[1]), d.tick_max_us, d.late_ticks, d.watchdog_cuts,
                d.adc_overflows, d.adc_restarts, d.motor_applies);
  report(text.data());
  std::array<char, 384> levels{};
  std::snprintf(levels.data(), levels.size(),
                "race=%s usable_eyes=%u line_reads=%s\n"
                "floor=[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]\n"
                "line =[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]\n",
                race_name(s.race), static_cast<unsigned>(s.usable_eyes),
                s.line_reads_higher ? "higher" : "lower", static_cast<double>(s.floor_level[0]),
                static_cast<double>(s.floor_level[1]), static_cast<double>(s.floor_level[2]),
                static_cast<double>(s.floor_level[3]), static_cast<double>(s.floor_level[4]),
                static_cast<double>(s.floor_level[5]), static_cast<double>(s.floor_level[6]),
                static_cast<double>(s.floor_level[7]), static_cast<double>(s.line_level[0]),
                static_cast<double>(s.line_level[1]), static_cast<double>(s.line_level[2]),
                static_cast<double>(s.line_level[3]), static_cast<double>(s.line_level[4]),
                static_cast<double>(s.line_level[5]), static_cast<double>(s.line_level[6]),
                static_cast<double>(s.line_level[7]));
  report(levels.data());
}

void print_monitor(const Measurements& m, const Diagnostics& d) {
  std::array<char, 384> text{};
  std::snprintf(text.data(), text.size(),
                "ir=[%u,%u,%u,%u,%u,%u,%u,%u]\n"
                "enc=[%" PRId32 ",%" PRId32
                "] gyro_z=%.4f tof=%.0f st=%u bat=%.2f\n"
                "imu 0x%02x r=%u tof 0x%04x r=%u button=%u\n",
                m.ir.value[0], m.ir.value[1], m.ir.value[2], m.ir.value[3], m.ir.value[4],
                m.ir.value[5], m.ir.value[6], m.ir.value[7], m.encoders.value[0],
                m.encoders.value[1], static_cast<double>(m.gyro_z_rad_s.value),
                static_cast<double>(m.distance_mm.value), m.range_status,
                static_cast<double>(m.battery_v.value), d.imu_identity, d.imu_ready ? 1U : 0U,
                d.tof_identity, d.tof_ready ? 1U : 0U, d.button_down ? 1U : 0U);
  report(text.data());
}

void print_record(const LogRecord& r) {
  std::array<char, 192> text{};
  std::snprintf(text.data(), text.size(),
                "%" PRIu32
                ",%s,%s,%.1f,%.4f,%.3f,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%s,%s,%u,%d\n",
                r.t_ms, phase_name(r.phase), reason_name(r.reason), static_cast<double>(r.s_counts),
                static_cast<double>(r.psi_rad), static_cast<double>(r.yaw_rate_rad_s), r.cps[0],
                r.cps[1], r.duty_percent[0], r.duty_percent[1], r.tof_mm, r.tof_status, r.ir[0],
                r.ir[1], r.ir[2], r.ir[3], r.ir[4], r.ir[5], r.ir[6], r.ir[7], race_name(r.race),
                shape_name(r.shape), r.dark_eyes, r.position_tenths);
  report(text.data());
}

// The RAM log when this boot has run something; otherwise a run saved in flash
// (run 0 is the newest of the last four).
void print_log(const LogRing& log, const Blackbox& blackbox, std::size_t lines, std::size_t run) {
  report(
    "t_ms,phase,reason,s_counts,psi_rad,rate,cps_l,cps_r,tgt_l,tgt_r,duty_l,duty_r,tof,tof_st,ir0,"
    "ir1,"
    "ir2,ir3,ir4,ir5,ir6,ir7,race,shape,dark,pos10\n");
  if (log.size() > 0 && run == 0) {
    const std::size_t count = log.size() < lines ? log.size() : lines;
    for (std::size_t i = log.size() - count; i < log.size(); ++i) {
      print_record(log.at(i));
    }
    return;
  }
  const std::size_t flash_run = log.size() > 0 ? run - 1 : run;
  const std::size_t total = blackbox.count(flash_run);
  std::array<char, 64> header{};
  std::snprintf(header.data(), header.size(), total == 0 ? "(no such run)\n" : "(flash run %u)\n",
                static_cast<unsigned>(flash_run));
  report(header.data());
  for (std::size_t i = total - (total < lines ? total : lines); i < total; ++i) {
    LogRecord record;
    if (blackbox.read(flash_run, i, record)) {
      print_record(record);
    }
  }
}

void handle_line(const char* text, const Snapshot& snapshot, const Measurements& measurements,
                 const LogRing& log, const Blackbox& blackbox, const Diagnostics& diagnostics) {
  if (std::strcmp(text, "status") == 0) {
    print_status(snapshot, diagnostics);
  } else if (std::strcmp(text, "monitor") == 0) {
    print_monitor(measurements, diagnostics);
  } else if (std::strcmp(text, "slots") == 0) {
    for (std::size_t slot = 0; slot < Blackbox::kSlots; ++slot) {
      std::uint32_t sequence = 0;
      std::uint32_t records = 0;
      std::array<char, 64> text_line{};
      if (blackbox.describe(slot, sequence, records)) {
        std::snprintf(text_line.data(), text_line.size(),
                      "slot %u: run #%" PRIu32 ", %" PRIu32 " records\n",
                      static_cast<unsigned>(slot), sequence, records);
      } else {
        std::snprintf(text_line.data(), text_line.size(), "slot %u: empty\n",
                      static_cast<unsigned>(slot));
      }
      report(text_line.data());
    }
  } else if (std::strncmp(text, "log", 3) == 0) {
    char* rest = nullptr;
    const long requested = std::strtol(text + 3, &rest, 10);
    const long run = std::strtol(rest, nullptr, 10);
    print_log(log, blackbox, requested > 0 ? static_cast<std::size_t>(requested) : kDefaultLogLines,
              run > 0 ? static_cast<std::size_t>(run) : 0);
  } else if (std::strcmp(text, "help") == 0) {
    report(
      "status | monitor | slots | log [lines [run]] | help\n"
      "log = this boot's log, or a saved run (run 0 newest, up to 4 back).\n"
      "The console never moves the robot; the button does.\n");
  } else if (text[0] != '\0') {
    report("unknown command\n");
  }
}

}  // namespace

void console_begin() {
  static_cast<void>(fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK));
  report(
    "\nLF-Bot bench console. Operator interface is the button and the LED.\n"
    "status | monitor | slots | log [lines [run]] | help\n");
}

void console_service(const Snapshot& snapshot, const Measurements& measurements, const LogRing& log,
                     const Blackbox& blackbox, const Diagnostics& diagnostics) {
  std::array<char, 32> input{};
  const int count = static_cast<int>(read(STDIN_FILENO, input.data(), input.size()));
  for (int i = 0; i < count; ++i) {
    const char byte = input[i];
    if (byte != '\n') {
      if (length < line.size() - 1) {
        line[length++] = byte;
      }
      continue;
    }
    line[length] = '\0';
    handle_line(line.data(), snapshot, measurements, log, blackbox, diagnostics);
    length = 0;
  }
}

}  // namespace lf::esp32
