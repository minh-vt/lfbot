#pragma once

#include <string>
#include <vector>

#include "race_arbiter.hpp"
#include "types.hpp"

namespace lf::sim {

struct TrackManifest {
  std::string name;
  std::string image_path;
  float arena_width_mm = 2400.0f;
  float arena_height_mm = 1600.0f;
  float start_x = 500.0f;
  float start_y = 1300.0f;
  float start_theta_deg = 0.0f;
  float min_finish_arm_dist_m = 12.0f;
  FinishBoxGeometry finish_box{5662.0f, 5822.0f, 1124.0f, 1284.0f};
  std::vector<Obstacle> obstacles;
};

// Tai thong tin sa ban tu file JSON. Tra ve false neu khong doc duoc.
bool load_track_manifest(const std::string& filepath, TrackManifest& out_manifest);

// Ghi thong tin sa ban ra file JSON. Tra ve false neu loi ghi file.
bool save_track_manifest(const std::string& filepath, const TrackManifest& manifest);

// Danh sach duong dan cac file manifest co san trong assets/
std::vector<std::string> get_available_track_manifests();

}  // namespace lf::sim
