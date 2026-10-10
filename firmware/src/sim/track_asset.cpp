#include "track_asset.hpp"

#include <cstddef>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

#include <raylib.h>

namespace lf::sim {
namespace {

using nlohmann::json;

void load_manifest(const json& doc, TrackManifest& manifest) {
  manifest.name = doc.value("name", std::string{});
  manifest.image_path = doc.value("image", std::string{});
  manifest.arena_width_mm = doc.value("arena_width", manifest.arena_width_mm);
  manifest.arena_height_mm = doc.value("arena_height", manifest.arena_height_mm);

  if (doc.contains("start_pose")) {
    const auto& pose = doc.at("start_pose");
    manifest.start_x_mm = pose.value("x", manifest.start_x_mm);
    manifest.start_y_mm = pose.value("y", manifest.start_y_mm);
    manifest.start_theta_deg = pose.value("theta_deg", manifest.start_theta_deg);
  }
  if (doc.contains("finish_box")) {
    const auto& box = doc.at("finish_box");
    manifest.finish_min_x = box.value("min_x", manifest.finish_min_x);
    manifest.finish_max_x = box.value("max_x", manifest.finish_max_x);
    manifest.finish_min_y = box.value("min_y", manifest.finish_min_y);
    manifest.finish_max_y = box.value("max_y", manifest.finish_max_y);
  }
  if (doc.contains("obstacles")) {
    for (const auto& item : doc.at("obstacles")) {
      ObstacleSpec spec;
      spec.x_mm = item.value("x", spec.x_mm);
      spec.y_mm = item.value("y", spec.y_mm);
      spec.width_mm = item.value("width", spec.width_mm);
      spec.height_mm = item.value("height", spec.height_mm);
      spec.angle_deg = item.value("angle_deg", spec.angle_deg);
      manifest.obstacles.push_back(spec);
    }
  }
}

}  // namespace

bool TrackAsset::load(const std::string& manifest_path) {
  std::ifstream file(manifest_path);
  if (!file.is_open()) {
    return false;
  }
  json doc;
  try {
    doc = json::parse(file);
  } catch (const json::exception&) {
    return false;
  }
  load_manifest(doc, manifest_);

  Image decoded = LoadImage(manifest_.image_path.c_str());
  if (decoded.data == nullptr) {
    return false;
  }
  ImageFormat(&decoded, PIXELFORMAT_UNCOMPRESSED_GRAYSCALE);
  width_ = decoded.width;
  height_ = decoded.height;
  const auto* pixels = static_cast<const unsigned char*>(decoded.data);
  const auto pixel_count = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  pixels_.assign(pixels, pixels + pixel_count);
  UnloadImage(decoded);
  return true;
}

unsigned char TrackAsset::sample_grayscale(float x_mm, float y_mm) const {
  const auto x = static_cast<int>(x_mm);
  const auto y = static_cast<int>(y_mm);
  if (x < 0 || y < 0 || x >= width_ || y >= height_) {
    return 255;
  }
  const auto index =
    static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
  return pixels_[index];
}

}  // namespace lf::sim
