#include "track_asset.hpp"

#include <png.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <utility>

namespace lf::sim {
namespace {

using Json = nlohmann::json;
constexpr std::size_t kMaxPixels = 64UL * 1024UL * 1024UL;

bool decode_png(const std::filesystem::path& path, std::vector<std::uint8_t>& pixels, int& width,
                int& height, std::string& error) {
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  if (png_image_begin_read_from_file(&image, path.c_str()) == 0) {
    error = image.message;
    return false;
  }
  image.format = PNG_FORMAT_RGBA;
  if (image.width == 0 || image.height == 0 || image.width > std::numeric_limits<int>::max() ||
      image.height > std::numeric_limits<int>::max() || PNG_IMAGE_SIZE(image) > kMaxPixels * 4U) {
    error = "PNG dimensions exceed bounded asset profile";
    png_image_free(&image);
    return false;
  }
  std::vector<png_byte> rgba(PNG_IMAGE_SIZE(image));
  if (png_image_finish_read(&image, nullptr, rgba.data(), 0, nullptr) == 0) {
    error = image.message;
    png_image_free(&image);
    return false;
  }
  width = static_cast<int>(image.width);
  height = static_cast<int>(image.height);
  const std::size_t count = static_cast<std::size_t>(width) * height;
  pixels.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    const unsigned alpha = rgba[i * 4U + 3U];
    const unsigned luminance =
      (299U * rgba[i * 4U] + 587U * rgba[i * 4U + 1U] + 114U * rgba[i * 4U + 2U] + 500U) / 1000U;
    pixels[i] = static_cast<std::uint8_t>((luminance * alpha + 255U * (255U - alpha)) / 255U);
  }
  png_image_free(&image);
  return true;
}

bool finite_positive(float value) {
  return std::isfinite(value) && value > 0.0F;
}

}  // namespace

bool TrackAsset::load(const std::string& manifest, std::string& error) {
  TrackAsset candidate;
  try {
    const std::filesystem::path manifest_path(manifest);
    std::ifstream file(manifest_path);
    if (!file) {
      error = "cannot open manifest: " + manifest;
      return false;
    }
    const Json json = Json::parse(file);
    const auto number = [&json](const char* key) { return json.at(key).get<float>(); };
    candidate.name = json.at("name").get<std::string>();
    candidate.image_path = json.at("image").get<std::string>();
    const std::filesystem::path image_path(candidate.image_path);
    if (image_path.is_absolute()) {
      error = "track image path must be relative to the manifest root";
      return false;
    }
    const auto configured_root = manifest_path.parent_path().parent_path();
    const auto root = std::filesystem::absolute(configured_root.empty() ? std::filesystem::path(".")
                                                                        : configured_root)
                        .lexically_normal();
    const auto resolved = std::filesystem::absolute(root / image_path).lexically_normal();
    const auto relative = resolved.lexically_relative(root);
    if (relative.empty() || *relative.begin() == "..") {
      error = "track image escapes the asset root";
      return false;
    }
    candidate.image_path = resolved.string();
    candidate.arena_width_mm = number("arena_width");
    candidate.arena_height_mm = number("arena_height");
    const auto& pose = json.at("start_pose");
    candidate.start_x_mm = pose.at("x").get<float>();
    candidate.start_y_mm = pose.at("y").get<float>();
    candidate.start_yaw_rad = pose.at("theta_deg").get<float>() * 0.01745329251994329577F;
    const auto& finish = json.at("finish_box");
    candidate.finish_min_x_mm = finish.at("min_x").get<float>();
    candidate.finish_max_x_mm = finish.at("max_x").get<float>();
    candidate.finish_min_y_mm = finish.at("min_y").get<float>();
    candidate.finish_max_y_mm = finish.at("max_y").get<float>();
    for (const auto& item : json.at("obstacles")) {
      candidate.obstacles.push_back({
        item.at("x").get<float>(),
        item.at("y").get<float>(),
        item.at("width").get<float>(),
        item.at("height").get<float>(),
        item.value("angle_deg", 0.0F) * 0.01745329251994329577F,
      });
    }
    if (!finite_positive(candidate.arena_width_mm) || !finite_positive(candidate.arena_height_mm) ||
        !std::isfinite(candidate.start_yaw_rad) || candidate.finish_min_x_mm < 0.0F ||
        candidate.finish_max_x_mm > candidate.arena_width_mm || candidate.finish_min_y_mm < 0.0F ||
        candidate.finish_max_y_mm > candidate.arena_height_mm ||
        candidate.finish_min_x_mm >= candidate.finish_max_x_mm ||
        candidate.finish_min_y_mm >= candidate.finish_max_y_mm || candidate.start_x_mm < 0.0F ||
        candidate.start_x_mm > candidate.arena_width_mm || candidate.start_y_mm < 0.0F ||
        candidate.start_y_mm > candidate.arena_height_mm) {
      error = "manifest contains invalid arena, start, or finish geometry";
      return false;
    }
    for (const auto& obstacle : candidate.obstacles) {
      if (!std::isfinite(obstacle.x_mm) || !std::isfinite(obstacle.y_mm) ||
          !finite_positive(obstacle.width_mm) || !finite_positive(obstacle.height_mm) ||
          !std::isfinite(obstacle.angle_rad)) {
        error = "manifest contains invalid obstacle geometry";
        return false;
      }
    }
    if (!decode_png(candidate.image_path, candidate.grayscale, candidate.image_width,
                    candidate.image_height, error)) {
      return false;
    }
    *this = std::move(candidate);
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

std::uint8_t TrackAsset::sample(float x_mm, float y_mm) const noexcept {
  if (image_width <= 0 || image_height <= 0 || arena_width_mm <= 0.0F || arena_height_mm <= 0.0F ||
      x_mm < 0.0F || y_mm < 0.0F || x_mm >= arena_width_mm || y_mm >= arena_height_mm) {
    return 255;
  }
  const int x = static_cast<int>(x_mm * static_cast<float>(image_width) / arena_width_mm);
  const int y = static_cast<int>(y_mm * static_cast<float>(image_height) / arena_height_mm);
  const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(image_width) +
                     static_cast<std::size_t>(x);
  return grayscale[index];
}

}  // namespace lf::sim
