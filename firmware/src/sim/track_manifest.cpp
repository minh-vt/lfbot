#include "track_manifest.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace lf::sim {

namespace {

// Helper parser de trich xuat chuoi giua 2 dau ngoac kep sau mot key
bool extract_string_field(const std::string& content, const std::string& key, std::string& out) {
  const std::string search = "\"" + key + "\"";
  const size_t pos = content.find(search);
  if (pos == std::string::npos) {
    return false;
  }
  const size_t colon = content.find(':', pos + search.length());
  if (colon == std::string::npos) {
    return false;
  }
  const size_t q1 = content.find('\"', colon + 1);
  if (q1 == std::string::npos) {
    return false;
  }
  const size_t q2 = content.find('\"', q1 + 1);
  if (q2 == std::string::npos) {
    return false;
  }
  out = content.substr(q1 + 1, q2 - q1 - 1);
  return true;
}

// Helper parser de trich xuat so float sau mot key
bool extract_float_field(const std::string& content, const std::string& key, float& out,
                         size_t start_offset = 0) {
  const std::string search = "\"" + key + "\"";
  const size_t pos = content.find(search, start_offset);
  if (pos == std::string::npos) {
    return false;
  }
  const size_t colon = content.find(':', pos + search.length());
  if (colon == std::string::npos) {
    return false;
  }
  size_t val_start = colon + 1;
  while (val_start < content.length() &&
         (std::isspace(static_cast<unsigned char>(content.at(val_start))) != 0)) {
    val_start++;
  }
  size_t val_end = val_start;
  while (val_end < content.length() &&
         (content.at(val_end) == '-' || content.at(val_end) == '+' || content.at(val_end) == '.' ||
          (std::isdigit(static_cast<unsigned char>(content.at(val_end))) != 0))) {
    val_end++;
  }
  if (val_end > val_start) {
    out = std::stof(content.substr(val_start, val_end - val_start));
    return true;
  }
  return false;
}

}  // namespace

bool load_track_manifest(const std::string& filepath, TrackManifest& out_manifest) {
  std::ifstream fs(filepath);
  if (!fs.is_open()) {
    if (filepath.rfind("assets/", 0) == 0) {
      fs.open("firmware/" + filepath);
    } else if (filepath.rfind("firmware/assets/", 0) == 0) {
      fs.open(filepath.substr(9));
    }
  }
  if (!fs.is_open()) {
    return false;
  }

  std::stringstream ss;
  ss << fs.rdbuf();
  const std::string content = ss.str();

  (void)extract_string_field(content, "name", out_manifest.name);
  (void)extract_string_field(content, "image", out_manifest.image_path);

  (void)extract_float_field(content, "arena_width", out_manifest.arena_width_mm);
  (void)extract_float_field(content, "arena_height", out_manifest.arena_height_mm);
  (void)extract_float_field(content, "min_finish_arm_dist_m", out_manifest.min_finish_arm_dist_m);

  // Parse start pose
  const size_t sp_pos = content.find("\"start_pose\"");
  if (sp_pos != std::string::npos) {
    (void)extract_float_field(content, "x", out_manifest.start_x, sp_pos);
    (void)extract_float_field(content, "y", out_manifest.start_y, sp_pos);
    (void)extract_float_field(content, "theta_deg", out_manifest.start_theta_deg, sp_pos);
  }

  // Parse finish box
  const size_t fb_pos = content.find("\"finish_box\"");
  if (fb_pos != std::string::npos) {
    (void)extract_float_field(content, "min_x", out_manifest.finish_box.min_x_mm, fb_pos);
    (void)extract_float_field(content, "max_x", out_manifest.finish_box.max_x_mm, fb_pos);
    (void)extract_float_field(content, "min_y", out_manifest.finish_box.min_y_mm, fb_pos);
    (void)extract_float_field(content, "max_y", out_manifest.finish_box.max_y_mm, fb_pos);
  }

  // Parse obstacles
  out_manifest.obstacles.clear();
  const size_t obs_pos = content.find("\"obstacles\"");
  if (obs_pos != std::string::npos) {
    size_t cur = obs_pos;
    while (true) {
      const size_t obj_start = content.find('{', cur);
      if (obj_start == std::string::npos) {
        break;
      }
      const size_t obj_end = content.find('}', obj_start);
      if (obj_end == std::string::npos) {
        break;
      }

      const std::string block = content.substr(obj_start, obj_end - obj_start + 1);
      Obstacle obs{};
      obs.width_mm = 105.0f;
      obs.height_mm = 60.0f;
      obs.angle_deg = 0.0f;

      const bool has_x = extract_float_field(block, "x", obs.pos.x);
      const bool has_y = extract_float_field(block, "y", obs.pos.y);
      (void)extract_float_field(block, "width", obs.width_mm);
      (void)extract_float_field(block, "height", obs.height_mm);
      (void)extract_float_field(block, "angle_deg", obs.angle_deg);

      if (has_x && has_y) {
        out_manifest.obstacles.push_back(obs);
      }
      cur = obj_end + 1;
    }
  }

  return true;
}

bool save_track_manifest(const std::string& filepath, const TrackManifest& manifest) {
  std::ofstream fs(filepath);
  if (!fs.is_open()) {
    return false;
  }

  fs << "{\n";
  fs << R"(  "name": ")" << manifest.name << R"(",)" << "\n";
  fs << R"(  "image": ")" << manifest.image_path << R"(",)" << "\n";
  fs << R"(  "arena_width": )" << manifest.arena_width_mm << ",\n";
  fs << R"(  "arena_height": )" << manifest.arena_height_mm << ",\n";
  fs << R"(  "start_pose": {)" << "\n";
  fs << R"(    "x": )" << manifest.start_x << ",\n";
  fs << R"(    "y": )" << manifest.start_y << ",\n";
  fs << R"(    "theta_deg": )" << manifest.start_theta_deg << "\n";
  fs << "  },\n";
  fs << R"(  "finish_box": {)" << "\n";
  fs << R"(    "min_x": )" << manifest.finish_box.min_x_mm << ",\n";
  fs << R"(    "max_x": )" << manifest.finish_box.max_x_mm << ",\n";
  fs << R"(    "min_y": )" << manifest.finish_box.min_y_mm << ",\n";
  fs << R"(    "max_y": )" << manifest.finish_box.max_y_mm << "\n";
  fs << "  },\n";
  fs << R"(  "min_finish_arm_dist_m": )" << manifest.min_finish_arm_dist_m << ",\n";
  fs << R"(  "obstacles": [)" << "\n";
  for (size_t i = 0; i < manifest.obstacles.size(); ++i) {
    const auto& obs = manifest.obstacles.at(i);
    fs << "    {\n";
    fs << R"(      "x": )" << obs.pos.x << ",\n";
    fs << R"(      "y": )" << obs.pos.y << ",\n";
    fs << R"(      "width": )" << obs.width_mm << ",\n";
    fs << R"(      "height": )" << obs.height_mm << ",\n";
    fs << R"(      "angle_deg": )" << obs.angle_deg << "\n";
    fs << "    }" << (i + 1 < manifest.obstacles.size() ? "," : "") << "\n";
  }
  fs << "  ]\n";
  fs << "}\n";

  return true;
}

std::vector<std::string> get_available_track_manifests() {
  std::vector<std::string> manifests;
  const std::array<std::filesystem::path, 2> search_dirs = {
    std::filesystem::path("assets"),
    std::filesystem::path("firmware/assets"),
  };

  for (const auto& dir : search_dirs) {
    std::error_code ec;
    if (std::filesystem::exists(dir, ec) && std::filesystem::is_directory(dir, ec)) {
      for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (entry.is_regular_file(ec) && entry.path().extension() == ".json") {
          const std::string filename = entry.path().filename().string();
          if (filename.rfind("track", 0) == 0) {
            manifests.push_back(entry.path().string());
          }
        }
      }
      if (!manifests.empty()) {
        break;
      }
    }
  }

  std::sort(manifests.begin(), manifests.end());
  return manifests;
}

}  // namespace lf::sim
