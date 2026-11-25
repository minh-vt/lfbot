#include "course_store.hpp"

#include <cmath>
#include <cstdint>
#include <type_traits>

namespace lf::esp32 {
namespace {

static_assert(std::is_trivially_copyable_v<Stretch>);

constexpr std::uint32_t kMagic = 0x31435346U;  // "FSC1"
constexpr std::size_t kSector = 4096;
constexpr std::size_t kStretchesOffset = kSector;
constexpr auto kSubtype = static_cast<esp_partition_subtype_t>(0x41);

struct Header {
  std::uint32_t magic = 0;
  // A record is only meaningful to firmware that cuts the line the same way.
  std::uint32_t stretch_size = 0;
  std::uint32_t stretch_counts = 0;
  std::uint32_t length = 0;
  std::uint32_t checksum = 0;
};

std::uint32_t stretch_counts() {
  return static_cast<std::uint32_t>(std::lround(kStretchCounts));
}

// FNV-1a over the stretches in use.
std::uint32_t checksum_of(const CourseRecord& record) {
  std::uint32_t hash = 2166136261U;
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(record.stretches.data());
  for (std::size_t i = 0; i < record.length * sizeof(Stretch); ++i) {
    hash = (hash ^ bytes[i]) * 16777619U;
  }
  return hash;
}

std::size_t round_up(std::size_t value) {
  return (value + kSector - 1) / kSector * kSector;
}

}  // namespace

void CourseStore::initialize() {
  partition_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, kSubtype, "course");
}

bool CourseStore::load(CourseRecord& record) const {
  Header header;
  if (partition_ == nullptr ||
      esp_partition_read(partition_, 0, &header, sizeof(header)) != ESP_OK ||
      header.magic != kMagic || header.stretch_size != sizeof(Stretch) ||
      header.stretch_counts != stretch_counts() || header.length == 0 ||
      header.length > kCourseStretches) {
    return false;
  }
  record.length = header.length;
  return esp_partition_read(partition_, kStretchesOffset, record.stretches.data(),
                            header.length * sizeof(Stretch)) == ESP_OK &&
         checksum_of(record) == header.checksum;
}

bool CourseStore::save(const CourseRecord& record) const {
  if (partition_ == nullptr || record.length == 0 || record.length > kCourseStretches) {
    return false;
  }
  const std::size_t bytes = record.length * sizeof(Stretch);
  if (esp_partition_erase_range(partition_, 0, round_up(kStretchesOffset + bytes)) != ESP_OK ||
      esp_partition_write(partition_, kStretchesOffset, record.stretches.data(), bytes) != ESP_OK) {
    return false;
  }
  // Header last: a power cut half way leaves no valid record.
  Header header;
  header.magic = kMagic;
  header.stretch_size = sizeof(Stretch);
  header.stretch_counts = stretch_counts();
  header.length = record.length;
  header.checksum = checksum_of(record);
  return esp_partition_write(partition_, 0, &header, sizeof(header)) == ESP_OK;
}

bool CourseStore::erase() const {
  return partition_ != nullptr && esp_partition_erase_range(partition_, 0, kSector) == ESP_OK;
}

}  // namespace lf::esp32
