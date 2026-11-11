#include "blackbox.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace lf::esp32 {
namespace {

static_assert(std::is_trivially_copyable_v<LogRecord>);

constexpr std::uint32_t kMagic = 0x4B42464CU;  // "LFBK"
constexpr std::size_t kSector = 4096;
constexpr std::size_t kRecordsOffset = kSector;
constexpr auto kSubtype = static_cast<esp_partition_subtype_t>(0x40);

struct Header {
  std::uint32_t magic = kMagic;
  std::uint32_t record_size = sizeof(LogRecord);
  std::uint32_t count = 0;
};

std::size_t round_up(std::size_t value) {
  return (value + kSector - 1) / kSector * kSector;
}

}  // namespace

void Blackbox::initialize() {
  partition_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, kSubtype, "blackbox");
}

bool Blackbox::save(const LogRing& log) const {
  if (partition_ == nullptr || log.size() == 0) {
    return false;
  }
  const std::size_t count = log.size();
  const std::size_t bytes = round_up(kRecordsOffset + count * sizeof(LogRecord));
  if (bytes > partition_->size || esp_partition_erase_range(partition_, 0, bytes) != ESP_OK) {
    return false;
  }
  // Records first, header last: a power cut half way leaves no valid header.
  std::array<std::uint8_t, kSector> chunk{};
  std::size_t filled = 0;
  std::size_t offset = kRecordsOffset;
  const auto flush = [&]() {
    const bool ok =
      filled == 0 || esp_partition_write(partition_, offset, chunk.data(), filled) == ESP_OK;
    offset += filled;
    filled = 0;
    return ok;
  };
  for (std::size_t i = 0; i < count; ++i) {
    const LogRecord& record = log.at(i);
    const auto* source = reinterpret_cast<const std::uint8_t*>(&record);
    std::size_t copied = 0;
    while (copied < sizeof(LogRecord)) {
      const std::size_t room = std::min(chunk.size() - filled, sizeof(LogRecord) - copied);
      std::memcpy(chunk.data() + filled, source + copied, room);
      filled += room;
      copied += room;
      if (filled == chunk.size() && !flush()) {
        return false;
      }
    }
  }
  if (!flush()) {
    return false;
  }
  Header header;
  header.count = static_cast<std::uint32_t>(count);
  return esp_partition_write(partition_, 0, &header, sizeof(header)) == ESP_OK;
}

std::size_t Blackbox::count() const {
  Header header;
  if (partition_ == nullptr ||
      esp_partition_read(partition_, 0, &header, sizeof(header)) != ESP_OK ||
      header.magic != kMagic || header.record_size != sizeof(LogRecord)) {
    return 0;
  }
  return header.count;
}

bool Blackbox::read(std::size_t index, LogRecord& record) const {
  return partition_ != nullptr &&
         esp_partition_read(partition_, kRecordsOffset + index * sizeof(LogRecord), &record,
                            sizeof(LogRecord)) == ESP_OK;
}

}  // namespace lf::esp32
