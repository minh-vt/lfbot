#include "blackbox.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <type_traits>

namespace lf::esp32 {
namespace {

static_assert(std::is_trivially_copyable_v<LogRecord>);

constexpr std::uint32_t kMagic = 0x32424B4CU;  // "LKB2": the slot layout with a sequence number
constexpr std::size_t kSector = 4096;
constexpr std::size_t kSlotSize = 0x40000;
constexpr std::size_t kRecordsOffset = kSector;
static_assert(kRecordsOffset + LogRing::kCapacity * sizeof(LogRecord) <= kSlotSize,
              "a full log must fit in one slot");
constexpr auto kSubtype = static_cast<esp_partition_subtype_t>(0x40);

std::size_t round_up(std::size_t value) {
  return (value + kSector - 1) / kSector * kSector;
}

}  // namespace

void Blackbox::initialize() {
  partition_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, kSubtype, "blackbox");
}

bool Blackbox::read_header(std::size_t slot, Header& header) const {
  return partition_ != nullptr &&
         esp_partition_read(partition_, slot * kSlotSize, &header, sizeof(header)) == ESP_OK &&
         header.magic == kMagic && header.record_size == sizeof(LogRecord);
}

std::size_t Blackbox::slot_of(std::size_t run) const {
  // Slots hold sequence numbers; the newest run is the largest.
  std::array<std::uint32_t, kSlots> sequence{};
  std::array<bool, kSlots> valid{};
  for (std::size_t slot = 0; slot < kSlots; ++slot) {
    Header header;
    valid[slot] = read_header(slot, header);
    sequence[slot] = header.sequence;
  }
  for (std::size_t rank = 0; rank <= run; ++rank) {
    std::size_t best = kSlots;
    for (std::size_t slot = 0; slot < kSlots; ++slot) {
      if (valid[slot] && (best == kSlots || sequence[slot] > sequence[best])) {
        best = slot;
      }
    }
    if (best == kSlots) {
      return kSlots;
    }
    if (rank == run) {
      return best;
    }
    valid[best] = false;
  }
  return kSlots;
}

bool Blackbox::save(const LogRing& log) const {
  if (partition_ == nullptr || log.size() == 0) {
    return false;
  }
  // The new run goes in the slot after the newest one.
  const std::size_t newest = slot_of(0);
  Header previous;
  const bool have_previous = newest < kSlots && read_header(newest, previous);
  const std::size_t slot = have_previous ? (newest + 1) % kSlots : 0;
  const std::uint32_t sequence = have_previous ? previous.sequence + 1 : 1;
  const std::size_t base = slot * kSlotSize;

  const std::size_t count = log.size();
  const std::size_t bytes = round_up(kRecordsOffset + count * sizeof(LogRecord));
  if (esp_partition_erase_range(partition_, base, bytes) != ESP_OK) {
    return false;
  }
  // Records first, header last: a power cut half way leaves no valid header.
  std::array<std::uint8_t, kSector> chunk{};
  std::size_t filled = 0;
  std::size_t offset = base + kRecordsOffset;
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
  header.magic = kMagic;
  header.record_size = sizeof(LogRecord);
  header.count = static_cast<std::uint32_t>(count);
  header.sequence = sequence;
  return esp_partition_write(partition_, base, &header, sizeof(header)) == ESP_OK;
}

bool Blackbox::describe(std::size_t slot, std::uint32_t& sequence, std::uint32_t& count) const {
  Header header;
  if (!read_header(slot, header)) {
    return false;
  }
  sequence = header.sequence;
  count = header.count;
  return true;
}

std::size_t Blackbox::count(std::size_t run) const {
  Header header;
  const std::size_t slot = slot_of(run);
  return slot < kSlots && read_header(slot, header) ? header.count : 0;
}

bool Blackbox::read(std::size_t run, std::size_t index, LogRecord& record) const {
  const std::size_t slot = slot_of(run);
  return slot < kSlots &&
         esp_partition_read(partition_,
                            slot * kSlotSize + kRecordsOffset + index * sizeof(LogRecord), &record,
                            sizeof(LogRecord)) == ESP_OK;
}

}  // namespace lf::esp32
