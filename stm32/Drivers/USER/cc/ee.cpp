#include "ee.hpp"

#include <algorithm>
#include <cstring>

#include "board.h"
#include "error_code.hpp"
#include "panic.hpp"

namespace {

constexpr uint8_t kCmdReadJedecId = 0x9Fu;
constexpr uint8_t kCmdReadStatusRegister1 = 0x05u;
constexpr uint8_t kCmdWriteEnable = 0x06u;
constexpr uint8_t kCmdWriteStatusRegister1 = 0x01u;
constexpr uint8_t kCmdReadData = 0x03u;
constexpr uint8_t kCmdSectorErase4K = 0x20u;
constexpr uint8_t kCmdPageProgram = 0x02u;

constexpr uint8_t kStatusBusyMask = 0x01u;
constexpr uint8_t kStatusWriteEnableLatchMask = 0x02u;
constexpr uint8_t kStatusProtectMask = 0xFCu;

constexpr uint8_t kJedecManufacturerWinbond = 0xEFu;
constexpr uint8_t kJedecCapacity16Mbit = 0x15u;

constexpr uint32_t kBusyPollLimit = 1000000u;
constexpr uint32_t kRecordMagic = 0x314C4545u;  // "EEL1"

uint32_t UpdateCrc32(uint32_t crc, uint8_t byte) {
  crc ^= byte;
  for (int i = 0; i < 8; ++i) {
    const uint32_t mask = -(crc & 1u);
    crc = (crc >> 1) ^ (0xEDB88320u & mask);
  }
  return crc;
}

bool BufferIsErased(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (data[i] != 0xFFu) {
      return false;
    }
  }
  return true;
}

}  // namespace

void EE::Init(GPIO &gpio, Spi1 &spi) {
  if (initialized_) {
    Panic(ErrorCode::kEepromReinit);
  }

  if (!spi.IsInitialized()) {
    Panic(ErrorCode::kEepromInvalidConfig);
  }

  gpio_ = &gpio;
  spi_ = &spi;

  CsHigh();

  uint8_t jedec_id[3] = {};
  if (!ReadJedecId(jedec_id) || jedec_id[0] != kJedecManufacturerWinbond ||
      jedec_id[2] != kJedecCapacity16Mbit) {
    Panic(ErrorCode::kEepromDeviceNotFound);
  }
  if (!EnsureWritable()) {
    Panic(ErrorCode::kEepromWriteFailed);
  }

  initialized_ = true;
  active_record_ = ScanLatestRecord();

  if (!active_record_.valid) {
    uint8_t probe[kHeaderSize] = {};
    const bool blank = ReadRaw(0u, probe, sizeof(probe)) &&
                       BufferIsErased(probe, sizeof(probe));
    if (!blank) {
      Format();
    }
    active_record_ = {};
    next_write_address_ = 0u;
  } else {
    next_write_address_ = AlignUp(RecordEnd(active_record_), kRecordAlign);
  }
}

void EE::Format() {
  if (!initialized_) {
    Panic(ErrorCode::kEepromNotInitialized);
  }

  for (uint32_t address = 0u; address < kFlashSize; address += kSectorSize) {
    if (!EraseSector(address)) {
      Panic(ErrorCode::kEepromFormatFailed);
    }
  }

  active_record_ = {};
  next_write_address_ = 0u;
}

bool EE::Read(void *dst, size_t len, size_t offset) const {
  if (dst == nullptr || !CheckRange(len, offset)) {
    return false;
  }

  auto *dst_bytes = static_cast<uint8_t *>(dst);
  std::memset(dst_bytes, 0xFF, len);
  if (!active_record_.valid || len == 0u) {
    return true;
  }

  if (offset >= active_record_.header.size) {
    return true;
  }

  const size_t readable =
      std::min(len, static_cast<size_t>(active_record_.header.size - offset));
  if (!ReadRaw(PayloadAddress(active_record_) + offset, dst, readable)) {
    return false;
  }
  return true;
}

bool EE::Write(const void *src, size_t len, size_t offset) {
  if (src == nullptr || !CheckRange(len, offset)) {
    return false;
  }

  if (len == 0u) {
    return true;
  }

  const uint8_t *src_bytes = static_cast<const uint8_t *>(src);
  if (CompareLogical(static_cast<uint32_t>(offset), src_bytes, len)) {
    return true;
  }

  const uint32_t current_size =
      active_record_.valid ? active_record_.header.size : 0u;
  const size_t write_end = offset + len;
  const uint32_t new_size =
      std::max(current_size, static_cast<uint32_t>(write_end));
  if (new_size > kCapacity) {
    return false;
  }

  const uint32_t record_size = kHeaderSize + new_size;
  const RecordState previous_record = active_record_;
  if (!PrepareWriteSpace(record_size)) {
    return false;
  }

  uint32_t crc32 = 0xFFFFFFFFu;
  uint8_t page[kPageSize] = {};
  for (uint32_t page_offset = 0u; page_offset < new_size;
       page_offset += kPageSize) {
    const size_t page_len =
        std::min(static_cast<size_t>(kPageSize),
                 static_cast<size_t>(new_size - page_offset));

    if (active_record_.valid && page_offset < current_size) {
      const size_t readable =
          std::min(page_len, static_cast<size_t>(current_size - page_offset));
      if (!ReadRaw(PayloadAddress(active_record_) + page_offset, page,
                   readable)) {
        return false;
      }
      if (readable < page_len) {
        std::memset(page + readable, 0xFF, page_len - readable);
      }
    } else {
      std::memset(page, 0xFF, page_len);
    }

    const size_t page_end = static_cast<size_t>(page_offset) + page_len;
    if (offset < page_end && write_end > page_offset) {
      const size_t dst_offset =
          (offset > page_offset) ? (offset - page_offset) : 0u;
      const size_t src_offset =
          (page_offset > offset) ? (page_offset - offset) : 0u;
      const size_t copy_begin = std::max(static_cast<size_t>(offset),
                                         static_cast<size_t>(page_offset));
      const size_t copy_end = std::min(write_end, page_end);
      const size_t copy_len = copy_end - copy_begin;
      std::memcpy(&page[dst_offset], &src_bytes[src_offset], copy_len);
    }

    for (size_t i = 0; i < page_len; ++i) {
      crc32 = UpdateCrc32(crc32, page[i]);
    }

    if (!ProgramBytes(PayloadAddress(next_write_address_) + page_offset, page,
                      page_len)) {
      return false;
    }
  }
  crc32 = ~crc32;

  RecordHeader header = {
      .magic = kRecordMagic,
      .sequence =
          active_record_.valid ? (active_record_.header.sequence + 1u) : 1u,
      .size = new_size,
      .payload_crc32 = crc32,
      .header_crc32 = 0u,
  };
  header.header_crc32 = ComputeHeaderCrc32(header);

  if (!ProgramBytes(next_write_address_,
                    reinterpret_cast<const uint8_t *>(&header),
                    sizeof(header))) {
    return false;
  }

  const RecordState verify = ReadRecord(next_write_address_);
  if (!verify.valid) {
    return false;
  }

  active_record_ = verify;
  next_write_address_ =
      AlignUp(next_write_address_ + record_size, kRecordAlign);

  if (previous_record.valid) {
    const uint32_t previous_begin = RecordSectorBegin(previous_record);
    const uint32_t previous_end = RecordSectorEnd(previous_record);
    const uint32_t active_begin = RecordSectorBegin(active_record_);
    const uint32_t active_end = RecordSectorEnd(active_record_);

    // Once the new record verifies, reclaiming the previous sectors is
    // best-effort cleanup. The committed data is already durable.
    if (!RangesOverlap(previous_begin, previous_end, active_begin,
                       active_end)) {
      static_cast<void>(EraseRange(previous_begin, previous_end));
    }
  }

  return true;
}

bool EE::ReadJedecId(uint8_t jedec_id[3]) const {
  if (jedec_id == nullptr || gpio_ == nullptr || spi_ == nullptr) {
    return false;
  }

  uint8_t tx[4] = {kCmdReadJedecId, 0xFFu, 0xFFu, 0xFFu};
  uint8_t rx[4] = {};
  CsLow();
  spi_->TxRx(tx, rx, sizeof(tx));
  CsHigh();

  jedec_id[0] = rx[1];
  jedec_id[1] = rx[2];
  jedec_id[2] = rx[3];
  return true;
}

bool EE::ReadStatus(uint8_t *status) const {
  if (status == nullptr || gpio_ == nullptr || spi_ == nullptr) {
    return false;
  }

  uint8_t tx[2] = {kCmdReadStatusRegister1, 0xFFu};
  uint8_t rx[2] = {};
  CsLow();
  spi_->TxRx(tx, rx, sizeof(tx));
  CsHigh();
  *status = rx[1];
  return true;
}

bool EE::WriteStatus(uint8_t status) {
  if (!WriteEnable()) {
    return false;
  }

  uint8_t cmd[2] = {kCmdWriteStatusRegister1, status};
  CsLow();
  spi_->Write(cmd, sizeof(cmd));
  CsHigh();

  if (!WaitWhileBusy()) {
    return false;
  }

  uint8_t verify = 0u;
  return ReadStatus(&verify) &&
         (verify & kStatusProtectMask) == (status & kStatusProtectMask);
}

bool EE::EnsureWritable() {
  uint8_t status = 0u;
  if (!ReadStatus(&status)) {
    return false;
  }

  if ((status & kStatusProtectMask) == 0u) {
    return true;
  }

  if (!WriteStatus(0x00u)) {
    return false;
  }

  if (!ReadStatus(&status)) {
    return false;
  }
  return (status & kStatusProtectMask) == 0u;
}

bool EE::WaitWhileBusy() const {
  uint8_t status = 0u;
  for (uint32_t attempt = 0; attempt < kBusyPollLimit; ++attempt) {
    if (!ReadStatus(&status)) {
      return false;
    }
    if ((status & kStatusBusyMask) == 0u) {
      return true;
    }
  }
  return false;
}

bool EE::WriteEnable() const {
  if (!WaitWhileBusy()) {
    return false;
  }

  uint8_t cmd = kCmdWriteEnable;
  CsLow();
  spi_->Write(&cmd, 1);
  CsHigh();

  uint8_t status = 0u;
  return ReadStatus(&status) && (status & kStatusWriteEnableLatchMask) != 0u;
}

bool EE::ReadRaw(uint32_t address, void *dst, size_t len) const {
  if ((dst == nullptr && len != 0u) || gpio_ == nullptr || spi_ == nullptr ||
      address > kFlashSize || len > (kFlashSize - address)) {
    return false;
  }

  if (len == 0u) {
    return true;
  }

  uint8_t cmd[4] = {
      kCmdReadData,
      static_cast<uint8_t>((address >> 16) & 0xFFu),
      static_cast<uint8_t>((address >> 8) & 0xFFu),
      static_cast<uint8_t>(address & 0xFFu),
  };

  CsLow();
  spi_->Write(cmd, sizeof(cmd));
  spi_->Read(static_cast<uint8_t *>(dst), len);
  CsHigh();
  return true;
}

bool EE::IsErased(uint32_t address, size_t len) const {
  if (address > kFlashSize || len > (kFlashSize - address)) {
    return false;
  }

  uint8_t buffer[64] = {};
  size_t offset = 0u;
  while (offset < len) {
    const size_t chunk = std::min(sizeof(buffer), len - offset);
    if (!ReadRaw(address + offset, buffer, chunk) ||
        !BufferIsErased(buffer, chunk)) {
      return false;
    }
    offset += chunk;
  }

  return true;
}

bool EE::CompareRaw(uint32_t address, const uint8_t *data, size_t len) const {
  if ((data == nullptr && len != 0u) || address > kFlashSize ||
      len > (kFlashSize - address)) {
    return false;
  }

  uint8_t buffer[32] = {};
  size_t offset = 0u;
  while (offset < len) {
    const size_t chunk = std::min(sizeof(buffer), len - offset);
    if (!ReadRaw(address + offset, buffer, chunk) ||
        std::memcmp(buffer, data + offset, chunk) != 0) {
      return false;
    }
    offset += chunk;
  }

  return true;
}

bool EE::EraseSector(uint32_t address) {
  if ((address % kSectorSize) != 0u || address > (kFlashSize - kSectorSize) ||
      !WriteEnable()) {
    return false;
  }

  uint8_t cmd[4] = {
      kCmdSectorErase4K,
      static_cast<uint8_t>((address >> 16) & 0xFFu),
      static_cast<uint8_t>((address >> 8) & 0xFFu),
      static_cast<uint8_t>(address & 0xFFu),
  };

  CsLow();
  spi_->Write(cmd, sizeof(cmd));
  CsHigh();
  return WaitWhileBusy();
}

bool EE::EraseRange(uint32_t begin, uint32_t end) {
  if (begin > end || end > kFlashSize || (begin % kSectorSize) != 0u ||
      (end % kSectorSize) != 0u) {
    return false;
  }

  for (uint32_t address = begin; address < end; address += kSectorSize) {
    if (!EraseSector(address)) {
      return false;
    }
  }

  return true;
}

bool EE::PageProgram(uint32_t address, const uint8_t *src, size_t len) {
  if ((src == nullptr && len != 0u) || len > kPageSize ||
      address > kFlashSize || len > (kFlashSize - address)) {
    return false;
  }

  if (len == 0u) {
    return true;
  }

  const uint32_t page_start = address / kPageSize;
  const uint32_t page_end = (address + len - 1u) / kPageSize;
  if (page_start != page_end || !WriteEnable()) {
    return false;
  }

  uint8_t cmd[4] = {
      kCmdPageProgram,
      static_cast<uint8_t>((address >> 16) & 0xFFu),
      static_cast<uint8_t>((address >> 8) & 0xFFu),
      static_cast<uint8_t>(address & 0xFFu),
  };

  CsLow();
  spi_->Write(cmd, sizeof(cmd));
  spi_->Write(src, len);
  CsHigh();
  return WaitWhileBusy();
}

bool EE::ProgramBytes(uint32_t address, const uint8_t *src, size_t len) {
  if ((src == nullptr && len != 0u) || address > kFlashSize ||
      len > (kFlashSize - address)) {
    return false;
  }

  size_t remaining = len;
  uint32_t cursor = address;
  const uint8_t *current = src;
  while (remaining > 0u) {
    const size_t page_offset = cursor % kPageSize;
    const size_t chunk =
        std::min(remaining, static_cast<size_t>(kPageSize - page_offset));
    if (!PageProgram(cursor, current, chunk)) {
      return false;
    }
    cursor += chunk;
    current += chunk;
    remaining -= chunk;
  }

  return true;
}

uint32_t EE::Crc32(const void *data, size_t len) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc = UpdateCrc32(crc, bytes[i]);
  }
  return ~crc;
}

uint32_t EE::AlignUp(uint32_t value, uint32_t alignment) {
  return (value + alignment - 1u) / alignment * alignment;
}

uint32_t EE::SectorBegin(uint32_t address) {
  return (address / kSectorSize) * kSectorSize;
}

uint32_t EE::SectorEnd(uint32_t address) {
  return AlignUp(address, kSectorSize);
}

bool EE::RangesOverlap(uint32_t lhs_begin, uint32_t lhs_end, uint32_t rhs_begin,
                       uint32_t rhs_end) {
  return lhs_begin < rhs_end && rhs_begin < lhs_end;
}

uint32_t EE::ComputeHeaderCrc32(const RecordHeader &header) {
  RecordHeader copy = header;
  copy.header_crc32 = 0u;
  return Crc32(&copy, sizeof(copy));
}

uint32_t EE::RecordEnd(const RecordState &record) {
  return record.address + kHeaderSize + record.header.size;
}

uint32_t EE::RecordSectorBegin(const RecordState &record) {
  return SectorBegin(record.address);
}

uint32_t EE::RecordSectorEnd(const RecordState &record) {
  return SectorEnd(RecordEnd(record));
}

uint32_t EE::PayloadAddress(uint32_t record_address) {
  return record_address + kHeaderSize;
}

uint32_t EE::PayloadAddress(const RecordState &record) const {
  return PayloadAddress(record.address);
}

bool EE::HeaderIsBlank(const RecordHeader &header) const {
  return header.magic == 0xFFFFFFFFu && header.sequence == 0xFFFFFFFFu &&
         header.size == 0xFFFFFFFFu && header.payload_crc32 == 0xFFFFFFFFu &&
         header.header_crc32 == 0xFFFFFFFFu;
}

bool EE::HeaderHasValidCrc(const RecordHeader &header) const {
  return header.header_crc32 == ComputeHeaderCrc32(header);
}

EE::RecordHeader EE::ReadHeader(uint32_t address) const {
  RecordHeader header{};
  if (!ReadRaw(address, &header, sizeof(header))) {
    std::memset(&header, 0, sizeof(header));
  }
  return header;
}

EE::RecordState EE::ReadRecord(uint32_t address) const {
  RecordState record = {
      .address = address,
      .header = ReadHeader(address),
      .valid = false,
  };

  if (HeaderIsBlank(record.header)) {
    return record;
  }

  record.valid = ValidateRecord(record);
  return record;
}

bool EE::ComputePayloadCrc32(uint32_t address, uint32_t size,
                             uint32_t *crc32) const {
  if (crc32 == nullptr) {
    return false;
  }

  uint8_t buffer[kPageSize] = {};
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t offset = 0u; offset < size; offset += kPageSize) {
    const size_t chunk = std::min(static_cast<size_t>(kPageSize),
                                  static_cast<size_t>(size - offset));
    if (!ReadRaw(address + offset, buffer, chunk)) {
      return false;
    }
    for (size_t i = 0; i < chunk; ++i) {
      crc = UpdateCrc32(crc, buffer[i]);
    }
  }

  *crc32 = ~crc;
  return true;
}

bool EE::ValidateRecord(const RecordState &record) const {
  if (HeaderIsBlank(record.header) || record.header.magic != kRecordMagic ||
      record.header.size > kCapacity) {
    return false;
  }

  if (record.address > (kFlashSize - (kHeaderSize + record.header.size))) {
    return false;
  }

  if (!HeaderHasValidCrc(record.header)) {
    return false;
  }

  uint32_t crc32 = 0u;
  return ComputePayloadCrc32(PayloadAddress(record.address), record.header.size,
                             &crc32) &&
         record.header.payload_crc32 == crc32;
}

EE::RecordState EE::ScanLatestRecord() const {
  RecordState latest{};
  for (uint32_t sector = 0u; sector < kFlashSize; sector += kSectorSize) {
    const RecordHeader sector_probe = ReadHeader(sector);

    // Fresh sectors are always erased from the start because writes only enter
    // a clean sector at its boundary.
    if (HeaderIsBlank(sector_probe)) {
      continue;
    }

    for (uint32_t offset = 0u; offset + kHeaderSize <= kSectorSize;
         offset += kRecordAlign) {
      const uint32_t cursor = sector + offset;
      RecordState record =
          (offset == 0u)
              ? RecordState{
                    .address = cursor,
                    .header = sector_probe,
                    .valid = false,
                }
              : ReadRecord(cursor);
      if (offset == 0u) {
        if (!HeaderIsBlank(record.header) &&
            record.header.magic == kRecordMagic &&
            HeaderHasValidCrc(record.header)) {
          record.valid = ValidateRecord(record);
        } else {
          record = ReadRecord(cursor);
        }
      }

      if (!record.valid) {
        continue;
      }

      if (!latest.valid || record.header.sequence >= latest.header.sequence) {
        latest = record;
      }
    }
  }
  return latest;
}

bool EE::PrepareWriteSpace(uint32_t record_size) {
  const uint32_t preserve_begin =
      active_record_.valid ? RecordSectorBegin(active_record_) : kFlashSize;
  const uint32_t preserve_end =
      active_record_.valid ? RecordSectorEnd(active_record_) : kFlashSize;

  const auto try_prepare = [&](uint32_t candidate) -> bool {
    if (candidate > kFlashSize || record_size > (kFlashSize - candidate)) {
      return false;
    }
    if (!EnsureRangeErased(candidate, candidate + record_size, preserve_begin,
                           preserve_end)) {
      return false;
    }
    next_write_address_ = candidate;
    return true;
  };

  if (try_prepare(next_write_address_)) {
    return true;
  }

  if (!active_record_.valid) {
    return false;
  }

  const uint32_t active_begin = RecordSectorBegin(active_record_);
  const uint32_t active_end = RecordSectorEnd(active_record_);
  const uint32_t prefix_size = active_begin;
  const uint32_t suffix_size = kFlashSize - active_end;
  const bool prefix_fits = record_size <= prefix_size;
  const bool suffix_fits = record_size <= suffix_size;

  if (!prefix_fits && !suffix_fits) {
    return false;
  }

  if (prefix_fits && (!suffix_fits || prefix_size >= suffix_size)) {
    return try_prepare(0u) || (suffix_fits && try_prepare(active_end));
  }

  return try_prepare(active_end) || (prefix_fits && try_prepare(0u));
}

bool EE::EnsureRangeErased(uint32_t begin, uint32_t end,
                           uint32_t preserve_begin, uint32_t preserve_end) {
  if (begin > end || end > kFlashSize) {
    return false;
  }

  for (uint32_t sector = SectorBegin(begin); sector < SectorEnd(end);
       sector += kSectorSize) {
    const uint32_t sector_end = sector + kSectorSize;
    const uint32_t overlap_begin = std::max(begin, sector);
    const uint32_t overlap_end = std::min(end, sector_end);

    if (RangesOverlap(sector, sector_end, preserve_begin, preserve_end)) {
      if (!IsErased(overlap_begin, overlap_end - overlap_begin)) {
        return false;
      }
      continue;
    }

    if (!IsErased(sector, kSectorSize) && !EraseSector(sector)) {
      return false;
    }
  }

  return true;
}

bool EE::ReadLogical(uint32_t offset, void *dst, size_t len) const {
  return Read(dst, len, offset);
}

bool EE::CompareLogical(uint32_t offset, const uint8_t *data,
                        size_t len) const {
  if ((data == nullptr && len != 0u) || !CheckRange(len, offset)) {
    return false;
  }

  uint8_t buffer[32] = {};
  size_t cursor = 0u;
  while (cursor < len) {
    const size_t chunk = std::min(sizeof(buffer), len - cursor);
    if (!ReadLogical(offset + static_cast<uint32_t>(cursor), buffer, chunk) ||
        std::memcmp(buffer, data + cursor, chunk) != 0) {
      return false;
    }
    cursor += chunk;
  }

  return true;
}

bool EE::CheckRange(size_t len, size_t offset) const {
  if (!initialized_ || offset > kCapacity) {
    return false;
  }
  return len <= (kCapacity - offset);
}

void EE::CsLow() const {
  gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, false);
}

void EE::CsHigh() const {
  gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, true);
}
