#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "gpio.hpp"
#include "spi.hpp"

class EE {
 public:
  static constexpr uint32_t kFlashSize = 2u * 1024u * 1024u;
  static constexpr uint32_t kPageSize = 256u;
  static constexpr uint32_t kSectorSize = 4u * 1024u;
  static constexpr uint32_t kHeaderSize = 20u;
  static constexpr uint32_t kRecordAlign = 16u;
  static constexpr uint32_t kCapacity = kFlashSize - kHeaderSize;

  static EE &GetInstance() {
    static EE instance;
    return instance;
  }

  void Init(GPIO &gpio, Spi1 &spi);

  uint32_t Capacity() const { return kCapacity; }
  bool IsInitialized() const { return initialized_; }

  void Format();
  bool Read(void *dst, size_t len, size_t offset = 0) const;
  bool Write(const void *src, size_t len, size_t offset = 0);

  template <typename T>
  bool ReadObject(T &dst, size_t offset = 0) const {
    static_assert(std::is_trivially_copyable_v<T>);
    return Read(&dst, sizeof(T), offset);
  }

  template <typename T>
  bool WriteObject(const T &src, size_t offset = 0) {
    static_assert(std::is_trivially_copyable_v<T>);
    return Write(&src, sizeof(T), offset);
  }

 private:
  friend class System;

  struct RecordHeader {
    uint32_t magic;
    uint32_t sequence;
    uint32_t size;
    uint32_t payload_crc32;
    uint32_t header_crc32;
  };

  struct RecordState {
    uint32_t address;
    RecordHeader header;
    bool valid;
  };

  static_assert(sizeof(RecordHeader) == kHeaderSize);

  EE() = default;
  ~EE() = default;

  EE(const EE &) = delete;
  EE &operator=(const EE &) = delete;

  bool ReadJedecId(uint8_t jedec_id[3]) const;
  bool ReadStatus(uint8_t *status) const;
  bool WriteStatus(uint8_t status);
  bool EnsureWritable();
  bool WaitWhileBusy() const;
  bool WriteEnable() const;
  bool ReadRaw(uint32_t address, void *dst, size_t len) const;
  bool IsErased(uint32_t address, size_t len) const;
  bool CompareRaw(uint32_t address, const uint8_t *data, size_t len) const;
  bool EraseSector(uint32_t address);
  bool EraseRange(uint32_t begin, uint32_t end);
  bool PageProgram(uint32_t address, const uint8_t *src, size_t len);
  bool ProgramBytes(uint32_t address, const uint8_t *src, size_t len);

  static uint32_t Crc32(const void *data, size_t len);
  static uint32_t AlignUp(uint32_t value, uint32_t alignment);
  static uint32_t SectorBegin(uint32_t address);
  static uint32_t SectorEnd(uint32_t address);
  static bool RangesOverlap(uint32_t lhs_begin, uint32_t lhs_end,
                            uint32_t rhs_begin, uint32_t rhs_end);
  static uint32_t ComputeHeaderCrc32(const RecordHeader &header);
  static uint32_t RecordEnd(const RecordState &record);
  static uint32_t RecordSectorBegin(const RecordState &record);
  static uint32_t RecordSectorEnd(const RecordState &record);
  static uint32_t PayloadAddress(uint32_t record_address);
  uint32_t PayloadAddress(const RecordState &record) const;
  bool HeaderIsBlank(const RecordHeader &header) const;
  bool HeaderHasValidCrc(const RecordHeader &header) const;
  RecordHeader ReadHeader(uint32_t address) const;
  RecordState ReadRecord(uint32_t address) const;
  bool ComputePayloadCrc32(uint32_t address, uint32_t size,
                           uint32_t *crc32) const;
  bool ValidateRecord(const RecordState &record) const;
  RecordState ScanLatestRecord() const;
  bool PrepareWriteSpace(uint32_t record_size);
  bool EnsureRangeErased(uint32_t begin, uint32_t end, uint32_t preserve_begin,
                         uint32_t preserve_end);
  bool ReadLogical(uint32_t offset, void *dst, size_t len) const;
  bool CompareLogical(uint32_t offset, const uint8_t *data, size_t len) const;
  bool CheckRange(size_t len, size_t offset) const;
  void CsLow() const;
  void CsHigh() const;

  bool initialized_ = false;
  GPIO *gpio_ = nullptr;
  Spi1 *spi_ = nullptr;
  RecordState active_record_{};
  uint32_t next_write_address_ = 0u;
};
