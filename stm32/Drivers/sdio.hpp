// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <span>

#include "outcome.hpp"

// SD card block device on the SDIO peripheral, 4-bit bus, DMA2 Stream 3.
// Meaning-free: 512-byte blocks in and out, no files, no ownership policy.
// SDHC/SDXC only -- byte-addressed V1 cards are refused at probe. A missing
// or failed card is a degraded bench, not a fault: nothing panics after init
// and callers carry on once CardPresent() says no.
class Sdio {
 public:
  static constexpr uint32_t kBlockBytes = 512;

  // kDone and kError report once, then the driver is idle again.
  enum class WriteStatus : uint8_t { kIdle, kBusy, kDone, kError };

  struct Stats {
    uint32_t cmd_timeouts = 0;
    uint32_t cmd_crc_errors = 0;
    uint32_t data_crc_errors = 0;
    uint32_t data_timeouts = 0;
    uint32_t write_errors = 0;
    uint32_t unaligned_refusals = 0;
  };

  static Sdio &GetInstance();

  bool CardPresent() const { return present_; }
  uint32_t BlockCount() const { return block_count_; }
  const Stats &GetStats() const { return stats_; }

  // Blocking, bench-tier. Length must be a nonzero multiple of kBlockBytes
  // and the address 4-byte aligned (DMA moves words), else refused.
  //
  // kRejected is the one worth acting on: it means a write is still in flight,
  // so the same call succeeds once PollWrite finishes. kInvalid is a caller
  // bug or an absent card, and the rest are the card misbehaving -- Stats
  // keeps the tally that says which, across calls.
  [[nodiscard]] Outcome ReadBlocks(uint32_t lba, std::span<uint8_t> dst);
  [[nodiscard]] Outcome WriteBlocks(uint32_t lba, std::span<const uint8_t> src);

  // Non-blocking: the buffer belongs to DMA until PollWrite reports kDone or
  // kError. Each PollWrite costs at most one command exchange (~3 us).
  [[nodiscard]] Outcome StartWrite(uint32_t lba, std::span<const uint8_t> src);
  WriteStatus PollWrite();

  bool Reprobe();

 private:
  friend class System;
  void Init();

  Sdio() = default;
  ~Sdio() = default;
  Sdio(const Sdio &) = delete;
  Sdio &operator=(const Sdio &) = delete;

  enum class WritePhase : uint8_t { kIdle, kData, kProgramming };
  // R3 carries no CRC, so the CPSM always flags CCRCFAIL for it: there the
  // flag means "response arrived", not "response corrupt".
  enum class Resp : uint8_t { kNone, kShort, kShortNoCrc, kLong };

  // ACMDs are a separate type: only SendAppCommand sends them, CMD55 and all.
  enum class Cmd : uint8_t {
    kGoIdle = 0,
    kAllSendCid = 2,
    kSendRelAddr = 3,
    kSelect = 7,
    kSendIfCond = 8,
    kSendCsd = 9,
    kStop = 12,
    kSendStatus = 13,
    kReadSingle = 17,
    kReadMulti = 18,
    kWriteSingle = 24,
    kWriteMulti = 25,
    kApp = 55,
  };
  enum class Acmd : uint8_t { kSetBusWidth = 6, kOpCond = 41 };
  enum class Dir : uint8_t { kToCard, kFromCard };

  bool ProbeCard();
  void PowerUpBus();
  Outcome SendCommand(Cmd cmd, uint32_t arg, Resp resp);
  Outcome SendAppCommand(Acmd cmd, uint32_t arg, Resp resp);
  bool WaitCardReady(uint32_t timeout_us);
  void ConfigureDma(const uint8_t *buf, Dir dir);
  void StopDma();
  void PrepareDataPath(uint32_t byte_len, Dir dir);
  void AbortTransfer();

  bool initialized_ = false;
  bool present_ = false;
  uint32_t block_count_ = 0;
  uint16_t rca_ = 0;

  WritePhase write_phase_ = WritePhase::kIdle;
  bool write_multi_ = false;
  uint32_t write_deadline_us_ = 0;

  Stats stats_{};
};
