// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include "error_code.hpp"
#include "mbedtls/sha256.h"  // IWYU pragma: keep
#include "uart.hpp"

extern "C" {
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"  // IWYU pragma: keep
}

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "esp32_limits.hpp"

class Programmer {
 public:
  static Programmer &GetInstance();

  enum class Target { kStm32, kEsp32 };

  // How long a transfer may go without the written count advancing before the
  // host counts as gone.
  static constexpr uint32_t kStallTimeoutMs = 3000;

  struct Config {
    struct VerifyConfig {
      bool esp32 = true;
      std::size_t esp32_chunk_bytes = 1024;
      bool stm32 = true;

      bool EnabledFor(Target target) const {
        return target == Target::kEsp32 ? esp32 : stm32;
      }
    };

    // STM32 boot control pins on the ESP32 side.
    gpio_num_t boot0_pin = GPIO_NUM_NC;
    gpio_num_t nrst_pin = GPIO_NUM_NC;  // active low reset

    uint32_t reset_pulse_ms = 50;
    uint32_t boot_settle_ms = 50;

    uint32_t sync_timeout_ms = 100;
    uint8_t sync_retries = 10;
    VerifyConfig verify{};
  };

  // The wire token off the BEGIN line; "esp32" selects the ESP32, anything
  // else defaults to the STM32.
  void SetTarget(const char *name);

  void Start(uint32_t total_size);
  void Poll();
  void Abort();
  void WatchForStall();

  // Feed bytes and internally advance writing SM.
  // Returns bytes accepted (may be < n for backpressure).
  size_t PushBytes(std::span<const uint8_t> bytes);

  bool Ready() const { return ctx_.ready && !Error(); }
  bool Done() const { return phase_ == Phase::kDone; }
  bool Error() const { return phase_ == Phase::kError; }
  uint32_t LastErrorCode() const;
  bool IsVerifying() const { return phase_ == Phase::kVerifying; }
  uint32_t Total() const;
  uint32_t Written() const;
  uint32_t VerifyOffset() const;
  size_t Free() const;
  bool Boot();

 private:
  friend class System;

  // A lifecycle, not derivable from the session data: Writing and Verifying
  // do different per-tick work, and Done versus Error is a decision.
  enum class Phase : uint8_t { kIdle, kWriting, kVerifying, kDone, kError };

  void Init(const Config &cfg, UartFcLink *uart);

  // The programming session's data, reset by Start.
  struct Ctx {
    UartFcLink *uart = nullptr;
    Config cfg{};
    Target target = Target::kStm32;

    uint32_t total_size = 0;
    uint32_t written = 0;

    bool ready = false;
    uint32_t restore_baud_rate = 115200;

    // staging buffer for bytes coming from HTTP (backpressure lives here)
    static constexpr size_t kBufCap =
        esp32_limits::kProgrammerStagingBufferBytes;
    uint8_t buf[kBufCap]{};
    size_t head = 0;
    size_t tail = 0;
    bool overflow = false;

    // A target write must be one contiguous block, so a chunk that straddles
    // the ring's wrap is staged here rather than sent in two pieces.
    static constexpr size_t kWriteChunkCap = 4096;
    uint8_t block[kWriteChunkCap]{};
    static_assert(kWriteChunkCap < kBufCap,
                  "write chunk must fit in the staging ring");

    uint32_t err = static_cast<uint32_t>(ErrorCode::Common::kOk);

    // verification
    mbedtls_sha256_context sha_ctx;
    uint8_t computed_hash[32];
    uint32_t verify_offset = 0;

    // ESP32 OTA
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *ota_part = nullptr;
  };

  static size_t RbUsed(size_t head, size_t tail, size_t cap) {
    return (tail >= head) ? (tail - head) : (cap - head + tail);
  }
  static size_t RbFree(size_t head, size_t tail, size_t cap) {
    return (cap - 1) - RbUsed(head, tail, cap);
  }

  void StepWriting();
  void StepVerifying();
  void EnterVerifying();
  void EnterDone();
  void Fail(ErrorCode::Esp32 code);

  void GpioInit();
  void Boot0Set(bool on);
  void NrstPulse(uint32_t pulse_ms);
  bool BeginTargetSession();
  size_t TargetWriteChunkLimit() const;
  bool WriteTargetChunk(const uint8_t *bytes, size_t len);
  bool FinalizeTargetWrite();
  size_t TargetVerifyChunkSize() const;
  [[nodiscard]] std::optional<size_t> ReadTargetVerifyChunk(
      std::span<uint8_t> dst);
  bool CompleteSuccessfulProgram();

  bool BeginEsp32Ota();
  bool WriteEsp32Chunk(const uint8_t *bytes, size_t len);
  bool FinalizeEsp32Ota();
  bool ActivateEsp32Ota();
  bool ReadEsp32PartitionBlock(uint32_t offset, uint8_t *bytes,
                               size_t len) const;

  bool BeginStm32Session();
  bool EnterStm32Bootloader();    // BOOT0/reset/0x7F/ACK (bounded retries)
  bool GetStm32BootloaderInfo();  // CMD_GET (0x00)
  bool EraseStm32Sectors();       // STM32 targeted EXT_ERASE preserving EEPROM
  bool WriteStm32Block(uint32_t addr, const uint8_t *bytes,
                       size_t len);  // CMD_WRITE_MEMORY (0x31)
  bool ReadStm32Block(uint32_t addr, uint8_t *bytes,
                      size_t len);  // CMD_READ_MEMORY (0x11)
  Ctx ctx_{};
  Phase phase_ = Phase::kIdle;
  uint32_t stall_mark_ms_ = 0;
  uint32_t stall_written_ = 0;

  // Out of line so the instance in GetInstance cannot constant-initialize:
  // a few non-zero members would drag the whole 12 KB object into .data.
  Programmer();

  ~Programmer() = default;
  Programmer(const Programmer &) = delete;
  Programmer &operator=(const Programmer &) = delete;
};
