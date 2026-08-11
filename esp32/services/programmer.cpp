// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "programmer.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <optional>
#include <span>

#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "driver/gpio.h"
#include "esp_log.h"
}

static constexpr char kTag[] = "programmer";

namespace {

// AN3155 caps a single READ_MEM or WRITE_MEM at 256 bytes.
constexpr std::size_t kStm32BlockBytes = 256;

// STM32 ROM bootloader (AN3155) replies. Plain bytes, not an enum: they come
// off a UART from a device that may be mid-reset and can hold anything.
constexpr uint8_t kAck = 0x79;
constexpr uint8_t kNack = 0x1F;
constexpr uint8_t kSyncByte = 0x7F;

enum class Stm32Cmd : uint8_t {
  kGet = 0x00,
  kReadMemory = 0x11,
  kGo = 0x21,
  kWriteMemory = 0x31,
  kEraseMemory = 0x43,
  kExtErase = 0x44,
};

// AN3155 frames every command as the opcode followed by its complement.
constexpr std::array<uint8_t, 2> CmdFrame(Stm32Cmd cmd) {
  const auto v = static_cast<uint8_t>(cmd);
  return {v, static_cast<uint8_t>(~v)};
}

constexpr auto kCmdGet = CmdFrame(Stm32Cmd::kGet);
constexpr auto kCmdReadMemory = CmdFrame(Stm32Cmd::kReadMemory);
constexpr auto kCmdWriteMemory = CmdFrame(Stm32Cmd::kWriteMemory);
constexpr auto kCmdExtErase = CmdFrame(Stm32Cmd::kExtErase);

[[nodiscard]] constexpr std::array<uint8_t, 5> Be32WithXorChecksum(
    uint32_t addr) {
  const uint8_t b3 = static_cast<uint8_t>(addr >> 24);
  const uint8_t b2 = static_cast<uint8_t>(addr >> 16);
  const uint8_t b1 = static_cast<uint8_t>(addr >> 8);
  const uint8_t b0 = static_cast<uint8_t>(addr);
  return {b3, b2, b1, b0, static_cast<uint8_t>(b3 ^ b2 ^ b1 ^ b0)};
}

struct FlashSector {
  uint16_t number;
  uint32_t address;
  uint32_t size;
};

struct Stm32FlashLayout {
  static constexpr uint32_t kBase = 0x08000000u;
  static constexpr uint32_t kFlashSize = 1024u * 1024u;

  static constexpr auto kSectors = std::to_array<FlashSector>({
      {0, 0x08000000u, 16u * 1024u},
      {1, 0x08004000u, 16u * 1024u},
      {2, 0x08008000u, 16u * 1024u},
      {3, 0x0800C000u, 16u * 1024u},
      {4, 0x08010000u, 64u * 1024u},
      {5, 0x08020000u, 128u * 1024u},
      {6, 0x08040000u, 128u * 1024u},
      {7, 0x08060000u, 128u * 1024u},
      {8, 0x08080000u, 128u * 1024u},
      {9, 0x080A0000u, 128u * 1024u},
      {10, 0x080C0000u, 128u * 1024u},
      {11, 0x080E0000u, 128u * 1024u},
  });

  struct Placement {
    uint32_t flash_addr;
    size_t max_chunk;
  };

  [[nodiscard]] static constexpr std::optional<Placement> ResolveOffset(
      uint32_t offset, uint32_t total_size) {
    if (offset >= total_size || total_size > kFlashSize) {
      return std::nullopt;
    }

    return Placement{kBase + offset, total_size - offset};
  }

  [[nodiscard]] static constexpr bool ContainsOffset(uint32_t offset,
                                                     uint32_t total_size) {
    return ResolveOffset(offset, total_size).has_value();
  }
};

constexpr uint32_t SectorTableBytes() {
  uint32_t sum = 0;
  for (const auto &sector : Stm32FlashLayout::kSectors) {
    sum += sector.size;
  }
  return sum;
}

// EraseStm32Sectors walks this table until the image is covered, and relies on
// an image bounded by kFlashSize never outrunning it.
static_assert(SectorTableBytes() == Stm32FlashLayout::kFlashSize,
              "sector table must cover the whole flash");

}  // namespace

void Programmer::Init(const Config &cfg, UartFcLink *uart) {
  ctx_.cfg = cfg;
  ctx_.uart = uart;
  ctx_.sm = &sm_;

  if (!ctx_.uart) {
    Panic(ErrorCode::Esp32::kProgrammerUartNull);
  }
  ctx_.restore_baud_rate = ctx_.uart->GetConfig().line.baud_rate;

  ctx_.st_verifying = &StVerifying_;
  ctx_.st_done = &StDone_;

  GpioInit();

  // Start in idle
  ctx_.ready = false;
  ctx_.err = static_cast<uint32_t>(ErrorCode::Common::kOk);
  ctx_.total_size = 0;
  ctx_.written = 0;
  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;

  ctx_.uart->Flush();

  // Ensure STM32 is reset on ESP32 boot
  NrstPulse(ctx_.cfg.reset_pulse_ms);
  Sys().Timebase().SleepMs(ctx_.cfg.boot_settle_ms);

  sm_.Start(StIdle_);
}
void Programmer::GpioInit() {
  const gpio_num_t boot0 = ctx_.cfg.boot0_pin;
  const gpio_num_t nrst = ctx_.cfg.nrst_pin;
  if (boot0 != GPIO_NUM_NC) {
    gpio_config_t io{};
    io.pin_bit_mask = (1ULL << static_cast<unsigned>(boot0));
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // Default BOOT0 low (normal boot)
    Boot0Set(false);
  }

  if (nrst != GPIO_NUM_NC) {
    gpio_config_t io{};
    io.pin_bit_mask = (1ULL << static_cast<unsigned>(nrst));
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // Default NRST deasserted (high for active-low reset)
    gpio_set_level(nrst, 1);
  }
}

void Programmer::Boot0Set(bool on) {
  const gpio_num_t pin = ctx_.cfg.boot0_pin;
  if (pin == GPIO_NUM_NC) return;

  // BOOT0 is assumed active-high
  gpio_set_level(pin, on ? 1 : 0);
}

void Programmer::NrstPulse(uint32_t pulse_ms) {
  const gpio_num_t pin = ctx_.cfg.nrst_pin;
  if (pin == GPIO_NUM_NC) return;

  // NRST is assumed active-low (assert=0, deassert=1)
  const int assert_level = 0;
  const int deassert_level = 1;

  gpio_set_level(pin, assert_level);
  Sys().Timebase().SleepMs(pulse_ms);
  gpio_set_level(pin, deassert_level);
}

bool Programmer::EnterStm32Bootloader() {
  // Put STM32 into ROM bootloader: BOOT0=1, reset pulse, settle
  Boot0Set(true);
  NrstPulse(ctx_.cfg.reset_pulse_ms);
  Sys().Timebase().SleepMs(ctx_.cfg.boot_settle_ms);

  // Flush any junk
  ctx_.uart->Flush();

  // Switch to standard baud rate for ROM bootloader
  ctx_.uart->SetBaudRate(115200);
  Sys().Timebase().SleepMs(10);

  // STM32 ROM bootloader sync:
  // Host sends 0x7F, device replies 0x79 (ACK) or 0x1F (NACK)
  uint8_t tx = kSyncByte;

  uint8_t retries = ctx_.cfg.sync_retries;
  while (retries--) {
    // Send sync
    (void)ctx_.uart->WriteByte(tx);

    // Ensure byte physically left UART
    ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

    // Wait for ACK. Some boards can leave a stale byte in the RX FIFO during
    // the reset-to-bootloader transition; keep reading until timeout so one
    // stray byte does not make us miss the actual ACK.
    const TimeMs deadline =
        TimeAfter(Sys().Timebase().NowMs(),
                  static_cast<TimeMs>(ctx_.cfg.sync_timeout_ms));
    uint16_t unexpected_count = 0;
    uint8_t last_unexpected = 0;
    bool saw_nack = false;

    while (!TimeReached(Sys().Timebase().NowMs(), deadline)) {
      const TimeMs now = Sys().Timebase().NowMs();
      TimeMs remaining = deadline - now;
      if (remaining > 10) {
        remaining = 10;
      }

      const auto rx = ctx_.uart->ReadByte(remaining);
      if (!rx) {
        continue;
      }

      if (*rx == kAck) {
        if (unexpected_count > 0) {
          ESP_LOGW(kTag,
                   "STM32 Connect ignored %u unexpected byte(s), last=0x%02X",
                   static_cast<unsigned>(unexpected_count), last_unexpected);
        }
        ESP_LOGI(kTag, "STM32 Connect ACK (0x79)");
        return true;  // ACK
      }

      if (*rx == kNack) {
        saw_nack = true;
        break;
      }

      last_unexpected = *rx;
      ++unexpected_count;
    }

    if (unexpected_count > 0) {
      ESP_LOGW(kTag, "STM32 Connect ignored %u unexpected byte(s), last=0x%02X",
               static_cast<unsigned>(unexpected_count), last_unexpected);
    }

    if (saw_nack) {
      ESP_LOGW(kTag, "STM32 Connect NACK (0x1F)");
    }

    // small delay between retries
    Sys().Timebase().SleepMs(10);
    ctx_.uart->Flush();
  }

  return false;
}

bool Programmer::GetStm32BootloaderInfo() {
  // CMD_GET: 0x00 0xFF
  ctx_.uart->Flush();
  ctx_.uart->WriteBytes(kCmdGet);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  // Expect: ACK + N + Version + N bytes + ACK
  // N = number of bytes to follow - 1
  const auto ack = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (ack != kAck) {
    ESP_LOGE(kTag, "CMD_GET failed to get initial ACK (0x%02X)",
             ack.value_or(0));
    return false;
  }

  const auto len = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (!len) {
    ESP_LOGE(kTag, "CMD_GET failed to get length");
    return false;
  }

  const auto ver = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (!ver) {
    ESP_LOGE(kTag, "CMD_GET failed to get version");
    return false;
  }

  ESP_LOGI(kTag, "STM32 Bootloader v%X.%X", *ver >> 4, *ver & 0xF);

  // Version was the first of the N + 1 bytes N counts, so N remain: the
  // supported-command list. Drained to stay framed for the final ACK, not
  // inspected — BeginStm32Session proceeds whatever this returns, so a
  // capability check here could only mislead.
  if (*len > 0) {
    uint8_t cmds[UINT8_MAX];  // *len is a uint8_t, so it always fits
    if (ctx_.uart->ReadBytes({cmds, *len}, ctx_.cfg.sync_timeout_ms) != *len) {
      ESP_LOGE(kTag, "CMD_GET failed to get commands");
      return false;
    }
  }

  const auto final_ack = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (final_ack != kAck) {
    ESP_LOGE(kTag, "CMD_GET missing final ACK");
    return false;
  }

  return true;
}

bool Programmer::EraseStm32Sectors() {
  if (ctx_.total_size == 0 || ctx_.total_size > Stm32FlashLayout::kFlashSize) {
    ESP_LOGE(kTag, "STM32 image size %u is invalid", (unsigned)ctx_.total_size);
    return false;
  }

  std::array<uint16_t, Stm32FlashLayout::kSectors.size()> sectors{};
  size_t sector_count = 0;
  uint32_t remaining = ctx_.total_size;

  for (const auto &sector : Stm32FlashLayout::kSectors) {
    sectors[sector_count++] = sector.number;
    if (remaining <= sector.size) {
      break;
    }
    remaining -= sector.size;
  }

  ESP_LOGI(kTag, "Sending EXT_ERASE (0x44) for %u sector(s)...",
           (unsigned)sector_count);

  ctx_.uart->Flush();
  ctx_.uart->WriteBytes(kCmdExtErase);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  const auto initial_ack = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (initial_ack != kAck) {
    ESP_LOGE(kTag, "EXT_ERASE failed to get initial ACK (0x%02X)",
             initial_ack.value_or(0));
    return false;
  }

  const uint16_t count_minus_one = static_cast<uint16_t>(sector_count - 1u);
  std::array<uint8_t, 2 + 2 * Stm32FlashLayout::kSectors.size() + 1> payload{};
  size_t payload_len = 0;
  uint8_t checksum = 0;

  payload[payload_len++] = (count_minus_one >> 8) & 0xFF;
  payload[payload_len++] = count_minus_one & 0xFF;
  checksum ^= payload[0];
  checksum ^= payload[1];

  for (size_t i = 0; i < sector_count; ++i) {
    const uint8_t hi = (sectors[i] >> 8) & 0xFF;
    const uint8_t lo = sectors[i] & 0xFF;
    payload[payload_len++] = hi;
    payload[payload_len++] = lo;
    checksum ^= hi;
    checksum ^= lo;
  }
  payload[payload_len++] = checksum;

  ctx_.uart->WriteBytes({payload.data(), payload_len});
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  const uint32_t erase_timeout_ms = 10000;
  const auto final_ack = ctx_.uart->ReadByte(erase_timeout_ms);
  if (final_ack != kAck) {
    ESP_LOGE(kTag, "EXT_ERASE failed to get final ACK (0x%02X)",
             final_ack.value_or(0));
    return false;
  }

  ESP_LOGI(kTag, "EXT_ERASE Success");
  return true;
}

bool Programmer::Boot() {
  ESP_LOGI(kTag, "Performing Hardware Reset to Boot App...");

  // Ensure BOOT0 is low (User Flash mode)
  Boot0Set(false);

  // Toggle Reset Pin
  NrstPulse(ctx_.cfg.reset_pulse_ms);

  // Restore application baud rate.
  ctx_.uart->SetBaudRate(ctx_.restore_baud_rate);

  return true;
}

bool Programmer::WriteStm32Block(uint32_t addr, const uint8_t *bytes,
                                 size_t len) {
  if (!bytes || len == 0 || len > kStm32BlockBytes) {
    return false;
  }

  // CMD_WRITE_MEMORY: 0x31 0xCE
  ctx_.uart->Flush();
  ctx_.uart->WriteBytes(kCmdWriteMemory);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  const auto initial_ack = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (initial_ack != kAck) {
    ESP_LOGE(kTag, "WRITE_MEM failed to get initial ACK (0x%02X)",
             initial_ack.value_or(0));
    return false;
  }

  // Address: 4 bytes (BE) + Checksum
  const auto addr_buf = Be32WithXorChecksum(addr);
  ctx_.uart->WriteBytes(addr_buf);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  const auto addr_ack = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (addr_ack != kAck) {
    ESP_LOGE(kTag, "WRITE_MEM failed to get addr ACK (0x%02X)",
             addr_ack.value_or(0));
    return false;
  }

  // Data: N (len-1), Data bytes, Checksum (N ^ bytes[0] ^ ... ^ bytes[len-1])
  uint8_t n = (uint8_t)(len - 1);
  uint8_t cs = n;
  for (size_t i = 0; i < len; ++i) {
    cs ^= bytes[i];
  }

  ctx_.uart->WriteByte(n);
  ctx_.uart->WriteBytes({bytes, len});
  ctx_.uart->WriteByte(cs);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  const auto data_ack = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (data_ack != kAck) {
    ESP_LOGE(kTag, "WRITE_MEM failed to get data ACK (0x%02X)",
             data_ack.value_or(0));
    return false;
  }

  return true;
}

bool Programmer::ReadStm32Block(uint32_t addr, uint8_t *bytes, size_t len) {
  if (!bytes || len == 0 || len > kStm32BlockBytes) {
    return false;
  }

  // CMD_READ_MEMORY: 0x11 0xEE
  ctx_.uart->Flush();
  ctx_.uart->WriteBytes(kCmdReadMemory);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  const auto initial_ack = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (initial_ack != kAck) {
    ESP_LOGE(kTag, "READ_MEM failed to get initial ACK (0x%02X)",
             initial_ack.value_or(0));
    return false;
  }

  // Address: 4 bytes (BE) + Checksum
  const auto addr_buf = Be32WithXorChecksum(addr);
  ctx_.uart->WriteBytes(addr_buf);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  const auto addr_ack = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (addr_ack != kAck) {
    ESP_LOGE(kTag, "READ_MEM failed to get addr ACK (0x%02X)",
             addr_ack.value_or(0));
    return false;
  }

  // Number of bytes to read: N (len-1) + Checksum (complement of N)
  uint8_t n = (uint8_t)(len - 1);
  uint8_t cs = ~n;

  uint8_t len_buf[2] = {n, cs};
  ctx_.uart->WriteBytes(len_buf);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  const auto len_ack = ctx_.uart->ReadByte(ctx_.cfg.sync_timeout_ms);
  if (len_ack != kAck) {
    ESP_LOGE(kTag, "READ_MEM failed to get length ACK (0x%02X)",
             len_ack.value_or(0));
    return false;
  }

  // Receive bytes
  int r = ctx_.uart->ReadBytes(
      {bytes, len},
      ctx_.cfg.sync_timeout_ms + (len / 10));  // extra time for bytes
  if (r != (int)len) {
    ESP_LOGE(kTag, "READ_MEM read data failed exp=%u got=%d", (unsigned)len, r);
    return false;
  }

  return true;
}

void Programmer::Start(uint32_t total_size) {
  // Reset session
  ctx_.total_size = total_size;
  ctx_.written = 0;
  ctx_.verify_offset = 0;
  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;
  ctx_.err = static_cast<uint32_t>(ErrorCode::Common::kOk);
  ctx_.ready = false;
  ctx_.ota_handle = 0;
  ctx_.ota_part = nullptr;

  ESP_LOGI(kTag, "Start target=%s size=%u",
           (ctx_.target == Target::kEsp32) ? "esp32" : "stm32",
           (unsigned)total_size);

  if (!BeginTargetSession(ctx_)) {
    return;
  }

  sm_.Start(StWriting_);
}

void Programmer::Poll(SmTick now) { sm_.Step(now); }

void Programmer::Abort(SmTick) {
  // Disable bootloader entry and clear buffers
  Boot0Set(false);

  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;
  ctx_.err = static_cast<uint32_t>(ErrorCode::Common::kOk);
  ctx_.ready = false;
  ctx_.total_size = 0;
  ctx_.written = 0;

  sm_.Start(StIdle_);

  // Restore application baud rate.
  if (ctx_.uart) {
    ctx_.uart->SetBaudRate(ctx_.restore_baud_rate);
  }
}

size_t Programmer::PushBytes(std::span<const uint8_t> bytes, SmTick now) {
  if (!ctx_.ready) return 0;  // not ready to accept bytes
  if (Error() || Done()) return 0;

  const size_t free = RbFree(ctx_.head, ctx_.tail, Ctx::kBufCap);
  const size_t take = (bytes.size() <= free) ? bytes.size() : free;

  if (take > 0) {
    const size_t until_wrap = std::min(take, Ctx::kBufCap - ctx_.tail);
    std::memcpy(ctx_.buf + ctx_.tail, bytes.data(), until_wrap);
    std::memcpy(ctx_.buf, bytes.data() + until_wrap, take - until_wrap);
    ctx_.tail = (ctx_.tail + take) % Ctx::kBufCap;
  }

  if (take < bytes.size()) {
    ctx_.overflow = true;
  }

  // Advance internal SM even when the span is empty (lets it finish draining /
  // finalize)
  sm_.Step(now);
  return take;
}

bool Programmer::Ready() const { return ctx_.ready && !Error(); }

bool Programmer::Done() const { return sm_.CurrentState() == &StDone_; }

bool Programmer::Error() const { return sm_.CurrentState() == &StError_; }

uint32_t Programmer::LastErrorCode() const {
  if (ctx_.err == static_cast<uint32_t>(ErrorCode::Common::kOk)) {
    return static_cast<uint32_t>(ErrorCode::Common::kUnknown);
  }
  return ctx_.err;
}

bool Programmer::IsVerifying() const {
  return sm_.CurrentState() == &StVerifying_;
}

uint32_t Programmer::Total() const { return ctx_.total_size; }
uint32_t Programmer::Written() const { return ctx_.written; }
uint32_t Programmer::VerifyOffset() const { return ctx_.verify_offset; }

size_t Programmer::Free() const {
  return RbFree(ctx_.head, ctx_.tail, Ctx::kBufCap);
}

bool Programmer::BeginTargetSession(Ctx &c) {
  return (c.target == Target::kEsp32) ? BeginEsp32Ota(c) : BeginStm32Session(c);
}

size_t Programmer::TargetWriteChunkLimit(const Ctx &c) const {
  return (c.target == Target::kEsp32)
             ? Ctx::kWriteChunkCap
             : kStm32BlockBytes;
}

bool Programmer::WriteTargetChunk(Ctx &c, const uint8_t *bytes, size_t len) {
  if (c.target == Target::kEsp32) {
    return WriteEsp32Chunk(c, bytes, len);
  }

  const auto placement =
      Stm32FlashLayout::ResolveOffset(c.written, c.total_size);
  if (!placement) {
    ESP_LOGE(kTag, "STM32 write offset 0x%08X is outside flash image",
             (unsigned)c.written);
    return false;
  }

  if (len > placement->max_chunk) {
    ESP_LOGE(kTag, "Write chunk crossed STM32 image boundary");
    return false;
  }

  if (!WriteStm32Block(placement->flash_addr, bytes, len)) {
    ESP_LOGE(kTag, "Write failed at addr 0x%08X",
             (unsigned)placement->flash_addr);
    return false;
  }

  return true;
}

bool Programmer::FinalizeTargetWrite(Ctx &c) {
  return (c.target == Target::kEsp32) ? FinalizeEsp32Ota(c) : true;
}

size_t Programmer::TargetVerifyChunkSize(const Ctx &c) const {
  return (c.target == Target::kEsp32)
             ? c.cfg.verify.esp32_chunk_bytes
             : kStm32BlockBytes;
}

std::optional<size_t> Programmer::ReadTargetVerifyChunk(
    Ctx &c, std::span<uint8_t> dst) {
  if (c.target == Target::kEsp32) {
    if (!ReadEsp32PartitionBlock(c, c.verify_offset, dst.data(), dst.size())) {
      return std::nullopt;
    }
    return dst.size();
  }

  const auto placement =
      Stm32FlashLayout::ResolveOffset(c.verify_offset, c.total_size);
  if (!placement) {
    ESP_LOGE(kTag, "STM32 verify offset 0x%08X is outside flash image",
             (unsigned)c.verify_offset);
    return std::nullopt;
  }

  const size_t n =
      (dst.size() < placement->max_chunk) ? dst.size() : placement->max_chunk;
  if (!ReadStm32Block(placement->flash_addr, dst.data(), n)) {
    return std::nullopt;
  }

  return n;
}

// Reached only by the STM32: an ESP32 image reboots out of
// CompleteSuccessfulProgram before this state exists. Both branches of
// verify.EnabledFor() funnel here, so one tone covers write-only and
// write-then-verify without either path knowing which ran.
void Programmer::DoneState::OnEnter(Ctx &) {
  Sys().TonePlayer().PlayBuiltin(message::Tone::kConfirm);
}

void Programmer::Fail(Ctx &c, ErrorCode::Esp32 code) {
  c.err = static_cast<uint32_t>(code);
  sm_.ReqTransition(StError_);
}

bool Programmer::CompleteSuccessfulProgram(Ctx &c) {
  if (c.target == Target::kEsp32) {
    if (!ActivateEsp32Ota(c)) {
      return false;
    }

    ESP_LOGI(kTag, "ESP32 OTA Successful. Rebooting...");
    Boot();
    esp_restart();
    return true;
  }

  c.sm->ReqTransition(*c.st_done);
  return true;
}

bool Programmer::BeginEsp32Ota(Ctx &c) {
  ESP_LOGI(kTag, "Starting ESP32 OTA...");
  c.ota_part = esp_ota_get_next_update_partition(NULL);
  if (!c.ota_part) {
    ESP_LOGE(kTag, "OTA partition not found!");
    c.err = static_cast<uint32_t>(
        ErrorCode::Esp32::kProgrammerOtaPartitionNotFound);
    sm_.Start(StError_);
    return false;
  }

  const esp_err_t err = esp_ota_begin(c.ota_part, c.total_size, &c.ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_ota_begin failed: %s", esp_err_to_name(err));
    c.err = static_cast<uint32_t>(ErrorCode::Esp32::kProgrammerOtaBeginFailed);
    sm_.Start(StError_);
    return false;
  }

  ESP_LOGI(kTag, "ESP32 OTA Initialized. Writing to partition subtype %d...",
           c.ota_part->subtype);
  c.ready = true;
  mbedtls_sha256_init(&c.sha_ctx);
  mbedtls_sha256_starts(&c.sha_ctx, 0);
  return true;
}

bool Programmer::WriteEsp32Chunk(Ctx &c, const uint8_t *bytes, size_t len) {
  const esp_err_t err = esp_ota_write(c.ota_handle, bytes, len);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_ota_write failed: %s", esp_err_to_name(err));
    c.err = static_cast<uint32_t>(ErrorCode::Esp32::kProgrammerOtaWriteFailed);
    return false;
  }
  return true;
}

bool Programmer::BeginStm32Session(Ctx &c) {
  if (!EnterStm32Bootloader()) {
    c.err = static_cast<uint32_t>(ErrorCode::Esp32::kProgrammerHandshakeFailed);
    c.ready = false;
    sm_.Start(StError_);
    return false;
  }

  c.ready = true;
  mbedtls_sha256_init(&c.sha_ctx);
  mbedtls_sha256_starts(&c.sha_ctx, 0);

  if (!GetStm32BootloaderInfo()) {
    ESP_LOGW(kTag, "Failed to get bootloader info, proceeding anyway...");
  }

  if (!EraseStm32Sectors()) {
    ESP_LOGE(kTag, "Failed to erase STM32 sectors!");
    c.err = static_cast<uint32_t>(ErrorCode::Esp32::kProgrammerEraseFailed);
    c.ready = false;
    sm_.Start(StError_);
    return false;
  }

  return true;
}

bool Programmer::FinalizeEsp32Ota(Ctx &c) {
  esp_err_t err = esp_ota_end(c.ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_ota_end failed: %s", esp_err_to_name(err));
    Fail(c, ErrorCode::Esp32::kProgrammerOtaEndFailed);
    return false;
  }
  c.ota_handle = 0;
  return true;
}

bool Programmer::ActivateEsp32Ota(Ctx &c) {
  esp_err_t err = esp_ota_set_boot_partition(c.ota_part);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_ota_set_boot_partition failed: %s",
             esp_err_to_name(err));
    Fail(c, ErrorCode::Esp32::kProgrammerOtaSetBootFailed);
    return false;
  }
  return true;
}

bool Programmer::ReadEsp32PartitionBlock(const Ctx &c, uint32_t offset,
                                         uint8_t *bytes, size_t len) const {
  if (c.ota_part == nullptr || bytes == nullptr || len == 0) {
    return false;
  }

  esp_err_t err = esp_partition_read(c.ota_part, offset, bytes, len);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_partition_read failed at offset 0x%08X: %s",
             (unsigned)offset, esp_err_to_name(err));
    return false;
  }
  return true;
}

// State implementations

void Programmer::WritingState::OnStep(Ctx &c, SmTick) {
  Programmer &self = Programmer::GetInstance();

  if (c.overflow) {
    self.Fail(c, ErrorCode::Esp32::kProgrammerBufferOverflow);
    return;
  }

  while (c.written < c.total_size) {
    const size_t remaining_file = c.total_size - c.written;
    const size_t needed =
        std::min(self.TargetWriteChunkLimit(c), remaining_file);

    if (RbUsed(c.head, c.tail, Ctx::kBufCap) < needed) {
      return;  // Wait for more bytes
    }

    const size_t until_wrap = std::min(needed, Ctx::kBufCap - c.head);
    std::memcpy(c.block, c.buf + c.head, until_wrap);
    std::memcpy(c.block + until_wrap, c.buf, needed - until_wrap);
    c.head = (c.head + needed) % Ctx::kBufCap;

    if (!self.WriteTargetChunk(c, c.block, needed)) {
      self.Fail(c, (c.target == Target::kEsp32)
                       ? ErrorCode::Esp32::kProgrammerOtaWriteFailed
                       : ErrorCode::Esp32::kProgrammerWriteFailed);
      return;
    }

    // Update ongoing SHA256
    if (c.target == Target::kEsp32 ||
        Stm32FlashLayout::ContainsOffset(c.written, c.total_size)) {
      mbedtls_sha256_update(&c.sha_ctx, c.block, needed);
    }
    c.written += needed;
  }

  mbedtls_sha256_finish(&c.sha_ctx, c.computed_hash);
  mbedtls_sha256_free(&c.sha_ctx);

  ESP_LOGI(kTag, "Write complete.");

  if (!self.FinalizeTargetWrite(c)) {
    return;
  }

  if (c.cfg.verify.EnabledFor(c.target)) {
    ESP_LOGI(kTag, "Verifying...");
    c.sm->ReqTransition(*c.st_verifying);
    return;
  }

  (void)self.CompleteSuccessfulProgram(c);
}

void Programmer::VerifyingState::OnEnter(Ctx &c) {
  c.verify_offset = 0;
  // Init SHA256 for readback
  mbedtls_sha256_init(&c.sha_ctx);
  mbedtls_sha256_starts(&c.sha_ctx, 0);
}

void Programmer::VerifyingState::OnStep(Ctx &c, SmTick) {
  Programmer &self = Programmer::GetInstance();

  if (c.verify_offset < c.total_size) {
    // Reuse the upload staging buffer as verify scratch after writing
    // completes.
    const size_t chunk =
        std::min({self.TargetVerifyChunkSize(c), Ctx::kBufCap,
                  static_cast<size_t>(c.total_size - c.verify_offset)});

    const auto read = self.ReadTargetVerifyChunk(c, {c.buf, chunk});
    if (!read) {
      self.Fail(c, ErrorCode::Esp32::kProgrammerReadFailed);
      return;
    }

    mbedtls_sha256_update(&c.sha_ctx, c.buf, *read);
    c.verify_offset += *read;
    return;
  }

  // Done reading
  uint8_t read_hash[32];
  mbedtls_sha256_finish(&c.sha_ctx, read_hash);
  mbedtls_sha256_free(&c.sha_ctx);

  // Compare
  if (std::memcmp(c.computed_hash, read_hash, 32) != 0) {
    ESP_LOGE(kTag, "Verification Failed! CRCs do not match.");
    self.Fail(c, ErrorCode::Esp32::kProgrammerVerifyFailed);
  } else {
    ESP_LOGI(kTag, "Verification Successful. Hash matches.");
    if (!self.CompleteSuccessfulProgram(c)) {
      return;
    }
  }
}
