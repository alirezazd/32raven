#include "programmer.hpp"

#include <cstring>

#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "driver/gpio.h"
#include "esp_log.h"
}

static constexpr char kTag[] = "programmer";

namespace {

constexpr size_t kEsp32OtaWriteChunkBytes = 4096;

struct FlashSector {
  uint16_t number;
  uint32_t address;
  uint32_t size;
};

struct Stm32FlashLayout {
  static constexpr uint32_t kBase = 0x08000000u;
  static constexpr uint32_t kVectorsSize = 16u * 1024u;  // sector 0
  static constexpr uint32_t kAppOffset = 0x0000C000u;    // sector 3

  static constexpr FlashSector kSectors[] = {
      {0, 0x08000000u, 16u * 1024u},   {1, 0x08004000u, 16u * 1024u},
      {2, 0x08008000u, 16u * 1024u},   {3, 0x0800C000u, 16u * 1024u},
      {4, 0x08010000u, 64u * 1024u},   {5, 0x08020000u, 128u * 1024u},
      {6, 0x08040000u, 128u * 1024u},  {7, 0x08060000u, 128u * 1024u},
      {8, 0x08080000u, 128u * 1024u},  {9, 0x080A0000u, 128u * 1024u},
      {10, 0x080C0000u, 128u * 1024u}, {11, 0x080E0000u, 128u * 1024u},
  };

  static bool ResolveOffset(uint32_t offset, uint32_t total_size,
                            uint32_t *flash_addr, size_t *max_chunk) {
    if (offset >= total_size || !flash_addr || !max_chunk) {
      return false;
    }

    if (offset < kVectorsSize) {
      *flash_addr = kBase + offset;
      *max_chunk = kVectorsSize - offset;
      return true;
    }

    if (offset < kAppOffset) {
      return false;
    }

    *flash_addr = kBase + offset;
    *max_chunk = total_size - offset;
    return true;
  }

  static bool ContainsOffset(uint32_t offset, uint32_t total_size) {
    uint32_t flash_addr = 0;
    size_t max_chunk = 0;
    return ResolveOffset(offset, total_size, &flash_addr, &max_chunk);
  }
};

}  // namespace

void Programmer::Init(const Config &cfg, UartFcLink *uart) {
  ctx_.cfg = cfg;
  ctx_.uart = uart;
  ctx_.sm = &sm_;

  if (!ctx_.uart) {
    Panic(ErrorCode::kProgrammerUartNull);
  }
  ctx_.restore_baud_rate = ctx_.uart->GetConfig().line.baud_rate;

  // Init pins
  // Bind state pointers
  ctx_.st_idle = &StIdle_;
  ctx_.st_writing = &StWriting_;
  ctx_.st_verifying = &StVerifying_;
  ctx_.st_done = &StDone_;
  ctx_.st_error = &StError_;

  GpioInit();

  // Start in idle
  ctx_.ready = false;
  ctx_.err = ErrorCode::kOk;
  ctx_.total_size = 0;
  ctx_.written = 0;
  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;

  // Ensure STM32 is reset on ESP32 boot
  NrstPulse(ctx_.cfg.reset_pulse_ms);

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

    // Default NRST deasserted (high if active-low reset)
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
  if (!ctx_.uart) return false;

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
  uint8_t tx = 0x7F;
  uint8_t rx = 0;

  uint8_t retries = ctx_.cfg.sync_retries;
  while (retries--) {
    // Send sync
    int w = ctx_.uart->Write(&tx, 1);
    (void)w;

    // Ensure byte physically left UART
    (void)ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

    // Wait for ACK. Some boards can leave a stale byte in the RX FIFO during
    // the reset-to-bootloader transition; keep reading until timeout so one
    // stray byte does not make us miss the actual ACK.
    const TimeMs deadline = TimeAfter(Sys().Timebase().NowMs(),
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

      int r = ctx_.uart->Read(&rx, 1, remaining);
      if (r != 1) {
        continue;
      }

      if (rx == 0x79) {
        if (unexpected_count > 0) {
          ESP_LOGW(kTag,
                   "STM32 Connect ignored %u unexpected byte(s), last=0x%02X",
                   static_cast<unsigned>(unexpected_count), last_unexpected);
        }
        ESP_LOGI(kTag, "STM32 Connect ACK (0x79)");
        return true;  // ACK
      }

      if (rx == 0x1F) {
        saw_nack = true;
        break;
      }

      last_unexpected = rx;
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
  if (!ctx_.uart) return false;

  // CMD_GET: 0x00 0xFF
  uint8_t cmd[] = {0x00, 0xFF};
  ctx_.uart->Flush();
  ctx_.uart->Write(cmd, sizeof(cmd));
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  // Expect: ACK + N + Version + N bytes + ACK
  // N = number of bytes to follow - 1
  uint8_t ack = 0;
  if (ctx_.uart->Read(&ack, 1, ctx_.cfg.sync_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "CMD_GET failed to get initial ACK (0x%02X)", ack);
    return false;
  }

  uint8_t len = 0;
  if (ctx_.uart->Read(&len, 1, ctx_.cfg.sync_timeout_ms) != 1) {
    ESP_LOGE(kTag, "CMD_GET failed to get length");
    return false;
  }

  uint8_t ver = 0;
  if (ctx_.uart->Read(&ver, 1, ctx_.cfg.sync_timeout_ms) != 1) {
    ESP_LOGE(kTag, "CMD_GET failed to get version");
    return false;
  }

  ESP_LOGI(kTag, "STM32 Bootloader v%X.%X", ver >> 4, ver & 0xF);

  // Read supported commands
  // We need to read 'len' bytes. Since 'ver' was one of the bytes (protocol
  // says N = number of bytes - 1) Wait, AN3155 says: ACK N = number of bytes -
  // 1 Version CMD1 CMD2
  // ...
  // CMDn
  // ACK
  // Total bytes to read after N is N + 1. Version is the first one.
  // So we have N more bytes to read after version.

  uint8_t cmds[256];
  // uint8_t len cannot exceed 255, so it fits in cmds

  if (len > 0) {
    if (ctx_.uart->Read(cmds, len, ctx_.cfg.sync_timeout_ms) != (int)len) {
      ESP_LOGE(kTag, "CMD_GET failed to get commands");
      return false;
    }

    // Verify critical commands: READ(0x11), WRITE(0x31), ERASE(0x43/44),
    // GO(0x21)
    bool has_read = false;
    bool has_write = false;
    bool has_erase = false;
    bool has_go = false;

    for (int i = 0; i < len; ++i) {
      switch (cmds[i]) {
        case 0x11:
          has_read = true;
          break;
        case 0x31:
          has_write = true;
          break;
        case 0x43:
        case 0x44:
          has_erase = true;
          break;
        case 0x21:
          has_go = true;
          break;
      }
    }

    if (!has_read || !has_write || !has_erase || !has_go) {
      ESP_LOGE(kTag, "Missing critical commands: R=%d W=%d E=%d GO=%d",
               has_read, has_write, has_erase, has_go);
      return false;
    }
    ESP_LOGI(kTag, "Bootloader capabilities OK");
  }

  uint8_t final_ack = 0;
  if (ctx_.uart->Read(&final_ack, 1, ctx_.cfg.sync_timeout_ms) != 1 ||
      final_ack != 0x79) {
    ESP_LOGE(kTag, "CMD_GET missing final ACK");
    return false;
  }

  return true;
}

bool Programmer::EraseStm32Sectors() {
  if (!ctx_.uart) return false;

  uint16_t sectors[10];
  size_t sector_count = 0;

  sectors[sector_count++] = 0;  // vectors

  if (ctx_.total_size > Stm32FlashLayout::kAppOffset) {
    uint32_t remaining = ctx_.total_size - Stm32FlashLayout::kAppOffset;
    for (const auto &sector : Stm32FlashLayout::kSectors) {
      if (sector.number < 3) {
        continue;
      }
      sectors[sector_count++] = sector.number;
      if (remaining <= sector.size) {
        break;
      }
      remaining -= sector.size;
    }
  }

  ESP_LOGI(kTag, "Sending EXT_ERASE (0x44) for %u sector(s)...",
           (unsigned)sector_count);

  uint8_t cmd[] = {0x44, 0xBB};
  ctx_.uart->Flush();
  ctx_.uart->Write(cmd, sizeof(cmd));
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  uint8_t ack = 0;
  if (ctx_.uart->Read(&ack, 1, ctx_.cfg.sync_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "EXT_ERASE failed to get initial ACK (0x%02X)", ack);
    return false;
  }

  const uint16_t count_minus_one = static_cast<uint16_t>(sector_count - 1u);
  uint8_t payload[2 + sizeof(sectors) + 1] = {};
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

  ctx_.uart->Write(payload, payload_len);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  const uint32_t erase_timeout_ms = 10000;
  ack = 0;
  if (ctx_.uart->Read(&ack, 1, erase_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "EXT_ERASE failed to get final ACK (0x%02X)", ack);
    return false;
  }

  ESP_LOGI(kTag, "EXT_ERASE Success (sectors 0 and 3+ only)");
  return true;
}

bool Programmer::MassEraseStm32() {
  if (!ctx_.uart) return false;

  ESP_LOGI(kTag, "Sending EXT_ERASE (0x44)...");

  // CMD_EXT_ERASE: 0x44 0xBB
  uint8_t cmd[] = {0x44, 0xBB};
  ctx_.uart->Flush();
  ctx_.uart->Write(cmd, sizeof(cmd));
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  uint8_t ack = 0;
  if (ctx_.uart->Read(&ack, 1, ctx_.cfg.sync_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "EXT_ERASE failed to get initial ACK (0x%02X)", ack);
    return false;
  }

  // To mass erase: 0xFF 0xFF + Checksum (0x00)
  // Wait, AN3155 says:
  // 1. Send 0x44 0xBB -> Receive ACK
  // 2. Send 0xFF 0xFF (Special erase code) + 0x00 (Checksum of 0xFF 0xFF xor
  // logic? No, simple sum checksum) Checksum calculation: 0xFF ^ 0xFF = 0x00.
  // Correct.

  uint8_t payload[] = {0xFF, 0xFF, 0x00};
  ctx_.uart->Write(payload, sizeof(payload));
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  // Wait for final ACK - this can take time for full chip erase!
  // Configured wait time might need to be longer.
  // Using a longer timeout here.
  const uint32_t erase_timeout_ms = 10000;

  ack = 0;
  if (ctx_.uart->Read(&ack, 1, erase_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "EXT_ERASE failed to get final ACK (0x%02X)", ack);
    return false;
  }

  ESP_LOGI(kTag, "EXT_ERASE Success");
  return true;
}

bool Programmer::Boot() {
  if (!ctx_.uart) return false;

  ESP_LOGI(kTag, "Performing Hardware Reset to Boot App...");

  // Ensure BOOT0 is low (User Flash mode)
  Boot0Set(false);

  // Toggle Reset Pin
  NrstPulse(ctx_.cfg.reset_pulse_ms);

  // Restore application baud rate.
  ctx_.uart->SetBaudRate(ctx_.restore_baud_rate);

  return true;
}

bool Programmer::WriteStm32Block(uint32_t addr, const uint8_t *data,
                                 size_t len) {
  if (!ctx_.uart || !data || len == 0 ||
      len > esp32_limits::kProgrammerStm32BlockBytes) {
    return false;
  }

  // CMD_WRITE_MEMORY: 0x31 0xCE
  uint8_t cmd[] = {0x31, 0xCE};
  ctx_.uart->Flush();
  ctx_.uart->Write(cmd, sizeof(cmd));
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  uint8_t ack = 0;
  if (ctx_.uart->Read(&ack, 1, ctx_.cfg.sync_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "WRITE_MEM failed to get initial ACK (0x%02X)", ack);
    return false;
  }

  // Address: 4 bytes (BE) + Checksum
  uint8_t addr_buf[5];
  addr_buf[0] = (addr >> 24) & 0xFF;
  addr_buf[1] = (addr >> 16) & 0xFF;
  addr_buf[2] = (addr >> 8) & 0xFF;
  addr_buf[3] = (addr >> 0) & 0xFF;
  addr_buf[4] = addr_buf[0] ^ addr_buf[1] ^ addr_buf[2] ^ addr_buf[3];

  ctx_.uart->Write(addr_buf, 5);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  if (ctx_.uart->Read(&ack, 1, ctx_.cfg.sync_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "WRITE_MEM failed to get addr ACK (0x%02X)", ack);
    return false;
  }

  // Data: N (len-1), Data bytes, Checksum (N ^ data[0] ^ ... ^ data[len-1])
  uint8_t n = (uint8_t)(len - 1);
  uint8_t cs = n;
  for (size_t i = 0; i < len; ++i) {
    cs ^= data[i];
  }

  ctx_.uart->Write(&n, 1);
  ctx_.uart->Write(data, len);
  ctx_.uart->Write(&cs, 1);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  if (ctx_.uart->Read(&ack, 1, ctx_.cfg.sync_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "WRITE_MEM failed to get data ACK (0x%02X)", ack);
    return false;
  }

  return true;
}

bool Programmer::ReadStm32Block(uint32_t addr, uint8_t *data, size_t len) {
  if (!ctx_.uart || !data || len == 0 ||
      len > esp32_limits::kProgrammerStm32BlockBytes) {
    return false;
  }

  // CMD_READ_MEMORY: 0x11 0xEE
  uint8_t cmd[] = {0x11, 0xEE};
  ctx_.uart->Flush();
  ctx_.uart->Write(cmd, sizeof(cmd));
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  uint8_t ack = 0;
  if (ctx_.uart->Read(&ack, 1, ctx_.cfg.sync_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "READ_MEM failed to get initial ACK (0x%02X)", ack);
    return false;
  }

  // Address: 4 bytes (BE) + Checksum
  uint8_t addr_buf[5];
  addr_buf[0] = (addr >> 24) & 0xFF;
  addr_buf[1] = (addr >> 16) & 0xFF;
  addr_buf[2] = (addr >> 8) & 0xFF;
  addr_buf[3] = (addr >> 0) & 0xFF;
  addr_buf[4] = addr_buf[0] ^ addr_buf[1] ^ addr_buf[2] ^ addr_buf[3];

  ctx_.uart->Write(addr_buf, 5);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  if (ctx_.uart->Read(&ack, 1, ctx_.cfg.sync_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "READ_MEM failed to get addr ACK (0x%02X)", ack);
    return false;
  }

  // Number of bytes to read: N (len-1) + Checksum (complement of N)
  uint8_t n = (uint8_t)(len - 1);
  uint8_t cs = ~n;

  uint8_t len_buf[2] = {n, cs};
  ctx_.uart->Write(len_buf, 2);
  ctx_.uart->DrainTx(ctx_.cfg.sync_timeout_ms);

  if (ctx_.uart->Read(&ack, 1, ctx_.cfg.sync_timeout_ms) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "READ_MEM failed to get length ACK (0x%02X)", ack);
    return false;
  }

  // Receive data
  int r = ctx_.uart->Read(
      data, len,
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
  ctx_.err = ErrorCode::kOk;
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

void Programmer::Abort(SmTick now) {
  (void)now;
  // Disable bootloader entry and clear buffers
  Boot0Set(false);

  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;
  ctx_.err = ErrorCode::kOk;
  ctx_.ready = false;
  ctx_.total_size = 0;
  ctx_.written = 0;

  sm_.Start(StIdle_);

  // Restore application baud rate.
  if (ctx_.uart) {
    ctx_.uart->SetBaudRate(ctx_.restore_baud_rate);
  }
}

size_t Programmer::PushBytes(const uint8_t *data, size_t n, SmTick now) {
  if (!ctx_.ready) return 0;  // not ready to accept bytes
  if (Error() || Done()) return 0;

  if (data && n) {
    const size_t free = RbFree(ctx_.head, ctx_.tail, Ctx::kBufCap);
    size_t take = (n <= free) ? n : free;

    for (size_t i = 0; i < take; ++i) {
      ctx_.buf[ctx_.tail] = data[i];
      ctx_.tail = (ctx_.tail + 1) % Ctx::kBufCap;
    }

    if (take < n) {
      ctx_.overflow = true;
    }
  }

  // Advance internal SM even if n==0 (lets it finish draining / finalize)
  sm_.Step(now);
  return n ? (n <= RbFree(ctx_.head, ctx_.tail, Ctx::kBufCap) ? n : 0) : 0;
}

bool Programmer::Ready() const { return ctx_.ready && !Error(); }

bool Programmer::Done() const {
  return (sm_.CurrentName() && std::strcmp(sm_.CurrentName(), "P.Done") == 0);
}

bool Programmer::Error() const {
  return (sm_.CurrentName() && std::strcmp(sm_.CurrentName(), "P.Error") == 0);
}

ErrorCode Programmer::LastErrorCode() const {
  return (ctx_.err == ErrorCode::kOk) ? ErrorCode::kUnknown : ctx_.err;
}

bool Programmer::IsVerifying() const {
  return (sm_.CurrentName() &&
          std::strcmp(sm_.CurrentName(), "P.Verifying") == 0);
}

uint32_t Programmer::Total() const { return ctx_.total_size; }
uint32_t Programmer::Written() const { return ctx_.written; }

size_t Programmer::Free() const {
  return RbFree(ctx_.head, ctx_.tail, Ctx::kBufCap);
}

bool Programmer::BeginTargetSession(Ctx &c) {
  return (c.target == Target::kEsp32) ? BeginEsp32Ota(c) : BeginStm32Session(c);
}

size_t Programmer::TargetWriteChunkLimit(const Ctx &c) const {
  return (c.target == Target::kEsp32) ? kEsp32OtaWriteChunkBytes
                                      : esp32_limits::kProgrammerStm32BlockBytes;
}

bool Programmer::WriteTargetChunk(Ctx &c, const uint8_t *data, size_t len) {
  if (c.target == Target::kEsp32) {
    return WriteEsp32Chunk(c, data, len);
  }

  uint32_t flash_addr = 0;
  size_t max_chunk = 0;
  if (!Stm32FlashLayout::ResolveOffset(c.written, c.total_size, &flash_addr,
                                       &max_chunk)) {
    return true;
  }

  if (len > max_chunk) {
    ESP_LOGE(kTag, "Write chunk crossed STM32 image region boundary");
    return false;
  }

  if (!WriteStm32Block(flash_addr, data, len)) {
    ESP_LOGE(kTag, "Write failed at addr 0x%08X", (unsigned)flash_addr);
    return false;
  }

  return true;
}

bool Programmer::FinalizeTargetWrite(Ctx &c) {
  return (c.target == Target::kEsp32) ? FinalizeEsp32Ota(c) : true;
}

size_t Programmer::TargetVerifyChunkSize(const Ctx &c) const {
  return (c.target == Target::kEsp32) ? c.cfg.verify.esp32_chunk_bytes
                                      : esp32_limits::kProgrammerStm32BlockBytes;
}

bool Programmer::ReadTargetVerifyChunk(Ctx &c, uint8_t *data, size_t *len) {
  if (data == nullptr || len == nullptr || *len == 0) {
    return false;
  }

  if (c.target == Target::kEsp32) {
    return ReadEsp32PartitionBlock(c, c.verify_offset, data, *len);
  }

  uint32_t flash_addr = 0;
  size_t max_chunk = 0;
  if (!Stm32FlashLayout::ResolveOffset(c.verify_offset, c.total_size, &flash_addr,
                                       &max_chunk)) {
    c.verify_offset = Stm32FlashLayout::kAppOffset;
    *len = 0;
    return true;
  }

  if (*len > max_chunk) {
    *len = max_chunk;
  }

  return ReadStm32Block(flash_addr, data, *len);
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

  if (c.sm && c.st_done) {
    c.sm->ReqTransition(*c.st_done);
  }
  return true;
}

bool Programmer::BeginEsp32Ota(Ctx &c) {
  ESP_LOGI(kTag, "Starting ESP32 OTA...");
  c.ota_part = esp_ota_get_next_update_partition(NULL);
  if (!c.ota_part) {
    ESP_LOGE(kTag, "OTA partition not found!");
    c.err = ErrorCode::kProgrammerOtaPartitionNotFound;
    sm_.Start(StError_);
    return false;
  }

  const esp_err_t err = esp_ota_begin(c.ota_part, c.total_size, &c.ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_ota_begin failed: %s", esp_err_to_name(err));
    c.err = ErrorCode::kProgrammerOtaBeginFailed;
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

bool Programmer::WriteEsp32Chunk(Ctx &c, const uint8_t *data, size_t len) {
  const esp_err_t err = esp_ota_write(c.ota_handle, data, len);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_ota_write failed: %s", esp_err_to_name(err));
    c.err = ErrorCode::kProgrammerOtaWriteFailed;
    return false;
  }
  return true;
}

bool Programmer::BeginStm32Session(Ctx &c) {
  if (!EnterStm32Bootloader()) {
    c.err = ErrorCode::kProgrammerHandshakeFailed;
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
    c.err = ErrorCode::kProgrammerEraseFailed;
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
    c.err = ErrorCode::kProgrammerOtaEndFailed;
    if (c.sm && c.st_error) {
      c.sm->ReqTransition(*c.st_error);
    }
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
    c.err = ErrorCode::kProgrammerOtaSetBootFailed;
    if (c.sm && c.st_error) {
      c.sm->ReqTransition(*c.st_error);
    }
    return false;
  }
  return true;
}

bool Programmer::ReadEsp32PartitionBlock(const Ctx &c, uint32_t offset,
                                         uint8_t *data, size_t len) const {
  if (c.ota_part == nullptr || data == nullptr || len == 0) {
    return false;
  }

  esp_err_t err = esp_partition_read(c.ota_part, offset, data, len);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_partition_read failed at offset 0x%08X: %s",
             (unsigned)offset, esp_err_to_name(err));
    return false;
  }
  return true;
}

// ---- State implementations ----

void Programmer::WritingState::OnStep(Ctx &c, SmTick now) {
  (void)now;

  // If buffer overflow happened, error out
  if (c.overflow) {
    c.err = ErrorCode::kProgrammerBufferOverflow;
    // Transition to error is handled by the SM check?
    // Actually, we need to request it. Since we can't easily request from here
    // without the parent SM pointer passed down or stored in Ctx differently
    // (Ctx has sm*), we use that.
    if (c.sm && c.st_error) {
      c.sm->ReqTransition(*c.st_error);
    }
    return;
  }

  while (c.written < c.total_size) {
    size_t available = RbUsed(c.head, c.tail, Ctx::kBufCap);

    const size_t protocol_limit = Programmer::GetInstance().TargetWriteChunkLimit(c);

    size_t needed = protocol_limit;
    size_t remaining_file = c.total_size - c.written;
    if (needed > remaining_file) needed = remaining_file;

    if (available < needed) {
      // Wait for more data
      return;
    }

    static uint8_t block[kEsp32OtaWriteChunkBytes];
    for (size_t i = 0; i < needed; ++i) {
      block[i] = c.buf[c.head];
      c.head = (c.head + 1) % Ctx::kBufCap;
    }

    if (!Programmer::GetInstance().WriteTargetChunk(c, block, needed)) {
      c.err = (c.target == Target::kEsp32)
                  ? ErrorCode::kProgrammerOtaWriteFailed
                  : ErrorCode::kProgrammerWriteFailed;
      if (c.sm && c.st_error) c.sm->ReqTransition(*c.st_error);
      return;
    }

    // Update ongoing SHA256
    if (c.target == Target::kEsp32 ||
        Stm32FlashLayout::ContainsOffset(c.written, c.total_size)) {
      mbedtls_sha256_update(&c.sha_ctx, block, needed);
    }
    c.written += needed;
  }

  // Done writing
  if (c.written >= c.total_size) {
    mbedtls_sha256_finish(&c.sha_ctx, c.computed_hash);
    mbedtls_sha256_free(&c.sha_ctx);

    ESP_LOGI(kTag, "Write complete.");

    if (!Programmer::GetInstance().FinalizeTargetWrite(c)) {
      return;
    }

    if (c.cfg.verify.EnabledFor(c.target)) {
      ESP_LOGI(kTag, "Verifying...");
      if (c.sm && c.st_verifying) {
        c.sm->ReqTransition(*c.st_verifying);
      }
      return;
    }

    if (!Programmer::GetInstance().CompleteSuccessfulProgram(c)) {
      return;
    }
  }
}

void Programmer::VerifyingState::OnEnter(Ctx &c) {
  c.verify_offset = 0;
  // Init SHA256 for readback
  mbedtls_sha256_init(&c.sha_ctx);
  mbedtls_sha256_starts(&c.sha_ctx, 0);
}

void Programmer::VerifyingState::OnStep(Ctx &c, SmTick) {
  while (c.verify_offset < c.total_size) {
    // Reuse the upload staging buffer as verify scratch after writing completes.
    size_t chunk = Programmer::GetInstance().TargetVerifyChunkSize(c);
    if (chunk > Ctx::kBufCap) {
      chunk = Ctx::kBufCap;
    }
    if (chunk > (c.total_size - c.verify_offset)) {
      chunk = c.total_size - c.verify_offset;
    }

    if (!Programmer::GetInstance().ReadTargetVerifyChunk(c, c.buf, &chunk)) {
      c.err = ErrorCode::kProgrammerReadFailed;
      if (c.sm && c.st_error) {
        c.sm->ReqTransition(*c.st_error);
      }
      return;
    }

    if (chunk == 0) {
      continue;
    }

    mbedtls_sha256_update(&c.sha_ctx, c.buf, chunk);
    c.verify_offset += chunk;

    return;
  }

  // Done reading
  uint8_t read_hash[32];
  mbedtls_sha256_finish(&c.sha_ctx, read_hash);
  mbedtls_sha256_free(&c.sha_ctx);

  // Compare
  if (std::memcmp(c.computed_hash, read_hash, 32) != 0) {
    ESP_LOGE(kTag, "Verification Failed! CRCs do not match.");
    c.err = ErrorCode::kProgrammerVerifyFailed;
    if (c.sm && c.st_error) c.sm->ReqTransition(*c.st_error);
  } else {
    ESP_LOGI(kTag, "Verification Successful. Hash matches.");
    if (!Programmer::GetInstance().CompleteSuccessfulProgram(c)) {
      return;
    }
  }
}
