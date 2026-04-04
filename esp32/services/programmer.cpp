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
  ctx_.err = 0;
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

bool Programmer::EnterBootloader() {
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

    // Wait for ACK
    int r = ctx_.uart->Read(&rx, 1, ctx_.cfg.sync_timeout_ms);
    if (r == 1) {
      if (rx == 0x79) {
        ESP_LOGI(kTag, "STM32 Connect ACK (0x79)");
        return true;  // ACK
      }
      if (rx == 0x1F) {
        ESP_LOGW(kTag, "STM32 Connect NACK (0x1F)");
      } else {
        ESP_LOGW(kTag, "STM32 Connect Unknown: 0x%02X", rx);
      }
    } else {
      // Timeout or no data, retry
    }

    // small delay between retries
    Sys().Timebase().SleepMs(10);
    ctx_.uart->Flush();
  }

  return false;
}

bool Programmer::GetBootloaderInfo() {
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

bool Programmer::EraseSectors() {
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

bool Programmer::MassErase() {
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

bool Programmer::WriteBlock(uint32_t addr, const uint8_t *data, size_t len) {
  if (!ctx_.uart || !data || len == 0 || len > 256) return false;

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

bool Programmer::ReadBlock(uint32_t addr, uint8_t *data, size_t len) {
  if (!ctx_.uart || !data || len == 0 || len > 256) return false;

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
  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;
  ctx_.err = 0;
  ctx_.ready = false;

  ESP_LOGI(kTag, "Start target=%s size=%u",
           (ctx_.target == Target::kEsp32) ? "esp32" : "stm32",
           (unsigned)total_size);

  if (ctx_.target == Target::kEsp32) {
    ESP_LOGI(kTag, "Starting ESP32 OTA...");
    ctx_.ota_part = esp_ota_get_next_update_partition(NULL);
    if (!ctx_.ota_part) {
      ESP_LOGE(kTag, "OTA partition not found!");
      ctx_.err = 7;  // ota_part_not_found
      sm_.Start(StError_);
      return;
    }

    esp_err_t err = esp_ota_begin(ctx_.ota_part, total_size, &ctx_.ota_handle);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "esp_ota_begin failed: %s", esp_err_to_name(err));
      ctx_.err = 8;  // ota_begin_failed
      sm_.Start(StError_);
      return;
    }

    ESP_LOGI(kTag, "ESP32 OTA Initialized. Writing to partition subtype %d...",
             ctx_.ota_part->subtype);
    ctx_.ready = true;
    // Skip SHA256 init for now, esp_ota verifies image checksum itself?
    // We can still do it for our own sanity/progress matching.
    mbedtls_sha256_init(&ctx_.sha_ctx);
    mbedtls_sha256_starts(&ctx_.sha_ctx, 0);

    sm_.Start(StWriting_);
    return;
  }

  // --- STM32 Logic ---
  // Run blocking handshake now (OK at this stage since upload is disabled)
  if (!EnterBootloader()) {
    ctx_.err = 1;  // handshake_failed
    ctx_.ready = false;
    sm_.Start(StError_);
    return;
  }

  // Handshake OK
  ctx_.ready = true;

  // Init SHA256 for write calculation
  mbedtls_sha256_init(&ctx_.sha_ctx);
  mbedtls_sha256_starts(&ctx_.sha_ctx, 0);  // 0 = SHA256

  if (!GetBootloaderInfo()) {
    ESP_LOGW(kTag, "Failed to get bootloader info, proceeding anyway...");
  }

  if (!EraseSectors()) {
    ESP_LOGE(kTag, "Failed to mass erase!");
    ctx_.err = 3;  // erase_failed
    ctx_.ready = false;
    sm_.Start(StError_);
    return;
  }

  // Start internal writing SM (placeholder until you implement AN3155 write
  // blocks)
  sm_.Start(StWriting_);
}

void Programmer::Poll(SmTick now) { sm_.Step(now); }

void Programmer::Abort(SmTick now) {
  (void)now;
  // Disable bootloader entry and clear buffers
  Boot0Set(false);

  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;
  ctx_.err = 0;
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

bool Programmer::IsVerifying() const {
  return (sm_.CurrentName() &&
          std::strcmp(sm_.CurrentName(), "P.Verifying") == 0);
}

uint32_t Programmer::Total() const { return ctx_.total_size; }
uint32_t Programmer::Written() const { return ctx_.written; }

size_t Programmer::Free() const {
  return RbFree(ctx_.head, ctx_.tail, Ctx::kBufCap);
}

// ---- State implementations ----

void Programmer::WritingState::OnStep(Ctx &c, SmTick now) {
  (void)now;

  // If buffer overflow happened, error out
  if (c.overflow) {
    c.err = 2;  // buffer_overflow
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

    // Determine optimal chunk size based on target capabilities
    // ESP32: Flash sector size (4KB) for efficiency
    // STM32: UART Bootloader protocol limit (256 bytes)
    const size_t protocol_limit = (c.target == Target::kEsp32) ? 4096 : 256;

    size_t needed = protocol_limit;
    size_t remaining_file = c.total_size - c.written;
    if (needed > remaining_file) needed = remaining_file;

    if (available < needed) {
      // Wait for more data
      return;
    }

    // Allocate for the largest possible chunk
    static uint8_t block[4096];
    for (size_t i = 0; i < needed; ++i) {
      block[i] = c.buf[c.head];
      c.head = (c.head + 1) % Ctx::kBufCap;
    }

    if (c.target == Target::kEsp32) {
      // --- ESP32 OTA Write ---
      esp_err_t err = esp_ota_write(c.ota_handle, block, needed);
      if (err != ESP_OK) {
        c.err = 9;  // ota_write_failed
        ESP_LOGE(kTag, "esp_ota_write failed: %s", esp_err_to_name(err));
        if (c.sm && c.st_error) c.sm->ReqTransition(*c.st_error);
        return;
      }
      // Continue to checksum update below
    } else {
      // --- STM32 UART Write ---
      // Write block
      // Note: This blocks the main loop, but each block is fast (~25ms @
      // 115200)
      uint32_t flash_addr = 0;
      size_t max_chunk = 0;
      if (Stm32FlashLayout::ResolveOffset(c.written, c.total_size, &flash_addr,
                                          &max_chunk)) {
        if (needed > max_chunk) {
          c.err = 4;
          ESP_LOGE(kTag, "Write chunk crossed STM32 image region boundary");
          if (c.sm && c.st_error) c.sm->ReqTransition(*c.st_error);
          return;
        }

        if (!Programmer::GetInstance().WriteBlock(flash_addr, block, needed)) {
          c.err = 4;  // write_failed
          ESP_LOGE(kTag, "Write failed at addr 0x%08X", (unsigned)flash_addr);
          if (c.sm && c.st_error) c.sm->ReqTransition(*c.st_error);
          return;
        }
      }
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

    if (c.target == Target::kEsp32) {
      // Finalize ESP32 OTA
      esp_err_t err = esp_ota_end(c.ota_handle);
      if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_ota_end failed: %s", esp_err_to_name(err));
        c.err = 10;  // ota_end_failed
        if (c.sm && c.st_error) c.sm->ReqTransition(*c.st_error);
        return;
      }

      err = esp_ota_set_boot_partition(c.ota_part);
      if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_ota_set_boot_partition failed: %s",
                 esp_err_to_name(err));
        c.err = 11;
        if (c.sm && c.st_error) c.sm->ReqTransition(*c.st_error);
        return;
      }

      ESP_LOGI(kTag, "ESP32 OTA Successful. Rebooting...");
      // Reboot STM32 first
      Programmer::GetInstance().Boot();
      // Delay slightly to ensure log is flushed?
      esp_restart();
      return;
    }

    // Connect to Verification for STM32
    ESP_LOGI(kTag, "Verifying...");
    if (c.sm && c.st_verifying) {
      c.sm->ReqTransition(*c.st_verifying);
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
    uint32_t flash_addr = 0;
    size_t max_chunk = 0;
    if (!Stm32FlashLayout::ResolveOffset(c.verify_offset, c.total_size,
                                         &flash_addr, &max_chunk)) {
      c.verify_offset = Stm32FlashLayout::kAppOffset;
      continue;
    }

    size_t chunk = 256;
    if (chunk > (c.total_size - c.verify_offset)) {
      chunk = c.total_size - c.verify_offset;
    }
    if (chunk > max_chunk) {
      chunk = max_chunk;
    }

    uint8_t buf[256];
    if (!Programmer::GetInstance().ReadBlock(flash_addr, buf, chunk)) {
      c.err = 5;  // verify_read_failed
      if (c.sm && c.st_error) c.sm->ReqTransition(*c.st_error);
      return;
    }

    mbedtls_sha256_update(&c.sha_ctx, buf, chunk);
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
    c.err = 6;  // verify_checksum_mismatch
    if (c.sm && c.st_error) c.sm->ReqTransition(*c.st_error);
  } else {
    ESP_LOGI(kTag, "Verification Successful. Hash matches.");
    if (c.sm && c.st_done) c.sm->ReqTransition(*c.st_done);
  }
}
