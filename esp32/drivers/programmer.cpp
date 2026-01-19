#include "programmer.hpp"
#include "timebase.hpp"

#include <cstring>

extern "C" {
#include "driver/gpio.h"
#include "esp_log.h"
}

static constexpr char kTag[] = "programmer";

void Programmer::Init(const Config &cfg, Uart *uart,
                      ErrorHandler error_handler) {
  if (initialized_)
    return;
  ctx_.cfg = cfg;
  ctx_.uart = uart;
  ctx_.error_handler = error_handler;
  ctx_.sm = &sm_;

  if (!ctx_.uart) {
    if (error_handler)
      error_handler("Programmer Uart is Null");
    return;
  }

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

  sm_.Start(StIdle_, 0);
  initialized_ = true;
}

void Programmer::GpioInit() { // TODO: Add a GPIO driver owner and assign pins
  const int kBoot0 = ctx_.cfg.boot0_gpio;
  const int kNrst = ctx_.cfg.nrst_gpio;

  if (kBoot0 >= 0) {
    gpio_config_t io{};
    io.pin_bit_mask = (1ULL << kBoot0);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // Default BOOT0 low (normal boot)
    Boot0Set(false);
  }

  if (kNrst >= 0) {
    gpio_config_t io{};
    io.pin_bit_mask = (1ULL << kNrst);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // Default NRST deasserted (high if active-low reset)
    // Default NRST deasserted (high for active-low reset)
    gpio_set_level((gpio_num_t)kNrst, 1);
  }
}

void Programmer::Boot0Set(bool on) {
  const int kPin = ctx_.cfg.boot0_gpio;
  if (kPin < 0)
    return;

  // BOOT0 is assumed active-high
  gpio_set_level((gpio_num_t)kPin, on ? 1 : 0);
}

void Programmer::NrstPulse(uint32_t pulse_ms) {
  const int kPin = ctx_.cfg.nrst_gpio;
  if (kPin < 0)
    return;

  // NRST is assumed active-low (assert=0, deassert=1)
  const int kAssertLevel = 0;
  const int kDeassertLevel = 1;

  gpio_set_level((gpio_num_t)kPin, kAssertLevel);
  SleepMs(pulse_ms);
  gpio_set_level((gpio_num_t)kPin, kDeassertLevel);
}

bool Programmer::EnterBootloader() {
  if (!ctx_.uart)
    return false;

  // Put STM32 into ROM bootloader: BOOT0=1, reset pulse, settle
  Boot0Set(true);
  NrstPulse(ctx_.cfg.reset_pulse_ms);
  SleepMs(ctx_.cfg.boot_settle_ms);

  // Configure UART for bootloader link (your Uart driver already exists)
  (void)ctx_.uart->SetBaudRate(ctx_.cfg.baudrate);

  // Flush any junk
  ctx_.uart->Flush();

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
        return true; // ACK
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
    SleepMs(10);
    ctx_.uart->Flush();
  }

  return false;
}

bool Programmer::GetBootloaderInfo() {
  if (!ctx_.uart)
    return false;

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

bool Programmer::EraseAll() {
  if (!ctx_.uart)
    return false;

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
  const uint32_t kEraseTimeoutMs = 10000;

  ack = 0;
  if (ctx_.uart->Read(&ack, 1, kEraseTimeoutMs) != 1 || ack != 0x79) {
    ESP_LOGE(kTag, "EXT_ERASE failed to get final ACK (0x%02X)", ack);
    return false;
  }

  ESP_LOGI(kTag, "EXT_ERASE Success");
  return true;
}

bool Programmer::Boot() {
  if (!ctx_.uart)
    return false;

  ESP_LOGI(kTag, "Performing Hardware Reset to Boot App...");

  // Ensure BOOT0 is low (User Flash mode)
  Boot0Set(false);

  // Toggle Reset Pin
  NrstPulse(ctx_.cfg.reset_pulse_ms);

  return true;
}

bool Programmer::WriteBlock(uint32_t addr, const uint8_t *data, size_t len) {
  if (!ctx_.uart || !data || len == 0 || len > 256)
    return false;

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
  if (!ctx_.uart || !data || len == 0 || len > 256)
    return false;

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
      data, len, ctx_.cfg.sync_timeout_ms + (len / 10)); // extra time for bytes
  if (r != (int)len) {
    ESP_LOGE(kTag, "READ_MEM read data failed exp=%u got=%d", (unsigned)len, r);
    return false;
  }

  return true;
}

void Programmer::Start(uint32_t total_size, SmTick now) {
  (void)now;
  if (!initialized_)
    return;

  // Reset session
  ctx_.total_size = total_size;
  ctx_.written = 0;
  ctx_.write_addr = 0x08000000;
  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;
  ctx_.err = 0;
  ctx_.ready = false;

  if (ctx_.target == Target::kEsp32) {
    ESP_LOGI(kTag, "Starting ESP32 OTA...");
    ctx_.ota_part = esp_ota_get_next_update_partition(NULL);
    if (!ctx_.ota_part) {
      ESP_LOGE(kTag, "OTA partition not found!");
      ctx_.err = 7; // ota_part_not_found
      sm_.Start(StError_, now);
      return;
    }

    esp_err_t err = esp_ota_begin(ctx_.ota_part, total_size, &ctx_.ota_handle);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "esp_ota_begin failed: %s", esp_err_to_name(err));
      ctx_.err = 8; // ota_begin_failed
      sm_.Start(StError_, now);
      return;
    }

    ESP_LOGI(kTag, "ESP32 OTA Initialized. Writing to partition subtype %d...",
             ctx_.ota_part->subtype);
    ctx_.ready = true;
    // Skip SHA256 init for now, esp_ota verifies image checksum itself?
    // We can still do it for our own sanity/progress matching.
    mbedtls_sha256_init(&ctx_.sha_ctx);
    mbedtls_sha256_starts(&ctx_.sha_ctx, 0);

    sm_.Start(StWriting_, now);
    return;
  }

  // --- STM32 Logic ---
  // Run blocking handshake now (OK at this stage since upload is disabled)
  if (!EnterBootloader()) {
    ctx_.err = 1; // handshake_failed
    ctx_.ready = false;
    sm_.Start(StError_, now);
    return;
  }

  // Handshake OK
  ctx_.ready = true;

  // Init SHA256 for write calculation
  mbedtls_sha256_init(&ctx_.sha_ctx);
  mbedtls_sha256_starts(&ctx_.sha_ctx, 0); // 0 = SHA256

  if (!GetBootloaderInfo()) {
    ESP_LOGW(kTag, "Failed to get bootloader info, proceeding anyway...");
  }

  if (!EraseAll()) {
    ESP_LOGE(kTag, "Failed to mass erase!");
    ctx_.err = 3; // erase_failed
    ctx_.ready = false;
    sm_.Start(StError_, now);
    return;
  }

  // Start internal writing SM (placeholder until you implement AN3155 write
  // blocks)
  sm_.Start(StWriting_, now);
}

void Programmer::Poll(SmTick now) {
  if (!initialized_)
    return;
  sm_.Step(now);
}

void Programmer::Abort(SmTick now) {
  (void)now;
  if (!initialized_)
    return;

  // Disable bootloader entry and clear buffers
  Boot0Set(false);

  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;
  ctx_.err = 0;
  ctx_.ready = false;
  ctx_.total_size = 0;
  ctx_.written = 0;

  sm_.Start(StIdle_, now);
}

size_t Programmer::PushBytes(const uint8_t *data, size_t n, SmTick now) {
  if (!initialized_)
    return 0;
  if (!ctx_.ready)
    return 0; // not ready to accept bytes
  if (Error() || Done())
    return 0;

  if (data && n) {
    const size_t kFree = RbFree(ctx_.head, ctx_.tail, Ctx::kBufCap);
    size_t take = (n <= kFree) ? n : kFree;

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

bool Programmer::Ready() const {
  return initialized_ && ctx_.ready && !Error();
}

bool Programmer::Done() const {
  return initialized_ &&
         (sm_.CurrentName() && std::strcmp(sm_.CurrentName(), "P.Done") == 0);
}

bool Programmer::Error() const {
  return initialized_ &&
         (sm_.CurrentName() && std::strcmp(sm_.CurrentName(), "P.Error") == 0);
}

bool Programmer::IsVerifying() const {
  return initialized_ && (sm_.CurrentName() &&
                          std::strcmp(sm_.CurrentName(), "P.Verifying") == 0);
}

uint32_t Programmer::Total() const { return ctx_.total_size; }
uint32_t Programmer::Written() const { return ctx_.written; }

size_t Programmer::Free() const {
  return RbFree(ctx_.head, ctx_.tail, Ctx::kBufCap);
}

// ---- State implementations ----

void Programmer::WritingState::OnEnter(Ctx &c, SmTick) {
  // Placeholder: in real AN3155 implementation, you would set up erase/write
  // parameters here. For now do nothing.
  (void)c;
}

void Programmer::WritingState::OnStep(Ctx &c, SmTick now) {
  (void)now;

  // If buffer overflow happened, error out
  if (c.overflow) {
    c.err = 2; // buffer_overflow
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

    // Default chunk size
    size_t needed = 256;
    size_t remaining_file = c.total_size - c.written;
    if (needed > remaining_file)
      needed = remaining_file;

    if (available < needed) {
      // Wait for more data
      return;
    }

    uint8_t block[256];
    for (size_t i = 0; i < needed; ++i) {
      block[i] = c.buf[c.head];
      c.head = (c.head + 1) % Ctx::kBufCap;
    }

    if (c.target == Target::kEsp32) {
      // --- ESP32 OTA Write ---
      esp_err_t err = esp_ota_write(c.ota_handle, block, needed);
      if (err != ESP_OK) {
        c.err = 9; // ota_write_failed
        ESP_LOGE(kTag, "esp_ota_write failed: %s", esp_err_to_name(err));
        if (c.sm && c.st_error)
          c.sm->ReqTransition(*c.st_error);
        return;
      }
      // Continue to checksum update below
    } else {
      // --- STM32 UART Write ---
      // Write block
      // Note: This blocks the main loop, but each block is fast (~25ms @
      // 115200)
      if (!Programmer::GetInstance().WriteBlock(c.write_addr, block, needed)) {
        c.err = 4; // write_failed
        ESP_LOGE(kTag, "Write failed at addr 0x%08X", (unsigned)c.write_addr);
        if (c.sm && c.st_error)
          c.sm->ReqTransition(*c.st_error);
        return;
      }
      c.write_addr += needed;
    }

    // Update ongoing SHA256
    mbedtls_sha256_update(&c.sha_ctx, block, needed);
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
        c.err = 10; // ota_end_failed
        if (c.sm && c.st_error)
          c.sm->ReqTransition(*c.st_error);
        return;
      }

      err = esp_ota_set_boot_partition(c.ota_part);
      if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_ota_set_boot_partition failed: %s",
                 esp_err_to_name(err));
        c.err = 11;
        if (c.sm && c.st_error)
          c.sm->ReqTransition(*c.st_error);
        return;
      }

      ESP_LOGI(kTag, "ESP32 OTA Successful. Rebooting...");
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

void Programmer::VerifyingState::OnEnter(Ctx &c, SmTick) {
  c.verify_addr = 0x08000000;
  // Init SHA256 for readback
  mbedtls_sha256_init(&c.sha_ctx);
  mbedtls_sha256_starts(&c.sha_ctx, 0);
}

void Programmer::VerifyingState::OnStep(Ctx &c, SmTick) {
  // Read back in chunks
  uint32_t start_addr = 0x08000000;
  uint32_t offset = c.verify_addr - start_addr;

  // Process reasonable chunks (e.g. 256)
  while (offset < c.total_size) {
    size_t chunk = 256;
    if (chunk > (c.total_size - offset)) {
      chunk = c.total_size - offset;
    }

    uint8_t buf[256];
    if (!Programmer::GetInstance().ReadBlock(c.verify_addr, buf, chunk)) {
      c.err = 5; // verify_read_failed
      if (c.sm && c.st_error)
        c.sm->ReqTransition(*c.st_error);
      return;
    }

    mbedtls_sha256_update(&c.sha_ctx, buf, chunk);
    c.verify_addr += chunk;
    offset = c.verify_addr - start_addr;

    // Yield occasionally if we are blocking too long
    // But for performance, maybe do a few? 1 block is fine.
    return;
  }

  // Done reading
  uint8_t read_hash[32];
  mbedtls_sha256_finish(&c.sha_ctx, read_hash);
  mbedtls_sha256_free(&c.sha_ctx);

  // Compare
  if (std::memcmp(c.computed_hash, read_hash, 32) != 0) {
    ESP_LOGE(kTag, "Verification Failed! CRCs do not match.");
    c.err = 6; // verify_checksum_mismatch
    if (c.sm && c.st_error)
      c.sm->ReqTransition(*c.st_error);
  } else {
    ESP_LOGI(kTag, "Verification Successful. Hash matches.");
    if (c.sm && c.st_done)
      c.sm->ReqTransition(*c.st_done);
  }
}

void Programmer::ErrorState::OnEnter(Ctx &c, SmTick) {
  c.error_handler("Programmer Failed");
}
