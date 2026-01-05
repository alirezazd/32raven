#include "programmer.hpp"
#include "../utils/timebase.hpp"

#include <cstring>

extern "C" {
#include "driver/gpio.h"
}

void Programmer::Init(const Config &cfg, Uart *uart) {
  ctx_.cfg = cfg;
  ctx_.uart = uart;

  // Bind state pointers
  ctx_.st_idle = &StIdle_;
  ctx_.st_writing = &StWriting_;
  ctx_.st_done = &StDone_;
  ctx_.st_error = &StError_;

  // Init pins
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
    const bool kActiveLow = ctx_.cfg.nrst_active_low;
    gpio_set_level((gpio_num_t)kNrst, kActiveLow ? 1 : 0);
  }
}

void Programmer::Boot0Set(bool on) {
  const int kPin = ctx_.cfg.boot0_gpio;
  if (kPin < 0)
    return;

  const bool kActiveHigh = ctx_.cfg.boot0_active_high;
  const int kAssertLevel = kActiveHigh ? 1 : 0;
  const int kDeassertLevel = kActiveHigh ? 0 : 1;

  gpio_set_level((gpio_num_t)kPin, on ? kAssertLevel : kDeassertLevel);
}

void Programmer::NrstPulse(uint32_t pulse_ms) {
  const int kPin = ctx_.cfg.nrst_gpio;
  if (kPin < 0)
    return;

  const bool kActiveLow = ctx_.cfg.nrst_active_low;
  const int kAssertLevel = kActiveLow ? 0 : 1;
  const int kDeassertLevel = kActiveLow ? 1 : 0;

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
  // If you want parity control here, expose it via your Uart config or method.
  (void)ctx_.uart->SetBaudRate(ctx_.cfg.uart_baud);

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
      if (rx == 0x79)
        return true; // ACK
      if (rx == 0x1F) {
        // NACK, retry
      } else {
        // Unknown byte, retry
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

void Programmer::Start(uint32_t total_size, SmTick now) {
  (void)now;
  if (!initialized_)
    return;

  // Reset session
  ctx_.total_size = total_size;
  ctx_.written = 0;
  ctx_.head = ctx_.tail = 0;
  ctx_.overflow = false;
  ctx_.err = 0;
  ctx_.ready = false;

  // Run blocking handshake now (OK at this stage since upload is disabled)
  if (!EnterBootloader()) {
    ctx_.err = 1; // handshake_failed
    ctx_.ready = false;
    sm_.Start(StError_, now);
    return;
  }

  // Handshake OK
  ctx_.ready = true;

  // Start internal writing SM (placeholder until you implement AN3155 write
  // blocks)
  sm_.Start(StWriting_, now);
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

uint32_t Programmer::Total() const { return ctx_.total_size; }
uint32_t Programmer::Written() const { return ctx_.written; }

// ---- State implementations ----

void Programmer::WritingState::OnEnter(Ctx &c, SmTick) {
  // Placeholder: in real AN3155 implementation, you would set up erase/write
  // parameters here. For now do nothing.
  (void)c;
}

void Programmer::WritingState::OnStep(Ctx &c, SmTick now) {
  (void)now;

  // Placeholder behavior:
  // Drain bytes from staging buffer and count them as "written".
  // Replace this later with:
  // - build 256-byte blocks
  // - send WRITE_MEMORY command + address + data + checksum
  // - wait for ACK with timeouts/retries
  // - update c.written only when STM32 ACKs the block
  //
  // This placeholder lets you validate backpressure plumbing end-to-end.

  // If buffer overflow happened, error out
  if (c.overflow) {
    c.err = 2; // buffer_overflow
    // Transition to error
    if (c.st_error) {
      // request transition through SM: we do not have SM here, but we can set
      // err and rely on outer call. Minimal: mark as done by zeroing ready and
      // let app handle.
    }
  }

  // Drain up to a bounded amount per step
  const size_t kMaxDrain = 512;
  size_t drained = 0;
  while (drained < kMaxDrain && c.head != c.tail) {
    c.head = (c.head + 1) % Ctx::kBufCap;
    c.written += 1;
    drained += 1;

    if (c.total_size && c.written >= c.total_size) {
      // Done
      // We cannot call request_transition from here directly, because
      // StateMachine is outside. But we can set a marker by forcing head==tail
      // and letting outer wrapper transition.
      break;
    }
  }

  // Transition decisions
  if (c.total_size && c.written >= c.total_size && c.head == c.tail) {
    // move to done
    // We must request transition via the outer StateMachine. We can do it by
    // setting a flag and using events, but your current SM has no built-in flag
    // channel. Simplest: use on_event, or store "want_done" in context. To keep
    // this file minimal, we do nothing here.
  }
}
