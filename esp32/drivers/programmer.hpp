#pragma once
#include "uart.hpp"
#include <cstddef>
#include <cstdint>

#include "state_machine.hpp"

class Programmer {
public:
  static Programmer &Get() {
    static Programmer instance;
    return instance;
  }

  struct Config {
    // STM32 boot control pins (ESP32 GPIO numbers)
    int boot0_gpio = -1; // set high to enter ROM bootloader
    int nrst_gpio = -1;  // active low reset

    bool boot0_active_high = true;
    bool nrst_active_low = true;

    uint32_t reset_pulse_ms = 50;
    uint32_t boot_settle_ms = 50;

    uint32_t sync_timeout_ms = 100;
    uint8_t sync_retries = 10;

    uint32_t uart_baud = 115200;
    Uart::Config::Parity uart_parity = Uart::Config::Parity::kEven;
  };

  // Begin a new flashing session: prepare STM32 bootloader and arm for writing.
  // You can implement handshake as blocking inside Start() if you want.
  void Start(uint32_t total_size, sm_tick_t now);

  void Abort(sm_tick_t now);

  // Feed bytes and internally advance writing SM.
  // Returns bytes accepted (may be < n for backpressure).
  size_t PushBytes(const uint8_t *data, size_t n, sm_tick_t now);

  bool Ready() const; // handshake done, ready to accept bytes
  bool Done() const;
  bool Error() const;

  uint32_t Total() const;
  uint32_t Written() const;

  bool IsInitialized() const { return initialized_; }

private:
  friend class System;
  void Init(const Config &cfg, Uart *uart);

  // ---- Internal context used by StateMachine<Ctx> ----
  struct Ctx {
    Uart *uart = nullptr;
    Config cfg{};

    uint32_t total_size = 0;
    uint32_t written = 0;

    bool ready = false;

    // staging buffer for bytes coming from HTTP (backpressure lives here)
    static constexpr size_t kBufCap = 2048;
    uint8_t buf[kBufCap]{};
    size_t head = 0;
    size_t tail = 0;
    bool overflow = false;

    uint32_t err = 0;

    // transitions
    IState<Ctx> *st_idle = nullptr;
    IState<Ctx> *st_writing = nullptr;
    IState<Ctx> *st_done = nullptr;
    IState<Ctx> *st_error = nullptr;
  };

  static size_t rb_used_(size_t head, size_t tail, size_t cap) {
    return (tail >= head) ? (tail - head) : (cap - head + tail);
  }
  static size_t rb_free_(size_t head, size_t tail, size_t cap) {
    return (cap - 1) - rb_used_(head, tail, cap);
  }

  // ---- Internal states (your SM template) ----
  class IdleState : public IState<Ctx> {
  public:
    const char *name() const override { return "P.Idle"; }
    void on_enter(Ctx &, sm_tick_t) override {}
    void on_step(Ctx &, sm_tick_t) override {}
    void on_exit(Ctx &, sm_tick_t) override {}
  };

  class WritingState : public IState<Ctx> {
  public:
    const char *name() const override { return "P.Writing"; }
    void on_enter(Ctx &c, sm_tick_t now) override;
    void on_step(Ctx &c, sm_tick_t now) override;
    void on_exit(Ctx &, sm_tick_t) override {}
  };

  class DoneState : public IState<Ctx> {
  public:
    const char *name() const override { return "P.Done"; }
    void on_enter(Ctx &, sm_tick_t) override {}
    void on_step(Ctx &, sm_tick_t) override {}
    void on_exit(Ctx &, sm_tick_t) override {}
  };

  class ErrorState : public IState<Ctx> {
  public:
    const char *name() const override { return "P.Error"; }
    void on_enter(Ctx &, sm_tick_t) override {}
    void on_step(Ctx &, sm_tick_t) override {}
    void on_exit(Ctx &, sm_tick_t) override {}
  };

  // ---- Private helpers (implemented in .cpp) ----
  void gpio_init_();
  void boot0_set_(bool on);
  void nrst_pulse_(uint32_t pulse_ms);
  bool handshake_blocking_(); // BOOT0/reset/0x7F/ACK (bounded retries)

  bool initialized_ = false;

  Ctx ctx_{};
  StateMachine<Ctx> sm_{ctx_};

  IdleState st_idle_{};
  WritingState st_writing_{};
  DoneState st_done_{};
  ErrorState st_error_{};

  Programmer() = default;
  ~Programmer() = default;
  Programmer(const Programmer &) = delete;
  Programmer &operator=(const Programmer &) = delete;
};
