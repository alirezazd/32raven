#pragma once
#include "mbedtls/sha256.h" // IWYU pragma: keep
#include "uart.hpp"

enum class ErrorCode;

extern "C" {
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h" // IWYU pragma: keep
}

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "state_machine.hpp"

class Programmer {
public:
  static Programmer &GetInstance() {
    static Programmer instance;
    return instance;
  }

  struct Config {
    // STM32 boot control pins (ESP32 GPIO numbers)
    int boot0_gpio = 6;
    int nrst_gpio = 7; // active low reset

    uint32_t reset_pulse_ms = 50;
    uint32_t boot_settle_ms = 50;

    uint32_t sync_timeout_ms = 100;
    uint8_t sync_retries = 10;
  };

  enum class Target { kStm32, kEsp32 };

  void SetTarget(Target t) { ctx_.target = t; }

  void Start(uint32_t total_size, SmTick now);
  void Poll(SmTick now);

  void Abort(SmTick now);

  // Feed bytes and internally advance writing SM.
  // Returns bytes accepted (may be < n for backpressure).
  size_t PushBytes(const uint8_t *data, size_t n, SmTick now);

  bool Ready() const; // handshake done, ready to accept bytes
  bool Done() const;
  bool Error() const;
  bool IsVerifying() const;

  uint32_t Total() const;
  uint32_t Written() const;

  size_t Free() const;

  bool IsInitialized() const { return initialized_; }

  bool Boot();

private:
  friend class System;
  ErrorCode Init(const Config &cfg, Uart *uart);
  // ---- Internal context used by StateMachine<Ctx> ----
  struct Ctx {
    Uart *uart = nullptr;
    Config cfg{};
    Target target = Target::kStm32;

    uint32_t total_size = 0;
    uint32_t written = 0;
    uint32_t write_addr = 0x08000000; // Start of STM32 Flash

    StateMachine<Ctx> *sm = nullptr;

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
    IState<Ctx> *st_verifying = nullptr; // New State
    IState<Ctx> *st_done = nullptr;
    IState<Ctx> *st_error = nullptr;

    // verification
    mbedtls_sha256_context sha_ctx;
    uint8_t computed_hash[32];
    uint32_t verify_addr = 0x08000000;

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

  class IdleState : public IState<Ctx> {
  public:
    const char *Name() const override { return "P.Idle"; }
    void OnEnter(Ctx &, SmTick) override {}
    void OnStep(Ctx &, SmTick) override {}
    void OnExit(Ctx &, SmTick) override {}
  };

  class WritingState : public IState<Ctx> {
  public:
    const char *Name() const override { return "P.Writing"; }
    void OnEnter(Ctx &c, SmTick now) override;
    void OnStep(Ctx &c, SmTick now) override;
    void OnExit(Ctx &, SmTick) override {}
  };

  class VerifyingState : public IState<Ctx> {
  public:
    const char *Name() const override { return "P.Verifying"; }
    void OnEnter(Ctx &c, SmTick now) override;
    void OnStep(Ctx &c, SmTick now) override;
    void OnExit(Ctx &c, SmTick now) override {}
  };

  class DoneState : public IState<Ctx> {
  public:
    const char *Name() const override { return "P.Done"; }
    void OnEnter(Ctx &, SmTick) override {}
    void OnStep(Ctx &, SmTick) override {}
    void OnExit(Ctx &, SmTick) override {}
  };

  class ErrorState : public IState<Ctx> {
  public:
    const char *Name() const override { return "P.Error"; }
    void OnEnter(Ctx &, SmTick) override;
    void OnStep(Ctx &, SmTick) override {}
    void OnExit(Ctx &, SmTick) override {}
  };

  void GpioInit();
  void Boot0Set(bool on);
  void NrstPulse(uint32_t pulse_ms);
  bool EnterBootloader();   // BOOT0/reset/0x7F/ACK (bounded retries)
  bool GetBootloaderInfo(); // CMD_GET (0x00)
  bool EraseAll();          // CMD_ERASE (0x43) or CMD_EXT_ERASE (0x44)
  bool WriteBlock(uint32_t addr, const uint8_t *data,
                  size_t len); // CMD_WRITE_MEMORY (0x31)
  bool ReadBlock(uint32_t addr, uint8_t *data,
                 size_t len); // CMD_READ_MEMORY (0x11)

  bool initialized_ = false;

  Ctx ctx_{};
  StateMachine<Ctx> sm_{ctx_};

  IdleState StIdle_{};
  WritingState StWriting_{};
  VerifyingState StVerifying_{};
  DoneState StDone_{};
  ErrorState StError_{};

  Programmer() = default;
  ~Programmer() = default;
  Programmer(const Programmer &) = delete;
  Programmer &operator=(const Programmer &) = delete;
};
