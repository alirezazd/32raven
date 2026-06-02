#pragma once
#include <cstdint>

#include "stm32f4xx.h"

// Independent watchdog (IWDG). Runs off the LSI (~32 kHz), so it survives a
// main-clock failure and resets the MCU unless Kick() is called within the
// timeout. Kick from the main loop (and the panic loop) ONLY — never from an
// ISR, or a wedged main loop would still be fed and the hang would go
// undetected.
class Watchdog {
 public:
  static Watchdog &GetInstance() {
    static Watchdog instance;
    return instance;
  }

  // Reload the counter — the periodic "still alive" signal. A single register
  // write; a harmless no-op if the watchdog was never started.
  void Kick() { IWDG->KR = kKeyReload; }

 private:
  friend class System;  // System::Init arms the watchdog once everything is up.

  Watchdog() = default;
  Watchdog(const Watchdog &) = delete;
  Watchdog &operator=(const Watchdog &) = delete;

  // Arm the watchdog: /64 prescaler off the LSI -> ~1 s nominal timeout (LSI is
  // ±~30%, so the real window is ~0.7-1.9 s). Frozen while the core is halted
  // under a debugger. Irreversible once armed; System::Init() calls this last,
  // after every component, so blocking bring-up can't trip it.
  void Init() {
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;  // pause when core is halted
    IWDG->KR = kKeyAccess;     // enable write access to PR/RLR
    IWDG->PR = IWDG_PR_PR_2;   // prescaler /64 -> ~2 ms per tick
    IWDG->RLR = kReloadCount;  // (kReloadCount + 1) ticks ≈ 1.0 s
    while (IWDG->SR != 0u) {   // wait for the PR/RLR write to latch
    }
    Kick();
    IWDG->KR = kKeyStart;  // start (irreversible)
  }

  static constexpr uint32_t kKeyAccess = 0x00005555u;  // unlock PR/RLR
  static constexpr uint32_t kKeyReload = 0x0000AAAAu;  // reload the counter
  static constexpr uint32_t kKeyStart = 0x0000CCCCu;   // start the watchdog
  static constexpr uint32_t kReloadCount = 499u;
};
