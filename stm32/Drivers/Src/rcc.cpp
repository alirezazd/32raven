// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "rcc.hpp"

#include "error_code.hpp"
#include "irq_priority.hpp"
#include "panic.hpp"
#include "stm32_config.hpp"
#include "uart.hpp"

namespace {
// Millisecond counter driven by the SysTick exception (1 kHz). It exists to
// bound the spin-waits below, which run before TimeBase (TIM2) is up.
volatile uint32_t g_boot_tick_ms = 0;

constexpr uint32_t kHseTimeoutMs = 100u;
constexpr uint32_t kHsiTimeoutMs = 2u;
constexpr uint32_t kPllTimeoutMs = 2u;
constexpr uint32_t kClockSwitchTimeoutMs = 5000u;
constexpr uint32_t kPllSourceHse = RCC_PLLCFGR_PLLSRC_HSE;
constexpr uint32_t kPllSourceHsi = 0u;
constexpr uint32_t kHsiHz = 16000000u;  // fixed silicon, not a Kconfig value
constexpr uint32_t kMicrosPerSecond = 1000000u;

// ApbDiv holds PPRE1-aligned field values; PPRE2 takes the same encoding at its
// own bit position, so take the offset from CMSIS instead of hand-copying it.
constexpr uint32_t kPpre2Shift = RCC_CFGR_PPRE2_Pos - RCC_CFGR_PPRE1_Pos;

constexpr uint32_t Ppre2Bits(Rcc::ApbDiv d) {
  return static_cast<uint32_t>(d) << kPpre2Shift;
}

// Reload fits 24 bits: 16000 at HSI 16 MHz boot, 168000 at target 168 MHz.
void ArmSysTick(uint32_t hclk_hz) {
  (void)SysTick_Config(hclk_hz / 1000U);
  NVIC_SetPriority(SysTick_IRQn, irq_priority::kSysTick);
}
}  // namespace

// C-callable tick increment, invoked from stm32f4xx_it.c::SysTick_Handler.
extern "C" void SystemTickInc(void) {
  g_boot_tick_ms = g_boot_tick_ms + 1u;
}

// Clock Security System failsafe, invoked from the NMI when HSE fails. Hardware
// has already disabled HSE and switched SYSCLK to HSI; normalise the tree onto
// a known HSI @ 16 MHz so panic LED cadence and telemetry baud stay correct,
// then hand off to Panic() (no reboot — the panic loop keeps the watchdog fed).
extern "C" void SystemOnClockSecurityFailure(void) {
  RCC->CIR = RCC_CIR_CSSC;  // clear CSSF so the NMI stops re-pending

  RCC->CR |= RCC_CR_HSION;
  while ((RCC->CR & RCC_CR_HSIRDY) == 0u) {
  }
  // SYSCLK = HSI (SW=00), AHB/APB1/APB2 prescalers = /1 -> 16 MHz everywhere.
  RCC->CFGR &= ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
  }
  SystemCoreClock = kHsiHz;  // not the configured HCLK — the PLL is gone

  // Re-fix what the panic loop depends on at the new 16 MHz clock: the TIM2 µs
  // tick, and the USART1 panic-telemetry baud against the new PCLK2.
  TIM2->PSC = static_cast<uint16_t>(Rcc::Apb1TimerHz() / kMicrosPerSecond - 1u);
  TIM2->EGR = TIM_EGR_UG;
  Uart1::GetInstance().SetBaudRate(kUart1Config.baud_rate);

  Panic(ErrorCode::Stm32::kHseClockFailure);
}

Rcc &Rcc::GetInstance() {
  static Rcc instance;
  return instance;
}

uint32_t Rcc::Apb1Divisor() {
  return ApbDivisor((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos);
}

uint32_t Rcc::Apb2Divisor() {
  return ApbDivisor((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos);
}

uint32_t Rcc::Apb1Hz() { return SystemCoreClock / Apb1Divisor(); }

uint32_t Rcc::Apb2Hz() { return SystemCoreClock / Apb2Divisor(); }

uint32_t Rcc::Apb1TimerHz() {
  const uint32_t divisor = Apb1Divisor();
  return ApbTimerHz(SystemCoreClock / divisor, divisor);
}

uint32_t Rcc::Apb2TimerHz() {
  const uint32_t divisor = Apb2Divisor();
  return ApbTimerHz(SystemCoreClock / divisor, divisor);
}

// The read-back is a bus-retire barrier: without it the next PWR access can
// reach the peripheral before its clock gate has taken effect.
void Rcc::EnablePwrClock() {
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  (void)RCC->APB1ENR;
}

// Read-back barrier as in EnablePwrClock.
void Rcc::SetVoltageScale(VoltageScale scale) {
  PWR->CR = (PWR->CR & ~PWR_CR_VOS) | static_cast<uint32_t>(scale);
  (void)PWR->CR;
}

// HSE/HSI + PLL bring-up. Boot-only, so reentrancy guards and unused
// LSI/LSE/HSE_BYPASS/PLL-OFF paths are omitted (Init Panics on re-init;
// SYSCLK = HSI from reset is the only entry state). HSI calibration trim is not
// written: RCC_CR.HSITRIM[7:3] resets to 0x10 (RM0090 §6.3.1), so it's a no-op.
void Rcc::InitOscillators(const Config &cfg) {
  const bool use_hse = (cfg.oscillator == Oscillator::kHse);
  uint32_t tickstart;

  if (use_hse) {
    // HSEON, no bypass: external crystal, not a clock input.
    RCC->CR |= RCC_CR_HSEON;
    tickstart = g_boot_tick_ms;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
      if ((g_boot_tick_ms - tickstart) > kHseTimeoutMs) {
        Panic(ErrorCode::Stm32::kRccOscConfigFailed);
      }
    }
  } else {
    // HSI is on out of reset; this re-enable is defensive.
    RCC->CR |= RCC_CR_HSION;
    tickstart = g_boot_tick_ms;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
      if ((g_boot_tick_ms - tickstart) > kHsiTimeoutMs) {
        Panic(ErrorCode::Stm32::kRccOscConfigFailed);
      }
    }
  }

  // Disable PLL before reconfiguring (off from reset; defensive).
  RCC->CR &= ~RCC_CR_PLLON;
  tickstart = g_boot_tick_ms;
  while ((RCC->CR & RCC_CR_PLLRDY) != 0U) {
    if ((g_boot_tick_ms - tickstart) > kPllTimeoutMs) {
      Panic(ErrorCode::Stm32::kRccOscConfigFailed);
    }
  }

  // PLLCFGR field packing:
  //   PLLSRC : bit 22       (kPllSource* is already pre-shifted)
  //   PLLM   : bits [5:0]   (raw value, no shift)
  //   PLLN   : bits [14:6]
  //   PLLP   : bits [17:16] (encoded ((PLLP>>1)-1))
  //   PLLQ   : bits [27:24]
  const uint32_t pll_source = use_hse ? kPllSourceHse : kPllSourceHsi;
  const uint32_t pllp_raw = static_cast<uint32_t>(cfg.pllp);
  RCC->PLLCFGR = pll_source | cfg.pllm | (cfg.plln << RCC_PLLCFGR_PLLN_Pos) |
                 (((pllp_raw >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) |
                 (cfg.pllq << RCC_PLLCFGR_PLLQ_Pos);

  RCC->CR |= RCC_CR_PLLON;
  tickstart = g_boot_tick_ms;
  while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
    if ((g_boot_tick_ms - tickstart) > kPllTimeoutMs) {
      Panic(ErrorCode::Stm32::kRccOscConfigFailed);
    }
  }
}

// AHB/APB dividers + SYSCLK switch to PLL + flash latency. Order is
// silicon-safety-critical:
//   1. Raise flash latency BEFORE SYSCLK rises — too few wait states at high
//      clock corrupts instruction fetch.
//   2. Park APB1+APB2 prescalers at /16 BEFORE the switch, else APB1 briefly
//      runs at HCLK (168 MHz), far over its 42 MHz spec.
//   3. Set the AHB divider.
//   4. Switch SYSCLK source to PLL and spin on RCC_CFGR.SWS.
//   5. Lower flash latency if target < current (dead at boot).
//   6. Apply target APB1+APB2 dividers once SYSCLK is stable.
//   7. Recompute SystemCoreClock and re-arm SysTick at the new HCLK so
//      SystemTickInc keeps firing at 1 kHz.
void Rcc::InitClockTree(const Config &cfg) {
  const uint32_t target_latency = cfg.flash_latency;
  const uint32_t current_latency = FLASH->ACR & FLASH_ACR_LATENCY;

  // 1. Latency UP.
  if (target_latency > current_latency) {
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | target_latency;
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != target_latency) {
      Panic(ErrorCode::Stm32::kRccClockConfigFailed);
    }
  }

  // 2. Park APB1/APB2 at /16 across the switch.
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV16;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV16;

  // 3. AHB divider (HPRE).
  RCC->CFGR =
      (RCC->CFGR & ~RCC_CFGR_HPRE) | static_cast<uint32_t>(cfg.ahb_divider);

  // 4. Switch SYSCLK → PLL; SWS mirrors SW once accepted. PLL is already
  //    locked (InitOscillators Panicked otherwise).
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  uint32_t tickstart = g_boot_tick_ms;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    if ((g_boot_tick_ms - tickstart) > kClockSwitchTimeoutMs) {
      Panic(ErrorCode::Stm32::kRccClockConfigFailed);
    }
  }

  // 5. Latency DOWN. Dead at boot for our config.
  if (target_latency < current_latency) {
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | target_latency;
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != target_latency) {
      Panic(ErrorCode::Stm32::kRccClockConfigFailed);
    }
  }

  // 6. Apply target APB1 + APB2 dividers.
  RCC->CFGR =
      (RCC->CFGR & ~RCC_CFGR_PPRE1) | static_cast<uint32_t>(cfg.apb1_divider);
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | Ppre2Bits(cfg.apb2_divider);

  // 7. Publish SystemCoreClock and re-arm SysTick at the new HCLK.
  SystemCoreClock = cfg.hclk_hz;
  ArmSysTick(SystemCoreClock);
}

void Rcc::Init(const Config &config) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kRccClockConfigFailed);
  }
  initialized_ = true;

  // The spin-wait timeouts below need the millisecond tick, and nothing has
  // armed it yet: SYSCLK is still the 16 MHz HSI the part booted on.
  ArmSysTick(SystemCoreClock);

  EnablePwrClock();
  SetVoltageScale(config.voltage_scale);
  InitOscillators(config);
  InitClockTree(config);

  // Arm CSS (HSE only): on HSE failure hardware falls back to HSI and raises an
  // NMI -> SystemOnClockSecurityFailure().
  if (config.oscillator == Oscillator::kHse) {
    RCC->CR |= RCC_CR_CSSON;
  }
}
