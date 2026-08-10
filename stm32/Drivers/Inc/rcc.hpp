// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include <cstdint>

#include "stm32f4xx.h"

// Oscillators, PLL, bus dividers, flash latency and voltage scale — everything
// that decides what frequency the rest of the chip runs at. Brought up first,
// before any peripheral that derives a divider from a bus clock, and the same
// place those peripherals read that bus clock back from.
class Rcc {
 public:
  // Enum underlying ints are the raw RCC register-bit patterns (CMSIS, no
  // HAL), so static_cast<uint32_t> writes the field directly. Exceptions:
  // PllP holds the human divisor (InitOscillators encodes it), and Oscillator
  // is branched on in rcc.cpp since it drives several RCC fields.

  enum class Oscillator : uint8_t { kHsi, kHse };

  enum class PllP : uint32_t {
    kDiv2 = 2,
    kDiv4 = 4,
    kDiv6 = 6,
    kDiv8 = 8,
  };

  enum class AhbDiv : uint32_t {
    kDiv1 = RCC_CFGR_HPRE_DIV1,
    kDiv2 = RCC_CFGR_HPRE_DIV2,
    kDiv4 = RCC_CFGR_HPRE_DIV4,
    kDiv8 = RCC_CFGR_HPRE_DIV8,
    kDiv16 = RCC_CFGR_HPRE_DIV16,
    kDiv64 = RCC_CFGR_HPRE_DIV64,
    kDiv128 = RCC_CFGR_HPRE_DIV128,
    kDiv256 = RCC_CFGR_HPRE_DIV256,
    kDiv512 = RCC_CFGR_HPRE_DIV512,
  };

  // PPRE1 bit positions; APB2 reuses them shifted by 3 into PPRE2, so one
  // ApbDiv enum covers both buses.
  enum class ApbDiv : uint32_t {
    kDiv1 = RCC_CFGR_PPRE1_DIV1,
    kDiv2 = RCC_CFGR_PPRE1_DIV2,
    kDiv4 = RCC_CFGR_PPRE1_DIV4,
    kDiv8 = RCC_CFGR_PPRE1_DIV8,
    kDiv16 = RCC_CFGR_PPRE1_DIV16,
  };

  // On F405/F407 the PWR_CR.VOS bit selects scale 1 (set) or scale 2 (clear).
  // Scale 3 is not available on this part.
  enum class VoltageScale : uint32_t {
    kScale1 = PWR_CR_VOS,
    kScale2 = 0,
  };

  // Cross-field constraints (PLL VCO range, APB1/APB2 max freqs, flash latency
  // vs SYSCLK, voltage scale vs clock) are solved and checked by the config
  // generator, not here.
  struct Config {
    Oscillator oscillator;
    uint8_t pllm;   // 2..63
    uint16_t plln;  // 50..432
    PllP pllp;
    uint8_t pllq;  // 2..15
    AhbDiv ahb_divider;
    ApbDiv apb1_divider;
    ApbDiv apb2_divider;
    uint8_t flash_latency;
    VoltageScale voltage_scale;
    uint32_t hclk_hz;  // what the tree above solves to; published as
                       // SystemCoreClock
  };

  static Rcc &GetInstance();

  // Both buses share one prescaler encoding: values below 4 mean undivided,
  // and 4..7 select /2 through /16.
  static constexpr uint32_t ApbDivisor(uint32_t ppre) {
    return ppre < 4u ? 1u : (1u << (ppre - 3u));
  }

  // RM0090 6.2: a timer runs at its bus clock while that bus is undivided, and
  // at twice the bus clock otherwise -- so /1 and /2 both leave it at HCLK.
  static constexpr uint32_t ApbTimerHz(uint32_t pclk_hz, uint32_t divisor) {
    return divisor == 1u ? pclk_hz : pclk_hz * 2u;
  }

  // Read back from RCC rather than taken from the generated config, so a
  // peripheral sizing a divider uses the tree it actually got.
  static uint32_t Apb1Divisor();
  static uint32_t Apb2Divisor();
  static uint32_t Apb1Hz();
  static uint32_t Apb2Hz();
  static uint32_t Apb1TimerHz();
  static uint32_t Apb2TimerHz();

 private:
  friend class System;
  void Init(const Config &config);

  static void EnablePwrClock();  // RCC_APB1ENR.PWREN
  static void SetVoltageScale(VoltageScale scale);
  static void InitOscillators(const Config &cfg);
  static void InitClockTree(const Config &cfg);

  Rcc() = default;
  ~Rcc() = default;
  Rcc(const Rcc &) = delete;
  Rcc &operator=(const Rcc &) = delete;

  bool initialized_ = false;
};
