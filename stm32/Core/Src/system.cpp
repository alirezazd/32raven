#include "system.hpp"

#include "button.hpp"
#include "dshot_tim1.hpp"
#include "error_code.hpp"
#include "gpio.hpp"
#include "irq_priority.hpp"
#include "led.hpp"
#include "panic.hpp"
#include "spi.hpp"
#include "stm32_config.hpp"
#include "time_base.hpp"
#include "uart.hpp"
#include "watchdog.hpp"

namespace {
// Millisecond counter driven by the SysTick exception (1 kHz). Backs spin-wait
// timeouts in InitOscillators/InitClockTree before TimeBase (TIM2) is up.
volatile uint32_t g_system_tick_ms = 0;

// Clock bring-up timeouts (ms) and PLL-source selector bits.
constexpr uint32_t kHseTimeoutMs = 100u;  // HSE_STARTUP_TIMEOUT
constexpr uint32_t kHsiTimeoutMs = 2u;
constexpr uint32_t kPllTimeoutMs = 2u;
constexpr uint32_t kClockSwitchTimeoutMs = 5000u;
constexpr uint32_t kPllSourceHse = RCC_PLLCFGR_PLLSRC_HSE;  // PLLSRC bit set
constexpr uint32_t kPllSourceHsi = 0u;                      // PLLSRC bit clr
}  // namespace

// C-callable tick increment, invoked from stm32f4xx_it.c::SysTick_Handler.
extern "C" void SystemTickInc(void) {
  g_system_tick_ms = g_system_tick_ms + 1u;
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
  SystemCoreClockUpdate();

  // Re-fix what the panic loop depends on at the new 16 MHz clock:
  // TIM2 µs tick — APB1 timer clock now 16 MHz (PPRE1 = /1) -> /16 = 1 MHz.
  TIM2->PSC = 16u - 1u;
  TIM2->EGR = TIM_EGR_UG;
  // USART1 panic-telemetry baud — recompute BRR against the new PCLK2.
  Uart1::GetInstance().SetBaudRate(kUart1Config.baud_rate);

  Panic(
      ErrorCode::Stm32::kHseClockFailure);  // disarms + screams; never returns
}

// Out-of-line so the `static System` lives in this one TU rather than emitting
// a linkonce/COMDAT copy per includer.
System &System::GetInstance() {
  static System instance;
  return instance;
}

void System::Init(const System::Config &config) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSystemReinit);
  }
  initialized_ = true;

  CoreInit();
  ConfigureSystemClock(config);
  NVIC_SetPriority(PendSV_IRQn, irq_priority::kPendSv);

  InitComponent(Component::kTimeBase);
  InitComponent(Component::kGpio);
  InitComponent(Component::kUart1);
  InitComponent(Component::kSpi1);
  InitComponent(Component::kEe);
  InitComponent(Component::kBattery);
  InitComponent(Component::kUart6);
  InitComponent(Component::kRcReceiver);
  InitComponent(Component::kCrsfLink);
  InitComponent(Component::kLed);
  InitComponent(Component::kSpi2);
  InitComponent(Component::kDshot);
  InitComponent(Component::kEscTelemetry);
  InitComponent(Component::kEscService);
  InitComponent(Component::kButton);
  InitComponent(Component::kUart2);
  InitComponent(Component::kM10);
  InitComponent(Component::kIcm42688p);
  InitComponent(Component::kMultirotorMixer);
  InitComponent(Component::kAhrs);
  InitComponent(Component::kRateController);
  InitComponent(Component::kAttitudeController);

  // Arm the watchdog last so blocking bring-up can't trip it. From here the
  // main loop (and the panic loop) must keep feeding it.
  Wdg().Init();
}

// ─── Direct-register CPU + clock bring-up helpers ────────────────────
// Boot-only static methods, invoked from System::Init / ConfigureSystemClock.
// On failure: Panic, matching the firmware-wide driver/service Init convention.

// Cortex / flash / SysTick bring-up: I-cache, D-cache, prefetch buffer, NVIC
// priority grouping (4 preempt bits, 0 sub), and the 1 kHz SysTick that backs
// the InitOscillators/InitClockTree spin-wait timeouts.
void System::CoreInit() {
  FLASH->ACR |= FLASH_ACR_ICEN;
  FLASH->ACR |= FLASH_ACR_DCEN;
  FLASH->ACR |= FLASH_ACR_PRFTEN;
  NVIC_SetPriorityGrouping(0x3U);  // group 4: 4 preempt bits, 0 sub
  // Reload fits 24 bits: 16000 at HSI 16 MHz boot, 168000 at target 168 MHz.
  (void)SysTick_Config(SystemCoreClock / 1000U);
  NVIC_SetPriority(SysTick_IRQn, irq_priority::kSysTick);
}

// APB1 clock gate to PWR (RCC_APB1ENR.PWREN). Read-back enforces a bus-retire
// barrier before the next PWR register access.
void System::EnablePwrClock() {
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  (void)RCC->APB1ENR;
}

// PWR_CR.VOS field write. Read-back barrier as in EnablePwrClock.
void System::SetVoltageScale(VoltageScale scale) {
  PWR->CR = (PWR->CR & ~PWR_CR_VOS) | static_cast<uint32_t>(scale);
  (void)PWR->CR;
}

// HSE/HSI + PLL bring-up. Boot-only, so reentrancy guards and unused
// LSI/LSE/HSE_BYPASS/PLL-OFF paths are omitted (System::Init Panics on re-init;
// SYSCLK = HSI from reset is the only entry state). HSI calibration trim is not
// written: RCC_CR.HSITRIM[7:3] resets to 0x10 (RM0090 §6.3.1), so it's a no-op.
void System::InitOscillators(const OscillatorConfig &cfg) {
  const bool use_hse = (cfg.oscillator == Oscillator::kHse);
  uint32_t tickstart;

  if (use_hse) {
    // HSEON, no bypass: external crystal, not a clock input.
    RCC->CR |= RCC_CR_HSEON;
    tickstart = g_system_tick_ms;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
      if ((g_system_tick_ms - tickstart) > kHseTimeoutMs) {
        Panic(ErrorCode::Stm32::kRccOscConfigFailed);
      }
    }
  } else {
    // HSI is on out of reset; this re-enable is defensive.
    RCC->CR |= RCC_CR_HSION;
    tickstart = g_system_tick_ms;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
      if ((g_system_tick_ms - tickstart) > kHsiTimeoutMs) {
        Panic(ErrorCode::Stm32::kRccOscConfigFailed);
      }
    }
  }

  // Disable PLL before reconfiguring (off from reset; defensive).
  RCC->CR &= ~RCC_CR_PLLON;
  tickstart = g_system_tick_ms;
  while ((RCC->CR & RCC_CR_PLLRDY) != 0U) {
    if ((g_system_tick_ms - tickstart) > kPllTimeoutMs) {
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
  tickstart = g_system_tick_ms;
  while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
    if ((g_system_tick_ms - tickstart) > kPllTimeoutMs) {
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
void System::InitClockTree(const Config &cfg) {
  const uint32_t target_latency = cfg.flash_latency;
  const uint32_t current_latency = FLASH->ACR & FLASH_ACR_LATENCY;

  // 1. Latency UP.
  if (target_latency > current_latency) {
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | target_latency;
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != target_latency) {
      Panic(ErrorCode::Stm32::kRccClockConfigFailed);
    }
  }

  // 2. Park APB1/APB2 at /16 across the switch. APB2 reuses PPRE1 bit values
  //    shifted left by 3.
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV16;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | (RCC_CFGR_PPRE1_DIV16 << 3);

  // 3. AHB divider (HPRE).
  RCC->CFGR =
      (RCC->CFGR & ~RCC_CFGR_HPRE) | static_cast<uint32_t>(cfg.ahb_divider);

  // 4. Switch SYSCLK → PLL; SWS mirrors SW once accepted. PLL is already
  //    locked (InitOscillators Panicked otherwise).
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  uint32_t tickstart = g_system_tick_ms;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    if ((g_system_tick_ms - tickstart) > kClockSwitchTimeoutMs) {
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
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) |
              (static_cast<uint32_t>(cfg.apb2_divider) << 3);

  // 7. Recompute SystemCoreClock and re-arm SysTick at the new HCLK.
  SystemCoreClockUpdate();
  (void)SysTick_Config(SystemCoreClock / 1000U);
  NVIC_SetPriority(SysTick_IRQn, irq_priority::kSysTick);
}

void System::InitComponent(Component c) {
  switch (c) {
    case Component::kTimeBase:
      System::GetInstance().Time().Init(kTimeBaseConfig);
      break;
    case Component::kGpio:
      GPIO::GetInstance().Init(kGpioDefault);
      break;
    case Component::kSpi1:
      Spi1::GetInstance().Init(kEeSpi1Config);
      break;
    case Component::kEe:
      EE::GetInstance().Init(GPIO::GetInstance(), Spi1::GetInstance());
      break;
    case Component::kBattery:
      Battery::GetInstance().Init(kBatteryConfig);
      break;
    case Component::kUart6:
      Uart6::GetInstance().Init(kUart6Config);
      break;
    case Component::kRcReceiver:
      RcReceiver::GetInstance().Init(kRcReceiverConfig, EE::GetInstance(),
                                     vehicle_state_);
      break;
    case Component::kCrsfLink:
      crsf_link_service_.Init(kCrsfLinkConfig, Uart6::GetInstance(),
                              vehicle_state_, RcReceiver::GetInstance(),
                              FcLink::GetInstance());
      break;
    case Component::kLed:
      LED::GetInstance().Init(GPIO::GetInstance(), kLedConfig);
      LED::GetInstance().Set(true);
      break;
    case Component::kUart1:
      Uart1::GetInstance().Init(kUart1Config);
      break;
    case Component::kSpi2:
      Spi2::GetInstance().Init(kSpi2Config);
      break;
    case Component::kDshot:
      DShotTim1::GetInstance().Init(kDshotTim1Config);
      break;
    case Component::kEscTelemetry:
      EscTelemetry::GetInstance().Init(kEscTelemetryConfig);
      break;
    case Component::kEscService:
      esc_service_.Init(kEscServiceConfig, EscTelemetry::GetInstance(),
                        vehicle_state_);
      break;
    case Component::kButton:
      Button::GetInstance().Init(GPIO::GetInstance(), kButtonConfig);
      break;
    case Component::kUart2:
      Uart2::GetInstance().Init(kUart2Config);
      break;
    case Component::kM10:
      M10::GetInstance().Init(kM10Config);
      break;
    case Component::kIcm42688p:
      Icm42688p::GetInstance().Init(GPIO::GetInstance(), Spi2::GetInstance(),
                                    EE::GetInstance(), kIcm42688pConfig);
      break;
    case Component::kMultirotorMixer:
      mixer_.Init(kMultirotorMixerConfig);
      break;
    case Component::kAhrs:
      ahrs_.Init(kAhrsConfig);
      break;
    case Component::kRateController:
      rate_controller_.Init(kRateControllerConfig);
      break;
    case Component::kAttitudeController:
      attitude_controller_.Init(kAttitudeControllerConfig);
      break;
  }
}

void System::ConfigureSystemClock(const System::Config &config) {
  EnablePwrClock();
  SetVoltageScale(config.voltage_scale);

  InitOscillators({
      .oscillator = config.oscillator,
      .pllm = config.pllm,
      .plln = config.plln,
      .pllp = config.pllp,
      .pllq = config.pllq,
  });

  InitClockTree(config);

  // Arm CSS (HSE only): on HSE failure hardware falls back to HSI and raises an
  // NMI -> SystemOnClockSecurityFailure().
  if (config.oscillator == Oscillator::kHse) {
    RCC->CR |= RCC_CR_CSSON;
  }
}
