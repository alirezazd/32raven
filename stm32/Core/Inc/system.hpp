#pragma once

#include "ahrs.hpp"
#include "attitude_controller.hpp"
#include "battery.hpp"
#include "button.hpp"
#include "command_handler.hpp"
#include "crsf_link_service.hpp"
#include "esc_service.hpp"
#include "fc_link.hpp"
#include "gpio.hpp"
#include "icm42688p.hpp"
#include "led.hpp"
#include "m10_service.hpp"
#include "multirotor_mixer.hpp"
#include "rate_controller.hpp"
#include "rc_receiver.hpp"
#include "stat_publisher.hpp"
#include "time_base.hpp"
#include "uart.hpp"
#include "vehicle_state.hpp"
#include "watchdog.hpp"

class System {
 public:
  // ── Clock + power configuration types ─────────────────────────────
  // Enum underlying ints are the raw RCC register-bit patterns (CMSIS,
  // no HAL), so static_cast<uint32_t> writes the field directly.
  // Exceptions: PllP holds the human divisor (InitOscillators encodes
  // it), and Oscillator is branched on in system.cpp since it drives
  // several RCC fields (HSE/HSI on/off, PLL source).

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

  // PPRE1 bit positions; APB2 reuses them shifted by 3 into PPRE2, so
  // one ApbDiv enum covers both buses.
  enum class ApbDiv : uint32_t {
    kDiv1 = RCC_CFGR_PPRE1_DIV1,
    kDiv2 = RCC_CFGR_PPRE1_DIV2,
    kDiv4 = RCC_CFGR_PPRE1_DIV4,
    kDiv8 = RCC_CFGR_PPRE1_DIV8,
    kDiv16 = RCC_CFGR_PPRE1_DIV16,
  };

  // On F405/F407 the PWR_CR.VOS bit selects scale 1 (set) or scale 2
  // (clear). Scale 3 is not available on this part.
  enum class VoltageScale : uint32_t {
    kScale1 = PWR_CR_VOS,
    kScale2 = 0,
  };

  // InitOscillators input: PLL reference + divider chain. pllp is the
  // human divisor (2/4/6/8), encoded to ((P>>1)-1) internally; pllm/n/q
  // are raw register values.
  struct OscillatorConfig {
    Oscillator oscillator;
    uint32_t pllm;
    uint32_t plln;
    PllP pllp;
    uint32_t pllq;
  };

  // Clock + power config for Init(). Cross-field constraints (PLL VCO
  // range, APB1/APB2 max freqs, flash latency vs SYSCLK, voltage scale
  // vs clock) are caller's responsibility — unchecked here.
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
  };

  static System &GetInstance();

  // Component identifiers used by InitComponent().
  enum class Component {
    kTimeBase,
    kGpio,
    kSpi1,
    kEe,
    kBattery,
    kUart6,
    kRcReceiver,
    kCrsfLink,
    kLed,
    kUart1,

    // SECONDARY DRIVERS
    kSpi2,
    kDshot,
    kEscTelemetry,
    kEscService,
    kButton,
    kUart2,
    kM10,
    kIcm42688p,
    kMultirotorMixer,
    kAhrs,
    kRateController,
    kAttitudeController,
  };

  void Init(const Config &config);

  LED &Led() { return LED::GetInstance(); }

  GPIO &Gpio() { return GPIO::GetInstance(); }
  TimeBase &Time() { return TimeBase::GetInstance(); }
  Watchdog &Wdg() { return Watchdog::GetInstance(); }
  Battery &Batt() { return Battery::GetInstance(); }
  Button &Btn() { return Button::GetInstance(); }
  Uart1 &FcUart() { return Uart1::GetInstance(); }
  // GPS UART
  Uart2 &GpsUart() { return Uart2::GetInstance(); }

  RcReceiver &RcRx() { return RcReceiver::GetInstance(); }
  CrsfLinkService &CrsfLinkSvc() { return crsf_link_service_; }
  EscService &EscSvc() { return esc_service_; }
  M10Service &GpsSvc() { return gps_service_; }
  Icm42688p &Imu() { return Icm42688p::GetInstance(); }
  multirotor_mixer::Mixer &MixerSvc() { return mixer_; }
  Ahrs &AhrsSvc() { return ahrs_; }
  RateController &RateControllerSvc() { return rate_controller_; }
  AttitudeController &AttitudeControllerSvc() { return attitude_controller_; }

  VehicleState &Vehicle() { return vehicle_state_; }
  FcLink &FcLinkSvc() { return FcLink::GetInstance(); }
  CommandHandler &GetCommandHandler() { return CommandHandler::GetInstance(); }
  StatPublisher &StatPubSvc() { return StatPublisher::GetInstance(); }

 private:
  void InitComponent(Component c);
  void ConfigureSystemClock(const Config &config);
  static void CoreInit();        // flash cache + NVIC group + SysTick
  static void EnablePwrClock();  // RCC_APB1ENR.PWREN
  static void SetVoltageScale(VoltageScale scale);
  static void InitOscillators(const OscillatorConfig &cfg);
  static void InitClockTree(const Config &cfg);
  bool initialized_ = false;
  M10Service gps_service_;
  VehicleState vehicle_state_;
  CrsfLinkService crsf_link_service_;
  EscService esc_service_;
  multirotor_mixer::Mixer mixer_;
  Ahrs ahrs_;
  RateController rate_controller_;
  AttitudeController attitude_controller_;

  // Empty by design — call Init() explicitly to bring up the chip.
  System() = default;
  ~System() = default;
  System(const System &) = delete;
  System &operator=(const System &) = delete;
};