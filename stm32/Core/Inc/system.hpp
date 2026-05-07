#pragma once

#include "battery.hpp"
#include "button.hpp"
#include "command_handler.hpp"
#include "crsf_link_service.hpp"
#include "ee.hpp"
#include "esc_service.hpp"
#include "esc_telemetry.hpp"
#include "fc_link.hpp"
#include "gpio.hpp"
#include "icm42688p.hpp"
#include "led.hpp"
#include "m10.hpp"
#include "m10_service.hpp"
#include "rc_receiver.hpp"
#include "spi.hpp"
#include "stat_publisher.hpp"
#include "time_base.hpp"
#include "uart.hpp"
#include "vehicle_state.hpp"

// Clock + power configuration enum classes used by `System::Config` below.
// Each value's underlying integer is the actual register-bit pattern the
// peripheral expects (CMSIS-level — no HAL wrappers), so
// `static_cast<uint32_t>(value)` produces the register write directly.
// PllP is special: HAL_RCC_OscConfig does its own integer-to-bitfield
// mapping for it, so the values here are the human-readable divisors.
// Oscillator is also special — it influences several HAL fields (HSE/HSI
// on/off, PLL source), so system.cpp branches on it explicitly.
namespace system_clock {

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

// PPRE1 bit positions — APB2 reuses these and HAL_RCC_ClockConfig shifts
// them by 3 internally to land in the PPRE2 field of RCC_CFGR. So a single
// ApbDiv enum works for both buses.
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

}  // namespace system_clock

class System {
 public:
  // Clock + power configuration consumed by Init(). The enum values are
  // already register-bit-aligned, so system.cpp can `static_cast` straight
  // into the HAL init structs. Cross-field constraints (PLL VCO range, max
  // APB1/APB2 freqs, flash latency vs SYSCLK, voltage scale vs target
  // clock) are documented in the reference manual; the build does not
  // police them.
  struct Config {
    system_clock::Oscillator oscillator;
    uint8_t pllm;   // 2..63
    uint16_t plln;  // 50..432
    system_clock::PllP pllp;
    uint8_t pllq;  // 2..15
    system_clock::AhbDiv ahb_divider;
    system_clock::ApbDiv apb1_divider;
    system_clock::ApbDiv apb2_divider;
    uint8_t flash_latency;
    system_clock::VoltageScale voltage_scale;
  };

  static System &GetInstance() {
    static System instance;
    return instance;
  }

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
    kIcm42688p
  };

  void Init(const Config &config);

  LED &Led() { return LED::GetInstance(); }

  Spi1 &GetSpi1() { return Spi1::GetInstance(); }
  Spi2 &GetSpi() { return Spi2::GetInstance(); }
  GPIO &Gpio() { return GPIO::GetInstance(); }
  TimeBase &Time() { return TimeBase::GetInstance(); }
  EE &GetEe() { return EE::GetInstance(); }
  Battery &GetBattery() { return Battery::GetInstance(); }
  Button &Btn() { return Button::GetInstance(); }
  Uart1 &GetUart() { return Uart1::GetInstance(); }
  // Alias for clarity (Console)
  Uart1 &GetUart1() { return Uart1::GetInstance(); }
  // GPS UART
  Uart2 &GetUart2() { return Uart2::GetInstance(); }
  Uart6 &GetUart6() { return Uart6::GetInstance(); }

  M10 &GetGps() { return M10::GetInstance(); }
  RcReceiver &GetRcReceiver() { return RcReceiver::GetInstance(); }
  CrsfLinkService &GetCrsfLinkService() { return crsf_link_service_; }
  EscService &GetEscService() { return esc_service_; }
  EscTelemetry &GetEscTelemetry() { return EscTelemetry::GetInstance(); }
  M10Service &ServiceGps() { return gps_service_; }
  Icm42688p &GetImu42688p() { return Icm42688p::GetInstance(); }

  VehicleState &GetVehicleState() { return vehicle_state_; }
  FcLink &GetFcLink() { return FcLink::GetInstance(); }
  CommandHandler &GetCommandHandler() { return CommandHandler::GetInstance(); }
  StatPublisher &GetStatPublisher() { return StatPublisher::GetInstance(); }

 private:
  void InitComponent(Component c);

  bool initialized_ = false;
  M10Service gps_service_;
  VehicleState vehicle_state_;
  CrsfLinkService crsf_link_service_;
  EscService esc_service_;

  System();
  ~System() {}
  System(const System &) = delete;
  System &operator=(const System &) = delete;
  void ConfigureSystemClock(const Config &config);
};

#define MICROS() (System::GetInstance().Time().Micros())
