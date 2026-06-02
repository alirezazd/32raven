#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "panic.hpp"
#include "stm32f4xx.h"

// Register-level GPIO configuration vocabulary — the small slice of the STM32
// HAL GPIO header this firmware used, reimplemented here with values that are
// bit-for-bit identical to HAL's so GPIO::ProgramPin's decoding (in gpio.cpp)
// is unchanged. The HAL spelling of the names is kept so the generated
// stm32_config.hpp can keep using them. Adding a peripheral pin on a new
// alternate function? Add its GPIO_AFx_* constant (value = the AF index x).

// Pin configuration record (was HAL's GPIO_InitTypeDef). Field names and order
// are preserved so the generated config's designated initializers still apply.
//
// The names below deliberately keep HAL's GPIO_* spelling because the config
// generator emits them verbatim, so they don't follow the project's kCamelCase
// rule for constants.
// NOLINTBEGIN(readability-identifier-naming)
struct GpioInit {
  uint32_t Pin;
  uint32_t Mode;
  uint32_t Pull;
  uint32_t Speed;
  uint32_t Alternate;
};

// --- Pin bitmask (GPIO_pins_define) --------------------------------------
inline constexpr uint32_t GPIO_PIN_0 = 0x0001u;
inline constexpr uint32_t GPIO_PIN_1 = 0x0002u;
inline constexpr uint32_t GPIO_PIN_2 = 0x0004u;
inline constexpr uint32_t GPIO_PIN_3 = 0x0008u;
inline constexpr uint32_t GPIO_PIN_4 = 0x0010u;
inline constexpr uint32_t GPIO_PIN_5 = 0x0020u;
inline constexpr uint32_t GPIO_PIN_6 = 0x0040u;
inline constexpr uint32_t GPIO_PIN_7 = 0x0080u;
inline constexpr uint32_t GPIO_PIN_8 = 0x0100u;
inline constexpr uint32_t GPIO_PIN_9 = 0x0200u;
inline constexpr uint32_t GPIO_PIN_10 = 0x0400u;
inline constexpr uint32_t GPIO_PIN_11 = 0x0800u;
inline constexpr uint32_t GPIO_PIN_12 = 0x1000u;
inline constexpr uint32_t GPIO_PIN_13 = 0x2000u;
inline constexpr uint32_t GPIO_PIN_14 = 0x4000u;
inline constexpr uint32_t GPIO_PIN_15 = 0x8000u;

// --- Mode (GPIO_mode_define) ---------------------------------------------
// Encoding 0x00WX00YZ: W = EXTI trigger (bit20+), X = EXTI mode (bit16+),
// Y = output type (bit4: 0=PP, 1=OD), Z = base mode (bits1:0). gpio.cpp's
// DecodeMode() relies on exactly this layout.
inline constexpr uint32_t GPIO_MODE_INPUT = 0x00000000u;
inline constexpr uint32_t GPIO_MODE_OUTPUT_PP = 0x00000001u;
inline constexpr uint32_t GPIO_MODE_OUTPUT_OD = 0x00000011u;
inline constexpr uint32_t GPIO_MODE_AF_PP = 0x00000002u;
inline constexpr uint32_t GPIO_MODE_AF_OD = 0x00000012u;
inline constexpr uint32_t GPIO_MODE_ANALOG = 0x00000003u;
inline constexpr uint32_t GPIO_MODE_IT_RISING =
    0x00110000u;  // MODE_INPUT | EXTI_IT | TRIGGER_RISING

// --- Output speed (GPIO_speed_define) ------------------------------------
inline constexpr uint32_t GPIO_SPEED_FREQ_LOW = 0x00000000u;
inline constexpr uint32_t GPIO_SPEED_FREQ_MEDIUM = 0x00000001u;
inline constexpr uint32_t GPIO_SPEED_FREQ_HIGH = 0x00000002u;
inline constexpr uint32_t GPIO_SPEED_FREQ_VERY_HIGH = 0x00000003u;

// --- Pull resistors (GPIO_pull_define) -----------------------------------
inline constexpr uint32_t GPIO_NOPULL = 0x00000000u;
inline constexpr uint32_t GPIO_PULLUP = 0x00000001u;
inline constexpr uint32_t GPIO_PULLDOWN = 0x00000002u;

// --- Alternate functions (value = AF index) ------------------------------
inline constexpr uint32_t GPIO_AF1_TIM1 = 0x01u;
inline constexpr uint32_t GPIO_AF5_SPI1 = 0x05u;
inline constexpr uint32_t GPIO_AF5_SPI2 = 0x05u;
inline constexpr uint32_t GPIO_AF7_USART2 = 0x07u;
inline constexpr uint32_t GPIO_AF7_USART3 = 0x07u;
// NOLINTEND(readability-identifier-naming)

class GPIO {
 public:
  struct PinConfig {
    GPIO_TypeDef *port;
    GpioInit init;
    bool active_low;
  };

  void WritePin(GPIO_TypeDef *port, uint16_t pin, bool state);
  bool ReadPin(GPIO_TypeDef *port, uint16_t pin);

 private:
  friend class System;
  template <size_t N>
  void Init(const std::array<PinConfig, N> &pins) {
    Init(pins.data(), N);
  }

  void Init(const PinConfig *pins, size_t count);

  static GPIO &GetInstance() {
    static GPIO instance;
    return instance;
  }

  GPIO() = default;
  ~GPIO() = default;
  GPIO(const GPIO &) = delete;
  GPIO &operator=(const GPIO &) = delete;

  bool initialized_ = false;
};
