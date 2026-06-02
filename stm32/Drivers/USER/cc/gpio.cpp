#include "gpio.hpp"

#include "error_code.hpp"
#include "panic.hpp"

namespace {

// Map a GPIO_TypeDef* to the bit in RCC->AHB1ENR that gates its clock.
// Returns 0 (no bit) for unrecognized ports — callers should panic.
uint32_t PortClockBit(GPIO_TypeDef *port) {
  if (port == GPIOA) return RCC_AHB1ENR_GPIOAEN;
  if (port == GPIOB) return RCC_AHB1ENR_GPIOBEN;
  if (port == GPIOC) return RCC_AHB1ENR_GPIOCEN;
  if (port == GPIOD) return RCC_AHB1ENR_GPIODEN;
  if (port == GPIOE) return RCC_AHB1ENR_GPIOEEN;
  if (port == GPIOF) return RCC_AHB1ENR_GPIOFEN;
  if (port == GPIOG) return RCC_AHB1ENR_GPIOGEN;
  if (port == GPIOH) return RCC_AHB1ENR_GPIOHEN;
  return 0;
}

// SYSCFG_EXTICR port-source code (0 = PA, 1 = PB, ...). NVIC routing for an
// EXTI line goes: pin n -> SYSCFG->EXTICR[n / 4] selects the source port.
uint32_t ExtiPortSource(GPIO_TypeDef *port) {
  if (port == GPIOA) return 0;
  if (port == GPIOB) return 1;
  if (port == GPIOC) return 2;
  if (port == GPIOD) return 3;
  if (port == GPIOE) return 4;
  if (port == GPIOF) return 5;
  if (port == GPIOG) return 6;
  if (port == GPIOH) return 7;
  Panic(ErrorCode::Stm32::kGpioInvalidPort);
  return 0;
}

void EnablePortClock(GPIO_TypeDef *port) {
  const uint32_t bit = PortClockBit(port);
  if (bit == 0) {
    Panic(ErrorCode::Stm32::kGpioInvalidPort);
  }
  RCC->AHB1ENR |= bit;
  (void)RCC->AHB1ENR;  // readback delay (errata: 2-cycle settling on AHB)
}

void EnableSyscfgClock() {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  (void)RCC->APB2ENR;
}

bool IsOutputMode(uint32_t mode) {
  return mode == GPIO_MODE_OUTPUT_PP || mode == GPIO_MODE_OUTPUT_OD;
}

// Decode the HAL-encoded GPIO_MODE_* macro into its component fields.
struct ModeBits {
  uint32_t base;    // 0..3 — GPIO MODER value (input/output/AF/analog)
  uint32_t output;  // 0=PP, 1=OD
  bool exti;        // true for IT_*/EVT_* modes
  bool rising;
  bool falling;
};

ModeBits DecodeMode(uint32_t mode) {
  return {
      .base = mode & 0x3u,
      .output = (mode >> 4u) & 0x1u,
      .exti = (mode & (0x3u << 16u)) != 0u,
      .rising = (mode & (0x1u << 20u)) != 0u,
      .falling = (mode & (0x2u << 20u)) != 0u,
  };
}

void ProgramPin(GPIO_TypeDef *port, uint8_t pin_index, const GpioInit &init) {
  const ModeBits mb = DecodeMode(init.Mode);
  const uint32_t shift2 = static_cast<uint32_t>(pin_index) * 2u;

  if (mb.exti) {
    // Pin acts as an EXTI input: GPIO mode stays INPUT, plus SYSCFG/EXTI
    // wiring. Output type/speed don't apply.
    EnableSyscfgClock();
    const uint32_t cr_idx = pin_index / 4u;
    const uint32_t cr_shift = static_cast<uint32_t>(pin_index % 4u) * 4u;
    SYSCFG->EXTICR[cr_idx] = (SYSCFG->EXTICR[cr_idx] & ~(0xFu << cr_shift)) |
                             (ExtiPortSource(port) << cr_shift);

    const uint32_t pin_mask = 1u << pin_index;
    if (mb.rising) {
      EXTI->RTSR |= pin_mask;
    } else {
      EXTI->RTSR &= ~pin_mask;
    }
    if (mb.falling) {
      EXTI->FTSR |= pin_mask;
    } else {
      EXTI->FTSR &= ~pin_mask;
    }
    EXTI->IMR |= pin_mask;

    port->MODER = (port->MODER & ~(0x3u << shift2));  // input mode (00)
    port->PUPDR =
        (port->PUPDR & ~(0x3u << shift2)) | ((init.Pull & 0x3u) << shift2);
    return;
  }

  // Non-EXTI: program MODER, OTYPER (outputs/AF), OSPEEDR (outputs/AF),
  // PUPDR (everything), AFR (AF only).
  port->MODER = (port->MODER & ~(0x3u << shift2)) | (mb.base << shift2);

  if (mb.base == 0x1u || mb.base == 0x2u) {  // OUTPUT or AF
    const uint32_t shift1 = pin_index;
    port->OTYPER = (port->OTYPER & ~(0x1u << shift1)) | (mb.output << shift1);
    port->OSPEEDR =
        (port->OSPEEDR & ~(0x3u << shift2)) | ((init.Speed & 0x3u) << shift2);
  }

  port->PUPDR =
      (port->PUPDR & ~(0x3u << shift2)) | ((init.Pull & 0x3u) << shift2);

  if (mb.base == 0x2u) {  // AF
    const uint32_t af_idx = pin_index / 8u;
    const uint32_t af_shift = static_cast<uint32_t>(pin_index % 8u) * 4u;
    port->AFR[af_idx] = (port->AFR[af_idx] & ~(0xFu << af_shift)) |
                        ((init.Alternate & 0xFu) << af_shift);
  }
}

void ApplyConfig(GPIO_TypeDef *port, const GpioInit &init) {
  for (uint8_t i = 0; i < 16u; ++i) {
    if ((init.Pin & (1u << i)) != 0u) {
      ProgramPin(port, i, init);
    }
  }
}

}  // namespace

void GPIO::Init(const PinConfig *pins, size_t pin_count) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kGpioReinit);
  }
  initialized_ = true;
  for (size_t i = 0; i < pin_count; i++) {
    EnablePortClock(pins[i].port);
  }

  // Drive safe initial output levels BEFORE flipping the pin to output —
  // prevents glitches during the input -> output transition.
  for (size_t i = 0; i < pin_count; i++) {
    const auto &pc = pins[i];
    if (!IsOutputMode(pc.init.Mode)) continue;
    const bool initial_level = pc.active_low;
    WritePin(pc.port, pc.init.Pin, initial_level);
  }
  for (size_t i = 0; i < pin_count; i++) {
    ApplyConfig(pins[i].port, pins[i].init);
  }
}

void GPIO::WritePin(GPIO_TypeDef *port, uint16_t pin, bool state) {
  if (state) {
    port->BSRR = pin;
  } else {
    port->BSRR = (uint32_t)pin << 16U;
  }
}

bool GPIO::ReadPin(GPIO_TypeDef *port, uint16_t pin) {
  return (port->IDR & pin) != 0;
}
