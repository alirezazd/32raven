#include "DShotCodec.hpp"
#include "DShotTim1.hpp"
#include "LED.hpp"
#include "TimeBase.hpp"
#include "UserConfig.hpp"
#include "system.hpp"
#include <cstdint>

extern "C" volatile uint32_t g_dma5_irq_seen;

static inline void busy_wait_us(uint32_t us) {
  const uint32_t start = TimeBase::micros();
  while ((uint32_t)(TimeBase::micros() - start) < us) {
  }
}

int main(void) {
  System::init(SYSTEM_DEFAULT);

  while (1) {
    LED::toggle();
    busy_wait_us(900000);
  }
}