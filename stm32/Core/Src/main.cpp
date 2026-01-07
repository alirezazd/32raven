#include "DShotCodec.hpp"
#include "DShotTim1.hpp"
#include "LED.hpp"
#include "TimeBase.hpp"
#include "UserConfig.hpp"
#include "system.hpp"
#include <cstdint>

extern "C" volatile uint32_t g_dma5_irq_seen;

static inline void BusyWaitUs(uint32_t us) {
  const uint32_t kStart = MICROS();
  while ((uint32_t)(MICROS() - kStart) < us) {
  }
}

int main(void) {
  System::GetInstance().Init(kSystemDefault);

  while (1) {
    System::GetInstance().Led().Toggle();
    BusyWaitUs(900000);
  }
}