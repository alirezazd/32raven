#ifndef USER_DMA_HPP
#define USER_DMA_HPP

#include "stm32f4xx_hal.h"

// Global DMA Handles (for ISR linkage)
extern "C" {
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_tim1_up;
}

class Dma {
public:
  static Dma &GetInstance() {
    static Dma instance;
    return instance;
  }

  void Init();

private:
  Dma() = default;
};

#endif // USER_DMA_HPP
