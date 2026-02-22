/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "board.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_tim1_up;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern void Uart1DmaTxComplete(void);
extern void Icm42688pOnIrq(void);
extern void Uart2DmaTxComplete(void);
extern void Uart1DmaError(uint32_t);
extern void Uart2DmaError(uint32_t);
extern void Uart1RxDmaError(uint32_t);
extern void Uart2RxDmaError(uint32_t);
extern void TimeBaseOnPpsIrq(uint32_t capture_val);
extern void TimeBaseOnTim5Irq(void);

extern void Uart1OnUartInterrupt(void);
extern void Uart2OnUartInterrupt(void);
extern void Uart1OnRxHalfCplt(void);
extern void Uart1OnRxCplt(void);
extern void Uart2OnRxHalfCplt(void);
extern void Uart2OnRxCplt(void);

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler(void) {
  // USART2 RX: DMA1 Stream5 (Stream 5 is High 4-7)
  const uint32_t hisr = DMA1->HISR;

  // Check for Transfer Error, Direct Mode Error, or FIFO Error
  if (hisr & (DMA_HISR_TEIF5 | DMA_HISR_DMEIF5 | DMA_HISR_FEIF5)) {
    // Clear ALL flags for this stream
    DMA1->HIFCR = DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5 |
                  DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5;
    Uart2RxDmaError(hisr);
    return;
  }

  if (hisr & (DMA_HISR_HTIF5 | DMA_HISR_TCIF5)) {
    // Clear HT/TC flags
    DMA1->HIFCR = DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5;

    if (hisr & DMA_HISR_HTIF5) {
      Uart2OnRxHalfCplt();
    }
    if (hisr & DMA_HISR_TCIF5) {
      Uart2OnRxCplt();
    }
  }
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void) { Uart1OnUartInterrupt(); }

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void) { Uart2OnUartInterrupt(); }

/**
 * @brief This function handles DMA1 stream6 global interrupt.
 */
void DMA1_Stream6_IRQHandler(void) {
  // USART2 TX: DMA1 Stream6 (Stream 6 is High 4-7)
  const uint32_t hisr = DMA1->HISR;

  // Check errors first
  if (hisr & (DMA_HISR_TEIF6 | DMA_HISR_DMEIF6)) {
    DMA1->HIFCR = DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6 |
                  DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;
    // Disable stream on error
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream6->CR & DMA_SxCR_EN) {
    }

    Uart2DmaError(hisr);
    return;
  }

  if (hisr & DMA_HISR_TCIF6) {
    DMA1->HIFCR = DMA_HIFCR_CTCIF6;
    Uart2DmaTxComplete();
  }
}

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & (1u << 10)) {
    EXTI->PR = (1u << 10);
    Icm42688pOnIrq();
  }
}

extern void Spi1RxDmaComplete(void);
extern void Spi1DmaError(uint32_t isr);

/**
 * @brief This function handles DMA2 stream0 global interrupt.
 */
void DMA2_Stream0_IRQHandler(void) {
  // RX Stream (Stream 0 is Low 0-3)
  const uint32_t lisr = DMA2->LISR;

  // Check errors
  if (lisr & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) {
    // Clear flags
    DMA2->LIFCR = DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 |
                  DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
    Spi1DmaError(lisr);
    return;
  }

  // Check TC (Transfer Complete)
  if (lisr & DMA_LISR_TCIF0) {
    // Hardening: Clear ALL flags for Stream 0 AND Stream 3 (opportunistic
    // cleanup)
    DMA2->LIFCR = (DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 |
                   DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0) |
                  (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3 |
                   DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3);
    Spi1RxDmaComplete();
  }
}

/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
void DMA2_Stream2_IRQHandler(void) {
  // USART1 RX: DMA2 Stream2 (Stream 2 is Low 0-3)
  const uint32_t lisr = DMA2->LISR;

  // Check for Transfer Error, Direct Mode Error, or FIFO Error
  if (lisr & (DMA_LISR_TEIF2 | DMA_LISR_DMEIF2 | DMA_LISR_FEIF2)) {
    // Clear ALL flags for this stream
    DMA2->LIFCR = DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2 |
                  DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;
    Uart1RxDmaError(lisr);
    return;
  }

  if (lisr & (DMA_LISR_HTIF2 | DMA_LISR_TCIF2)) {
    // Clear HT/TC flags
    DMA2->LIFCR = DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;

    if (lisr & DMA_LISR_HTIF2) {
      Uart1OnRxHalfCplt();
    }
    if (lisr & DMA_LISR_TCIF2) {
      Uart1OnRxCplt();
    }
  }
}

/**
 * @brief This function handles DMA2 stream3 global interrupt.
 */
void DMA2_Stream3_IRQHandler(void) {
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  // TX Stream (Stream 3 is Low 0-3)
  const uint32_t lisr = DMA2->LISR;

  // Hardening: Always clear any pending TX flags (cheap hygiene)
  DMA2->LIFCR = DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3 |
                DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3;

  // Check errors
  if (lisr & (DMA_LISR_TEIF3 | DMA_LISR_DMEIF3 | DMA_LISR_FEIF3)) {
    Spi1DmaError(lisr);
  }
  // We don't care about TC on TX for SPI Full Duplex (relies on RX TC)

  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream5 global interrupt.
 */
void DMA2_Stream5_IRQHandler(void) {
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_up);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream7 global interrupt.
 */
void DMA2_Stream7_IRQHandler(void) {
  // USART1 TX: DMA2 Stream7 => High 4-7
  const uint32_t hisr = DMA2->HISR;

  if (hisr & (DMA_HISR_TEIF7 | DMA_HISR_DMEIF7)) {
    DMA2->HIFCR = DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7 |
                  DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
    // Disable stream on error
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream7->CR & DMA_SxCR_EN) {
    }

    Uart1DmaError(hisr);
    return;
  }

  if (hisr & DMA_HISR_TCIF7) {
    DMA2->HIFCR = DMA_HIFCR_CTCIF7;
    Uart1DmaTxComplete();
  }
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void) {
  // Check for Capture/Compare 1 interrupt
  if (TIM2->SR & TIM_SR_CC1IF) {
    // Clear CC1 flag
    TIM2->SR = (uint16_t)~TIM_SR_CC1IF;

    // Read captured value and apply compensation
    extern uint32_t g_pps_compensation;
    uint32_t capture_val = TIM2->CCR1 - g_pps_compensation;

    // Call PPS handler
    TimeBaseOnPpsIrq(capture_val);
  }
}

/**
 * @brief This function handles TIM5 global interrupt.
 */
void TIM5_IRQHandler(void) {
  if (TIM5->SR & TIM_SR_UIF) {
    TIM5->SR = (uint16_t)~TIM_SR_UIF;
    TimeBaseOnTim5Irq();
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
