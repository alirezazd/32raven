#pragma once

#include <cstdint>

// Centralized NVIC priorities for every interrupt the firmware enables.
//
// Cortex-M priority is "lower number = higher preemption priority." With the
// default STM32 HAL grouping (NVIC_PRIORITYGROUP_4) all four implemented bits
// are preempt — every IRQ here can interrupt every other lower-priority IRQ.
//
// Ordering invariant (must hold; nothing enforces it at compile time):
//
//   IMU SPI DMA (2)              ── express lane: must finish before next
//   sample IMU EXTI INT (3)             ── kicks the SPI DMA above PendSV (4)
//   ── express bottom-half right under the IMU path ESC TLM, USART2/6 + DMAs
//   (5) ── normal sensor / telemetry traffic DShot TIM1 DMA (6)           ──
//   motor frame DMA, kicked off by the slow tick TIM5 slow tick (7) ── 1 kHz
//   scheduler tick USART1 / FcLink + DMA (10)   ── ESP32 link, fully background
//
// Don't shuffle these without thinking through the sample budget — the IMU
// path is the only thing that can preempt PendSV and starts the chain that
// every other deadline depends on.

namespace irq_priority {

// ---- Express lane (sensor → controller → motors) ------------------------
inline constexpr uint32_t kImuSpiDma = 2;  // SPI2 RX/TX DMA (DMA1 Stream3/4)
inline constexpr uint32_t kImuInt = 3;     // EXTI line for kImuInt
inline constexpr uint32_t kPendSv = 4;     // express bottom-half

// ---- Sensor / telemetry traffic -----------------------------------------
inline constexpr uint32_t kEscTelemetry = 5;  // USART3 + DMA1 Stream1
inline constexpr uint32_t kUart2 = 5;         // M10 GPS
inline constexpr uint32_t kUart6 = 5;         // RC receiver
inline constexpr uint32_t kUart2Dma = 5;      // DMA1 Stream5/6
inline constexpr uint32_t kUart6Dma = 5;      // DMA2 Stream1/6

// ---- Motors -------------------------------------------------------------
inline constexpr uint32_t kDshotTim1Dma = 6;  // DMA2 Stream5

// ---- Background ---------------------------------------------------------
inline constexpr uint32_t kTimeBaseTim5 = 7;  // 1 kHz slow tick scheduler
inline constexpr uint32_t kUart1 = 10;        // FcLink USART1
inline constexpr uint32_t kUart1Dma = 10;     // DMA2 Stream2/7

}  // namespace irq_priority
