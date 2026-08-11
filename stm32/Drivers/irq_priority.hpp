// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

#include "stm32f4xx.h"

// Centralized NVIC priorities for every interrupt the firmware enables.
// Cortex-M priority is "lower number = higher preemption priority."
// A priority here is a position in the tier order below rather than a chosen
// number: the ordering is the specification, so nothing can be given a value
// that contradicts where it sits, and symbols sharing a tier cannot drift
// apart. The IMU path is the only thing that can preempt PendSV and starts the
// chain every other deadline depends on; re-check the sample budget before
// reordering.

namespace irq_priority {

enum class Tier : uint32_t {
  kImuSpiDma,        // express lane: finish before the next sample arrives
  kImuInt,           // the EXTI that starts the transfer above
  kPendSv,           // express bottom-half, just under the IMU path
  kSensorTelemetry,  // ESC telemetry, GPS and RC receiver
  kDshotDma,         // motor frame DMA
  kSlowTick,         // releases one slow-loop pass, and kicks the DShot DMA
  kBackgroundLink,   // ESP32 link, fully background
  kUsb,              // ESC config CDC; host-paced
  kBootTick,         // coarse boot ms-tick; lowest
  kTierCount,
};

// Levels held free above the top tier for anything that must outrank the IMU
// path. A gap between tiers is not worth reserving -- deriving the numbers
// makes inserting one free -- but headroom above the top cannot be recovered
// by renumbering.
inline constexpr uint32_t kReservedTop = 2;

constexpr uint32_t Of(Tier tier) {
  return kReservedTop + static_cast<uint32_t>(tier);
}

// Express lane (sensor → controller → motors)
inline constexpr uint32_t kImuSpiDma =
    Of(Tier::kImuSpiDma);  // SPI2 RX/TX DMA (DMA1 Stream3/4)
inline constexpr uint32_t kImuInt = Of(Tier::kImuInt);  // EXTI line for kImuInt
inline constexpr uint32_t kPendSv = Of(Tier::kPendSv);

// Sensor / telemetry traffic
inline constexpr uint32_t kEscTelemetry =
    Of(Tier::kSensorTelemetry);  // USART3 + DMA1 Stream1
inline constexpr uint32_t kUart2 = Of(Tier::kSensorTelemetry);  // M10 GPS
inline constexpr uint32_t kUart6 = Of(Tier::kSensorTelemetry);  // RC receiver
inline constexpr uint32_t kUart2Dma =
    Of(Tier::kSensorTelemetry);  // DMA1 Stream5/6
inline constexpr uint32_t kUart6Dma =
    Of(Tier::kSensorTelemetry);  // DMA2 Stream1/6

// Motors
inline constexpr uint32_t kDshotTim1Dma = Of(Tier::kDshotDma);  // DMA2 Stream5

// Background
inline constexpr uint32_t kTimeBaseTim5 = Of(Tier::kSlowTick);
inline constexpr uint32_t kUart1 = Of(Tier::kBackgroundLink);  // FcLink USART1
inline constexpr uint32_t kUart1Dma =
    Of(Tier::kBackgroundLink);  // DMA2 Stream2/7
inline constexpr uint32_t kUsbOtgFs = Of(Tier::kUsb);
inline constexpr uint32_t kSysTick = Of(Tier::kBootTick);

// Written to AIRCR's PRIGROUP field, which splits the implemented bits between
// preemption and subpriority. Every bit has to go to preemption or the tiers
// above stop meaning anything: subpriority never preempts, it only orders
// what is already pending, so two tiers sharing a preempt group cannot
// interrupt each other however far apart their numbers are.
inline constexpr uint32_t kPriorityGrouping = 7u - __NVIC_PRIO_BITS;

// NVIC_SetPriority keeps only the implemented bits, so a tier list deeper than
// they can express would fold the lowest tiers into the ones above rather than
// fail.
static_assert(
    kReservedTop + static_cast<uint32_t>(Tier::kTierCount) <=
        (1u << __NVIC_PRIO_BITS),
    "more priority tiers than the implemented priority bits can express");

}  // namespace irq_priority
