// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include <cstddef>
#include <cstdint>
#include <span>

// First-frame spacing for round-robin schedulers that emit one frame per tick.
// Slot i opens at i * stagger and repeats on its own period, so two slots come
// due together at some point exactly when the gcd of their periods divides the
// gap between their offsets. Where rescheduling advances by period rather than
// from now, that shared tick then recurs for the life of the link.
namespace slot_stagger {
namespace detail {

constexpr std::uint32_t Gcd(std::uint32_t a, std::uint32_t b) {
  while (b != 0) {
    const std::uint32_t remainder = a % b;
    a = b;
    b = remainder;
  }
  return a;
}

// Every pair, not just each slot against the origin: two slots can share a
// tick with each other while both clear slot 0. A zero period marks a disabled
// stream, which is never due and so never shares anything.
constexpr bool Clears(std::uint32_t stagger,
                      std::span<const std::uint32_t> periods) {
  for (std::size_t i = 0; i < periods.size(); ++i) {
    if (periods[i] == 0) {
      continue;
    }
    for (std::size_t j = i + 1; j < periods.size(); ++j) {
      if (periods[j] == 0) {
        continue;
      }
      const std::uint64_t gap = static_cast<std::uint64_t>(j - i) * stagger;
      if (gap % Gcd(periods[i], periods[j]) == 0) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace detail

// The tuned spacing when no two slots ever come due together, else the next
// value up that manages it. Zero when nothing below twice the target works,
// which callers turn into a build error rather than shipping a ladder that
// collides forever.
constexpr std::uint32_t Pick(std::uint32_t target,
                             std::span<const std::uint32_t> periods) {
  const std::uint64_t limit = static_cast<std::uint64_t>(target) * 2u;
  for (std::uint64_t stagger = target; stagger < limit; ++stagger) {
    if (detail::Clears(static_cast<std::uint32_t>(stagger), periods)) {
      return static_cast<std::uint32_t>(stagger);
    }
  }
  return 0u;
}

}  // namespace slot_stagger
