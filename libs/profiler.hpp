// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

// Wall-clock cost of named code sections, sampled over a window.
//
// Disabled, this compiles to nothing: Measure() becomes the bare call it
// wraps, and the storage is an empty struct. That is what makes it safe to
// leave at a call site permanently rather than adding and deleting scaffolding
// each time a question comes up.
//
// It never logs. Stats come out through Get()/Worst() and the caller decides
// what to do with them, so the profiler stays free of any transport, format
// or allocation dependency.
//
// Wall clock, not CPU time: a section preempted by an interrupt is charged for
// the interrupt. That is deliberate -- for a section competing with a control
// loop, the elapsed time is the thing that matters.
//
// `Id` is an enum class whose last enumerator is kCount. `Clock` supplies a
// static Micros(); the unit is only ever the caller's, so a millisecond clock
// works as long as everything downstream reads the numbers that way.
template <typename Id, typename Clock, bool kEnabled>
class Profiler {
 public:
  static constexpr size_t kCount = static_cast<size_t>(Id::kCount);

  struct Stat {
    uint32_t calls = 0;
    // Held separately because either alone misleads: a max hides the ordinary
    // case behind one outlier, and a mean hides the outlier that blew a
    // deadline. Mean is total / calls.
    uint32_t total_us = 0;
    uint32_t min_us = 0xFFFFFFFFu;
    uint32_t max_us = 0;
  };

  // Time `body` against `id`. Disabled, this is exactly `body()`.
  template <typename Fn>
  void Measure(Id id, Fn &&body) {
    if constexpr (!kEnabled) {
      std::forward<Fn>(body)();
    } else {
      const uint32_t start_us = Clock::Micros();
      std::forward<Fn>(body)();
      Record(id, Clock::Micros() - start_us);
    }
  }

  // Gap since the previous call for this id -- the cadence a periodic caller
  // actually achieved, which its own duration cannot show. The first call
  // establishes the reference and records nothing.
  void Interval(Id id) {
    if constexpr (kEnabled) {
      const size_t slot = static_cast<size_t>(id);
      const uint32_t now_us = Clock::Micros();
      if (last_seen_us_[slot] != 0u) {
        Record(id, now_us - last_seen_us_[slot]);
      }
      last_seen_us_[slot] = now_us;
    }
  }

  // For a cost the caller already timed for its own reasons.
  void Record(Id id, uint32_t elapsed_us) {
    if constexpr (kEnabled) {
      Stat &stat = stats_[static_cast<size_t>(id)];
      stat.calls++;
      stat.total_us += elapsed_us;
      if (elapsed_us < stat.min_us) {
        stat.min_us = elapsed_us;
      }
      if (elapsed_us > stat.max_us) {
        stat.max_us = elapsed_us;
      }
    }
  }

  const Stat &Get(Id id) const { return stats_[static_cast<size_t>(id)]; }

  // The id holding the largest max this window, so a report can name the worst
  // section without the caller ranking them itself. kCount when nothing ran.
  Id Worst() const {
    if constexpr (!kEnabled) {
      return Id::kCount;
    } else {
      size_t worst = kCount;
      uint32_t worst_us = 0;
      for (size_t i = 0; i < kCount; ++i) {
        if (stats_[i].calls != 0u && stats_[i].max_us >= worst_us) {
          worst_us = stats_[i].max_us;
          worst = i;
        }
      }
      return static_cast<Id>(worst);
    }
  }

  // Starts the next window. Interval references survive it, so a reset does
  // not fabricate one huge gap out of the first call after it.
  void ResetWindow() {
    if constexpr (kEnabled) {
      stats_.fill(Stat{});
    }
  }

 private:
  // Disabled, these are empty and the whole object costs nothing.
  struct Empty {};
  [[no_unique_address]] std::conditional_t<kEnabled, std::array<Stat, kCount>,
                                           Empty> stats_{};
  [[no_unique_address]] std::conditional_t<
      kEnabled, std::array<uint32_t, kCount>, Empty> last_seen_us_{};
};
