// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

struct TopicConfig {
  // Zero disables the topic outright -- it is never due and never selected.
  uint32_t period = 0;
  // Longest a caller may keep suppressing an unchanged payload. Zero means it
  // may suppress forever.
  uint32_t max_silence = 0;
  uint8_t priority = 0;
};

struct TopicState {
  uint32_t next_due = 0;
  uint32_t last_sent = 0;
};

// Which topic goes next on a link that emits whole frames per poll.
//
// The scheduler never sees a payload, so the links it serves need not agree on
// a frame size and change detection stays with whoever owns the data --
// `SilenceExpired` is the hook a suppressing caller asks when it has to stop.
//
// Times are plain counts compared wrap-safely, so the unit is the caller's:
// microseconds on the STM32 links, milliseconds on the ESP32. One unit per
// scheduler, across the config, the stagger and every timestamp passed in.
//
// The spans are not owned; build them from static storage, not from locals.
class TopicScheduler {
 public:
  // `stagger` spaces the topics so they do not come due together; pick it with
  // slot_stagger::Pick against the same periods.
  void Init(std::span<const TopicConfig> config, std::span<TopicState> state,
            uint32_t stagger, uint32_t now) {
    config_ = config;
    state_ = state;
    for (size_t i = 0; i < state_.size(); ++i) {
      state_[i] = TopicState{
          .next_due = now + (static_cast<uint32_t>(i) * stagger),
          .last_sent = 0,
      };
    }
  }

  bool IsDue(size_t topic, uint32_t now) const {
    return config_[topic].period != 0u &&
           static_cast<int32_t>(now - state_[topic].next_due) >= 0;
  }

  // Highest priority among the due topics, most overdue breaking the tie.
  // Priority wins outright rather than by weight, so it orders a burst without
  // guaranteeing service: a topic starves for as long as a higher-priority one
  // keeps coming due. Equal priorities throughout give pure round-robin.
  std::optional<size_t> NextDue(uint32_t now) const {
    std::optional<size_t> best;
    uint8_t best_priority = 0;
    uint32_t best_overdue = 0;

    for (size_t i = 0; i < config_.size(); ++i) {
      if (!IsDue(i, now)) {
        continue;
      }

      const uint32_t overdue = now - state_[i].next_due;
      if (!best || config_[i].priority > best_priority ||
          (config_[i].priority == best_priority && overdue > best_overdue)) {
        best = i;
        best_priority = config_[i].priority;
        best_overdue = overdue;
      }
    }
    return best;
  }

  // The caller's cue to send an unchanged payload anyway. True before the
  // first send, so a topic cannot start out suppressed.
  bool SilenceExpired(size_t topic, uint32_t now) const {
    if (config_[topic].max_silence == 0u) {
      return false;
    }
    return state_[topic].last_sent == 0u ||
           static_cast<uint32_t>(now - state_[topic].last_sent) >=
               config_[topic].max_silence;
  }

  void MarkSent(size_t topic, uint32_t now) {
    state_[topic].last_sent = now;
    Advance(topic, now);
  }

  // The topic had its turn and declined it -- no data, or nothing changed.
  void Skip(size_t topic, uint32_t now) { Advance(topic, now); }

 private:
  // Advancing the deadline rather than measuring from the send holds the rate
  // at exactly the period, and holds the stagger in the phase
  // slot_stagger::Pick proved collision-free. The resync covers what that
  // breaks: a topic unserviced for longer than its period -- transport down,
  // source suspended -- would return owing a burst of catch-up frames, and
  // losing the phase is the cheaper of the two.
  void Advance(size_t topic, uint32_t now) {
    const uint32_t period = config_[topic].period;
    state_[topic].next_due += period;
    if (static_cast<int32_t>(now - state_[topic].next_due) >= 0) {
      state_[topic].next_due = now + period;
    }
  }

  std::span<const TopicConfig> config_{};
  std::span<TopicState> state_{};
};
