// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "tone_player.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"
#include "rtttl/tone_scores.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
}

namespace {
constexpr const char *kTag = "tone_player";
constexpr uint32_t kMinBpm = 1;
constexpr uint32_t kDefaultWholeNoteMs = 1000;

const char *BuiltinToneToRtttl(TonePlayer::BuiltinTone tone) {
  const auto index = static_cast<std::size_t>(tone);
  return (index < std::size(tone_scores::kAll)) ? tone_scores::kAll[index]
                                                : nullptr;
}
}  // namespace

TonePlayer &TonePlayer::GetInstance() {
  static TonePlayer instance;
  return instance;
}

void TonePlayer::Init(const Config &cfg, Buzzer *buzzer) {
  static constexpr uint32_t kTaskStackBytes = 3072;
  static StaticQueue_t request_queue_buffer;
  static uint8_t
      request_queue_storage[kPendingRequestQueueDepth * sizeof(PendingRequest)];
  static StaticTask_t task_buffer;
  static StackType_t task_stack[kTaskStackBytes];
  cfg_ = cfg;
  buzzer_ = buzzer;
  if (buzzer_ == nullptr) {
    Panic(ErrorCode::Esp32::kTonePlayerInitFailed);
  }

  playback_volume_ = cfg_.volume;
  buzzer_->SetDutyCycle(DutyCycleForVolume(playback_volume_));
  buzzer_->Stop();
  request_queue_ =
      xQueueCreateStatic(kPendingRequestQueueDepth, sizeof(PendingRequest),
                         request_queue_storage, &request_queue_buffer);
  if (request_queue_ == nullptr) {
    Panic(ErrorCode::Esp32::kTonePlayerInitFailed);
  }
  task_handle_ = xTaskCreateStatic(TaskEntry, "tone_player", kTaskStackBytes,
                                   this, 1, task_stack, &task_buffer);
  if (task_handle_ == nullptr) {
    Panic(ErrorCode::Esp32::kTonePlayerInitFailed);
  }
  ESP_LOGI(kTag, "initialized");
}

bool TonePlayer::PlayRtttl(const char *rtttl, int volume) {
  if (rtttl == nullptr || *rtttl == '\0' || request_queue_ == nullptr) {
    return false;
  }

  const PendingRequest request = {.score = rtttl, .volume = volume};
  return xQueueSend((QueueHandle_t)request_queue_, &request, 0) == pdPASS;
}

void TonePlayer::PlayBuiltin(BuiltinTone tone, int volume) {
  const char *rtttl = BuiltinToneToRtttl(tone);
  if (rtttl == nullptr) {
    return;
  }
  (void)PlayRtttl(rtttl, volume);
}

void TonePlayer::PlayBuiltinNow(BuiltinTone tone, int volume) {
  const char *rtttl = BuiltinToneToRtttl(tone);
  if (rtttl == nullptr || request_queue_ == nullptr) {
    return;
  }
  const PendingRequest request = {.score = rtttl, .volume = volume};
  preempt_ = true;
  xQueueReset((QueueHandle_t)request_queue_);
  (void)xQueueSend((QueueHandle_t)request_queue_, &request, 0);
  // The player sleeps out the current note, so the queue alone would not
  // be seen until it ends.
  if (task_handle_ != nullptr) {
    xTaskNotifyGive((TaskHandle_t)task_handle_);
  }
}

void TonePlayer::StopPlayback() {
  playing_ = false;
  score_ = nullptr;
  cursor_ = nullptr;
  next_change_ms_ = 0;
  buzzer_->Stop();
}

void TonePlayer::TaskEntry(void *param) {
  static_cast<TonePlayer *>(param)->Task();
}

void TonePlayer::Task() {
  while (true) {
    if (playing_) {
      if (preempt_) {
        preempt_ = false;
        StopPlayback();
        continue;
      }

      const TimeMs now = Sys().Timebase().NowMs();
      if (TimeReached(now, next_change_ms_)) {
        const std::optional<NoteEvent> event = ParseNextNote();
        if (!event) {
          StopPlayback();
        } else {
          StartEvent(*event, now);
        }
        continue;
      }

      // Woken early by a preempt; otherwise this sleeps out the note.
      const TimeMs wait_ms = next_change_ms_ - now;
      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms > 0 ? wait_ms : 1));
      continue;
    }

    PendingRequest request{};
    if (xQueueReceive((QueueHandle_t)request_queue_, &request, portMAX_DELAY) !=
        pdPASS) {
      continue;
    }
    if (request.score == nullptr) {
      continue;
    }
    // The flag belongs to whatever was playing when the preempt arrived;
    // clearing it here stops the replacement from cutting itself off.
    preempt_ = false;

    if (!ParseHeader(request.score)) {
      ESP_LOGE(kTag, "RTTTL header parse failed");
      StopPlayback();
      continue;
    }

    score_ = request.score;
    playing_ = true;
    playback_volume_ = (request.volume < 0)
                           ? kMaxVolume
                           : std::min<int>(request.volume, kMaxVolume);

    const std::optional<NoteEvent> event = ParseNextNote();
    if (!event) {
      StopPlayback();
      continue;
    }

    StartEvent(*event, Sys().Timebase().NowMs());
  }
}

bool TonePlayer::ParseHeader(const char *rtttl) {
  const char *p = rtttl;

  while (*p != '\0' && *p != ':') {
    ++p;
  }
  if (*p != ':') {
    return false;
  }
  ++p;

  default_duration_ = 4;
  default_octave_ = 6;
  bpm_ = 63;

  while (*p != '\0' && *p != ':') {
    if (*p == 'd' && *(p + 1) == '=') {
      p += 2;
      const ParsedNumber parsed = ParseNumber(p);
      p = parsed.next;
      if (parsed.value != 0) {
        default_duration_ = parsed.value;
      }
    } else if (*p == 'o' && *(p + 1) == '=') {
      p += 2;
      const ParsedNumber parsed = ParseNumber(p);
      p = parsed.next;
      if (parsed.value <= 8) {
        default_octave_ = parsed.value;
      }
    } else if (*p == 'b' && *(p + 1) == '=') {
      p += 2;
      const ParsedNumber parsed = ParseNumber(p);
      p = parsed.next;
      if (parsed.value >= kMinBpm) {
        bpm_ = parsed.value;
      }
    } else {
      ++p;
    }

    if (*p == ',') {
      ++p;
    }
  }

  if (*p != ':') {
    return false;
  }
  ++p;

  whole_note_ms_ = static_cast<TimeMs>((240000UL) / bpm_);
  if (whole_note_ms_ == 0) {
    whole_note_ms_ = kDefaultWholeNoteMs;
  }

  cursor_ = p;
  return true;
}

std::optional<TonePlayer::NoteEvent> TonePlayer::ParseNextNote() {
  if (cursor_ == nullptr) {
    return std::nullopt;
  }

  cursor_ = SkipSeparators(cursor_);
  if (*cursor_ == '\0') {
    return std::nullopt;
  }

  const ParsedNumber duration = ParseNumber(cursor_);
  cursor_ = duration.next;
  const uint32_t duration_value = duration.value;
  const uint32_t duration_divisor =
      (duration_value == 0) ? default_duration_ : duration_value;
  if (duration_divisor == 0) {
    return std::nullopt;
  }

  char note = *cursor_;
  if (note == '\0') {
    return std::nullopt;
  }
  if (note >= 'A' && note <= 'Z') {
    note = static_cast<char>(note - 'A' + 'a');
  }
  ++cursor_;

  Accidental accidental = Accidental::kNatural;
  if (*cursor_ == '#') {
    accidental = Accidental::kSharp;
    ++cursor_;
  } else if (*cursor_ == '_') {
    accidental = Accidental::kFlat;
    ++cursor_;
  }

  bool dotted = false;
  if (*cursor_ == '.') {
    dotted = true;
    ++cursor_;
  }

  uint32_t octave = default_octave_;
  if (*cursor_ >= '0' && *cursor_ <= '9') {
    const ParsedNumber parsed = ParseNumber(cursor_);
    cursor_ = parsed.next;
    octave = parsed.value;
  }

  if (*cursor_ == '.') {
    dotted = true;
    ++cursor_;
  }

  if (*cursor_ == ',') {
    ++cursor_;
  }

  TimeMs duration_ms = static_cast<TimeMs>(whole_note_ms_ / duration_divisor);
  if (dotted) {
    duration_ms = static_cast<TimeMs>(duration_ms + (duration_ms / 2));
  }

  NoteEvent event{};
  event.duration_ms = duration_ms;

  // A rest is a real event with no frequency, not a parse failure.
  if (note == 'p') {
    event.freq_hz = 0;
    return event;
  }

  event.freq_hz = NoteToFrequencyHz(note, accidental, octave);
  if (event.freq_hz == 0) {
    return std::nullopt;
  }
  return event;
}

void TonePlayer::StartEvent(const NoteEvent &event, TimeMs now) {
  if (event.freq_hz == 0) {
    buzzer_->Stop();
  } else {
    buzzer_->SetDutyCycle(DutyCycleForVolume(playback_volume_));
    buzzer_->Start(event.freq_hz);
  }
  next_change_ms_ = TimeAfter(now, event.duration_ms);
}

float TonePlayer::DutyCycleForVolume(uint8_t volume) const {
  const uint8_t clamped = std::min<uint8_t>(volume, kMaxVolume);
  const float normalized = static_cast<float>(clamped) / 10.0f;
  return 0.5f * normalized * normalized;
}

const char *TonePlayer::SkipSeparators(const char *p) {
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == ',') {
    ++p;
  }
  return p;
}

TonePlayer::ParsedNumber TonePlayer::ParseNumber(const char *p) {
  uint32_t value = 0;
  bool have_digit = false;
  while (*p >= '0' && *p <= '9') {
    have_digit = true;
    value = (value * 10U) + static_cast<uint32_t>(*p - '0');
    ++p;
  }
  return ParsedNumber{.value = have_digit ? value : 0, .next = p};
}

uint32_t TonePlayer::NoteToFrequencyHz(char note, Accidental accidental,
                                       uint32_t octave) {
  int semitone = -1;
  switch (note) {
    case 'c':
      semitone = 0;
      break;
    case 'd':
      semitone = 2;
      break;
    case 'e':
      semitone = 4;
      break;
    case 'f':
      semitone = 5;
      break;
    case 'g':
      semitone = 7;
      break;
    case 'a':
      semitone = 9;
      break;
    case 'b':
      semitone = 11;
      break;
    default:
      return 0;
  }

  switch (accidental) {
    case Accidental::kSharp:
      ++semitone;
      break;
    case Accidental::kFlat:
      --semitone;
      break;
    case Accidental::kNatural:
      break;
  }

  int octave_i = static_cast<int>(octave);
  if (semitone < 0) {
    semitone += 12;
    --octave_i;
  } else if (semitone >= 12) {
    semitone -= 12;
    ++octave_i;
  }
  if (octave_i < 0) {
    return 0;
  }

  const int midi_note = ((octave_i + 1) * 12) + semitone;
  const float semitone_offset = static_cast<float>(midi_note - 69) / 12.0f;
  const float freq = 440.0f * std::pow(2.0f, semitone_offset);
  return static_cast<uint32_t>(std::lround(freq));
}
