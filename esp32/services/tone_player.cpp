#include "tone_player.hpp"

#include <algorithm>
#include <cmath>

#include "panic.hpp"

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

constexpr const char kBeepRtttl[] = "Beep:d=16,o=6,b=240:c";
constexpr const char kConfirmRtttl[] = "Confirm:d=32,o=6,b=200:g,c7,16e7";
constexpr const char kWarningRtttl[] = "Warning:d=32,o=6,b=180:c,p,e,p,c5";
constexpr const char kErrorRtttl[] = "Error:d=32,o=6,b=180:a,p,a,p,a,p,a,p,a";

const char *BuiltinToneToRtttl(TonePlayer::BuiltinTone tone) {
  switch (tone) {
    case TonePlayer::BuiltinTone::kBeep:
      return kBeepRtttl;
    case TonePlayer::BuiltinTone::kConfirm:
      return kConfirmRtttl;
    case TonePlayer::BuiltinTone::kWarning:
      return kWarningRtttl;
    case TonePlayer::BuiltinTone::kError:
      return kErrorRtttl;
    default:
      return nullptr;
  }
}
}  // namespace

void TonePlayer::Init(const Config &cfg, Buzzer *buzzer) {
  static constexpr uint32_t kTaskStackBytes = 3072;
  static StaticQueue_t request_queue_buffer;
  static uint8_t
      request_queue_storage[kPendingRequestQueueDepth * sizeof(PendingRequest)];
  static StaticTask_t task_buffer;
  static StackType_t task_stack[kTaskStackBytes];

  if (initialized_) {
    Panic(ErrorCode::kTonePlayerInitFailed);
  }

  cfg_ = cfg;
  buzzer_ = buzzer;
  if (buzzer_ == nullptr) {
    Panic(ErrorCode::kTonePlayerInitFailed);
  }

  playback_volume_ = cfg_.volume;
  (void)buzzer_->SetDutyCycle(DutyCycleForVolume(playback_volume_));
  (void)buzzer_->Stop();
  request_queue_ =
      xQueueCreateStatic(kPendingRequestQueueDepth, sizeof(PendingRequest),
                         request_queue_storage, &request_queue_buffer);
  if (request_queue_ == nullptr) {
    Panic(ErrorCode::kTonePlayerInitFailed);
  }
  task_handle_ = xTaskCreateStatic(TaskEntry, "tone_player", kTaskStackBytes,
                                   this, 1, task_stack, &task_buffer);
  if (task_handle_ == nullptr) {
    Panic(ErrorCode::kTonePlayerInitFailed);
  }
  initialized_ = true;
  ESP_LOGI(kTag, "initialized");
}

bool TonePlayer::PlayRtttl(const char *rtttl, int volume) {
  if (!initialized_ || rtttl == nullptr || *rtttl == '\0') {
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

void TonePlayer::Stop() {
  if (!initialized_) {
    return;
  }

  xQueueReset((QueueHandle_t)request_queue_);
  if (task_handle_ != nullptr) {
    xTaskNotifyGive((TaskHandle_t)task_handle_);
  }
}

void TonePlayer::TaskEntry(void *param) {
  static_cast<TonePlayer *>(param)->Task();
}

void TonePlayer::Task() {
  while (true) {
    if (playing_) {
      if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
        playing_ = false;
        score_ = nullptr;
        cursor_ = nullptr;
        next_change_ms_ = 0;
        (void)buzzer_->Stop();
        continue;
      }

      const TimeMs now = NowMs();
      if (TimeReached(now, next_change_ms_)) {
        const NoteEvent event = ParseNextNote();
        if (!event.valid) {
          playing_ = false;
          score_ = nullptr;
          cursor_ = nullptr;
          next_change_ms_ = 0;
          (void)buzzer_->Stop();
        } else {
          StartEvent(event, now);
        }
        continue;
      }

      const TimeMs wait_ms = next_change_ms_ - now;
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms > 0 ? wait_ms : 1)) >
          0) {
        playing_ = false;
        score_ = nullptr;
        cursor_ = nullptr;
        next_change_ms_ = 0;
        (void)buzzer_->Stop();
      }
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

    if (!ParseHeader(request.score)) {
      ESP_LOGE(kTag, "RTTTL header parse failed");
      playing_ = false;
      score_ = nullptr;
      cursor_ = nullptr;
      next_change_ms_ = 0;
      (void)buzzer_->Stop();
      continue;
    }

    score_ = request.score;
    playing_ = true;
    playback_volume_ = (request.volume < 0)
                           ? kMaxVolume
                           : std::min<int>(request.volume, kMaxVolume);

    const NoteEvent event = ParseNextNote();
    if (!event.valid) {
      playing_ = false;
      score_ = nullptr;
      cursor_ = nullptr;
      next_change_ms_ = 0;
      (void)buzzer_->Stop();
      continue;
    }

    StartEvent(event, NowMs());
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
      const uint32_t value = ParseNumber(p);
      if (value != 0) {
        default_duration_ = value;
      }
    } else if (*p == 'o' && *(p + 1) == '=') {
      p += 2;
      const uint32_t value = ParseNumber(p);
      if (value <= 8) {
        default_octave_ = value;
      }
    } else if (*p == 'b' && *(p + 1) == '=') {
      p += 2;
      const uint32_t value = ParseNumber(p);
      if (value >= kMinBpm) {
        bpm_ = value;
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

TonePlayer::NoteEvent TonePlayer::ParseNextNote() {
  if (cursor_ == nullptr) {
    return {};
  }

  SkipSeparators(cursor_);
  if (*cursor_ == '\0') {
    return {};
  }

  const uint32_t duration_value = ParseNumber(cursor_);
  const uint32_t duration_divisor =
      (duration_value == 0) ? default_duration_ : duration_value;
  if (duration_divisor == 0) {
    return {};
  }

  char note = *cursor_;
  if (note == '\0') {
    return {};
  }
  if (note >= 'A' && note <= 'Z') {
    note = static_cast<char>(note - 'A' + 'a');
  }
  ++cursor_;

  bool sharp = false;
  bool flat = false;
  if (*cursor_ == '#') {
    sharp = true;
    ++cursor_;
  } else if (*cursor_ == '_') {
    flat = true;
    ++cursor_;
  }

  bool dotted = false;
  if (*cursor_ == '.') {
    dotted = true;
    ++cursor_;
  }

  uint32_t octave = default_octave_;
  if (*cursor_ >= '0' && *cursor_ <= '9') {
    octave = ParseNumber(cursor_);
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
  event.valid = true;

  if (note == 'p') {
    event.freq_hz = 0;
    return event;
  }

  event.freq_hz = NoteToFrequencyHz(note, sharp, flat, octave);
  if (event.freq_hz == 0) {
    event.valid = false;
  }
  return event;
}

void TonePlayer::StartEvent(const NoteEvent &event, TimeMs now) {
  if (event.freq_hz == 0) {
    (void)buzzer_->Stop();
  } else {
    (void)buzzer_->SetDutyCycle(DutyCycleForVolume(playback_volume_));
    (void)buzzer_->Start(event.freq_hz);
  }
  next_change_ms_ = TimeAfter(now, event.duration_ms);
}

float TonePlayer::DutyCycleForVolume(uint8_t volume) const {
  const uint8_t clamped = std::min<uint8_t>(volume, kMaxVolume);
  const float normalized = static_cast<float>(clamped) / 10.0f;
  return 0.5f * normalized * normalized;
}

void TonePlayer::SkipSeparators(const char *&p) {
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == ',') {
    ++p;
  }
}

uint32_t TonePlayer::ParseNumber(const char *&p) {
  uint32_t value = 0;
  bool have_digit = false;
  while (*p >= '0' && *p <= '9') {
    have_digit = true;
    value = (value * 10U) + static_cast<uint32_t>(*p - '0');
    ++p;
  }
  return have_digit ? value : 0;
}

uint32_t TonePlayer::NoteToFrequencyHz(char note, bool sharp, bool flat,
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

  if (sharp) {
    ++semitone;
  }
  if (flat) {
    --semitone;
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
