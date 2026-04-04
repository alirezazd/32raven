#pragma once

#include <cstddef>
#include <cstdint>

#include "buzzer.hpp"
#include "esp32_limits.hpp"
#include "timebase.hpp"

class TonePlayer {
 public:
  static constexpr uint8_t kMaxVolume = 10;
  static constexpr std::size_t kPendingRequestQueueDepth =
      esp32_limits::kTonePlayerPendingRequestQueueDepth;

  enum class BuiltinTone {
    kBeep,
    kConfirm,
    kWarning,
    kError,
  };

  struct Config {
    uint8_t volume = 3;  // 0..10
  };

  static TonePlayer &GetInstance() {
    static TonePlayer instance;
    return instance;
  }

  void Init(const Config &cfg, Buzzer *buzzer);

  bool PlayRtttl(const char *rtttl, int volume = -1);
  void PlayBuiltin(BuiltinTone tone, int volume = -1);
  void Stop();
  bool IsPlaying() const { return playing_; }

 private:
  struct NoteEvent {
    uint32_t freq_hz = 0;
    TimeMs duration_ms = 0;
    bool valid = false;
  };

  struct PendingRequest {
    const char *score = nullptr;
    int volume = -1;
  };

  TonePlayer() = default;
  ~TonePlayer() = default;
  TonePlayer(const TonePlayer &) = delete;
  TonePlayer &operator=(const TonePlayer &) = delete;

  static void TaskEntry(void *param);
  void Task();
  bool ParseHeader(const char *rtttl);
  NoteEvent ParseNextNote();
  void StartEvent(const NoteEvent &event, TimeMs now);
  float DutyCycleForVolume(uint8_t volume) const;

  static void SkipSeparators(const char *&p);
  static uint32_t ParseNumber(const char *&p);
  static uint32_t NoteToFrequencyHz(char note, bool sharp, bool flat,
                                    uint32_t octave);

  Buzzer *buzzer_ = nullptr;
  Config cfg_{};
  bool playing_ = false;
  const char *score_ = nullptr;
  const char *cursor_ = nullptr;

  uint32_t default_duration_ = 4;
  uint32_t default_octave_ = 6;
  uint32_t bpm_ = 63;
  TimeMs whole_note_ms_ = 0;
  TimeMs next_change_ms_ = 0;
  uint8_t playback_volume_ = kMaxVolume;
  void *task_handle_ = nullptr;    // TaskHandle_t
  void *request_queue_ = nullptr;  // QueueHandle_t
};
