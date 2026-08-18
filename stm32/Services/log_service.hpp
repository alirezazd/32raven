// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "ff.h"
#include "message.hpp"
#include "ring_buffer.hpp"
#include "shared_state.hpp"
#include "topic_scheduler.hpp"

// The blackbox: ULog onto the SD card. Each flight records into one
// contiguous preallocated file, so the arm path never waits on FAT and the
// flight path writes raw sectors through Sdio -- FatFs sees the file again
// only at finalize.
//
// PushEstimate produces from the control tick (PendSV), Poll consumes from
// the main tick, and the ring is their whole interface. Ring and staging stay
// in plain SRAM: CCM is not DMA-reachable and the flush is a DMA read.
//
// Init panics on an unusable card; past init a full file or a dead write
// leaves the service inert and counting, never feeding Sentinel.
class LogService {
 public:
  // Only `period` is read: this scheduler emits every due topic rather than
  // picking one, and never suppresses. `priority` and `max_silence` are
  // carried to keep one TopicConfig shape across both schedulers.
  struct Config {
    uint32_t prealloc_mb;
    TopicConfig rc_input{};
    TopicConfig battery{};
    TopicConfig esc_telemetry{};
    TopicConfig gps{};
    TopicConfig imu_health{};
    TopicConfig crsf_link{};
    TopicConfig logger_status{};
  };

  static constexpr size_t kSlowTopicCount = 7;

  // The only member callable from PendSV; a full ring drops the record.
  void PushEstimate(const EstimatorState &estimate);

  // Both blocking and idempotent; StopFlight also preallocates the next file.
  void StartFlight(uint32_t now_us);
  void StopFlight();

  // The MSC handover: the host owns the volume between these and may have
  // rewritten anything, the FAT included, so Remount re-probes from scratch.
  void ReleaseCard();
  void RemountAfterMsc();

  // Blocking FatFs reads; the callers gate on Idle so they land disarmed.
  void ListLogs(uint8_t first, message::LogListReplyMsg &out);
  void ReadLog(const message::LogReadMsg &req, message::LogDataMsg &out);

  void Poll(uint32_t now_us);

 private:
  friend class System;
  void Init(const Config &cfg, SharedState &blackboard);

  // ULog message ids double as topic-scheduler slots for the slow set.
  enum MsgId : uint16_t {
    kMsgEstimator = 0,
    kMsgRc,
    kMsgBattery,
    kMsgEscTelemetry,
    kMsgGps,
    kMsgImuHealth,
    kMsgCrsfLink,
    kMsgLogger,
    kMsgCount,
  };
  static_assert(kSlowTopicCount == kMsgCount - 1);

  struct Stats {
    uint32_t dropped_bytes = 0;
    uint32_t write_errors = 0;
  };

  void PrepareNextFile();
  bool TryReuseEmptyLog(uint32_t index, FSIZE_t bytes);
  void FormatLogName(uint32_t index);
  void AdoptOpenFile(FSIZE_t bytes);
  uint64_t Now64(uint32_t now_us);
  uint64_t Stamp64(uint32_t src_us) const;

  void AppendToStaging(const void *data, size_t len);
  void AppendUlogMessage(uint8_t type, const void *payload, uint16_t len);
  void AppendDefinitions(uint64_t now64);
  void AppendSlowTopics(uint64_t now64, uint32_t now_us);
  void StageFromRing();
  void TryStartFlush();
  void DrainFlush();
  void FinishFlush(bool ok);
  void FailSink();

  Config cfg_{};
  bool initialized_ = false;
  bool mounted_ = false;
  bool file_ready_ = false;

  FATFS fs_{};
  FIL file_{};
  char file_name_[13] = {};

  // The contiguous run f_expand carved out, in absolute card sectors.
  uint32_t file_start_lba_ = 0;
  uint32_t file_capacity_bytes_ = 0;

  // Sized for the card's silence, not its speed: a wear-levelling stall parks
  // the flush for hundreds of milliseconds while the control tick keeps
  // producing ~110 KB/s.
  static constexpr size_t kRingBytes = 32 * 1024;
  static constexpr size_t kStagingBytes = 4096;

  RingBuffer<uint8_t, kRingBytes> ring_{};
  alignas(4) uint8_t staging_[2][kStagingBytes] = {};
  uint8_t fill_index_ = 0;
  size_t fill_len_ = 0;
  bool dma_busy_ = false;

  // Producer gate, written from the main tick and read from PendSV. Separate
  // from session_open_ so a dead card stops the producer while the file stays
  // open for a best-effort finalize at disarm.
  volatile bool logging_ = false;
  bool session_open_ = false;
  bool sink_failed_ = false;

  uint32_t log_bytes_ = 0;      // ULog stream length
  uint32_t flushed_bytes_ = 0;  // committed to the card, sector-granular

  // 32-bit deltas accumulated into a 64-bit total: the unwrapped host clock,
  // which is also the domain the IMU servo stamps EstimatorState in.
  uint64_t total_us_ = 0;
  uint32_t last_now_us_ = 0;
  bool time_seeded_ = false;

  TopicScheduler slow_sched_{};
  // In MsgId order; a member, not a local, because the scheduler holds the
  // span rather than a copy.
  std::array<TopicConfig, kSlowTopicCount> slow_configs_{};
  std::array<TopicState, kSlowTopicCount> slow_states_{};

  // One file cached for the WiFi pull, reopened when the name changes.
  FIL read_file_{};
  char read_name_[13] = {};
  bool read_open_ = false;

  SharedState *blackboard_ = nullptr;
  Stats stats_{};
};
