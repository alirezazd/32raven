// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "log_service.hpp"

#include <charconv>
#include <cstdio>
#include <cstring>
#include <limits>
#include <span>
#include <string_view>

#include "error_code.hpp"
#include "fc_link.hpp"
#include "panic.hpp"
#include "sdio.hpp"
#include "system.hpp"
#include "watchdog.hpp"

namespace {

// Far above any real root directory, far below forever.
constexpr uint32_t kMaxRootScanEntries = 4096;

// A time budget: preallocation walks the FAT a sector at a time over SDIO.
// 32 GB at the 32 KB-cluster default builds a 4 MB table; the same card cut
// into 4 KB clusters builds 32 MB, which is what this rejects.
constexpr uint32_t kMaxFatSectors = 20000;

// What a ULog reader takes for "no sample", as opposed to a sample of zero.
constexpr float kMissingFloat = std::numeric_limits<float>::quiet_NaN();

// FatFs R0.15 ends a directory with FR_OK and an empty name it never writes,
// so the name must be cleared before every read or the walk repeats its last
// entry forever. Fixed upstream in R0.15a.
void ClearDirEntry(FILINFO &info) { info.fname[0] = '\0'; }

void LogSdFailure(FcLink &fc_link, const char *what, int res) {
  const Sdio::Stats &s = Sdio::GetInstance().GetStats();
  fc_link.SendLog(
      "sd: %s failed (%d) cto=%lu ccrc=%lu dto=%lu dcrc=%lu", what, res,
      static_cast<unsigned long>(s.cmd_timeouts),
      static_cast<unsigned long>(s.cmd_crc_errors),
      static_cast<unsigned long>(s.data_timeouts),
      static_cast<unsigned long>(s.data_crc_errors));
}

// ULog wire format (docs.px4.io/main/en/dev_log/ulog_file_format): every
// message is {uint16 size, uint8 type} + payload, and size counts payload only.

constexpr uint8_t kUlogMagic[8] = {0x55, 0x4C, 0x6F, 0x67,
                                   0x01, 0x12, 0x35, 0x01};

struct __attribute__((packed)) MsgHeader {
  uint16_t size;
  uint8_t type;
};

// This message must lead the definitions section; all-zero means nothing
// appended and nothing a reader may skip.
struct __attribute__((packed)) FlagBits {
  uint8_t compat[8];
  uint8_t incompat[8];
  uint64_t appended_offsets[3];
};

// Each layout must match its format string byte for byte; the static_asserts
// are the only thing tying the two together.

// PX4's sensor_gyro_fifo / sensor_accel_fifo, field for field: one record per
// FIFO read, `scale` turning a count into SI. Gyro and accel are separate
// topics rather than one packed record because PlotJuggler and Flight Review
// find the FIFO expansion by topic name, and a record can only carry one.
struct __attribute__((packed)) SensorFifoRecord {
  MsgHeader hdr;
  uint16_t msg_id;
  uint64_t timestamp;
  uint64_t timestamp_sample;
  uint32_t device_id;
  float dt;
  float scale;
  uint8_t samples;
  int32_t x[kImuMaxSamples];
  int32_t y[kImuMaxSamples];
  int32_t z[kImuMaxSamples];
};
// The format message declares these widths by hand, and a reader trusts it
// over the bytes: padding here would shift every field after it.
static_assert(sizeof(SensorFifoRecord) ==
              sizeof(MsgHeader) + 2u + 8u + 8u + 4u + 4u + 4u + 1u +
                  (3u * kImuMaxSamples * 4u));
constexpr size_t DecimalDigits(uint32_t value) {
  size_t digits = 1;
  for (; value >= 10u; value /= 10u) {
    ++digits;
  }
  return digits;
}

// The only formats with a build-time array width in them. Built rather than
// written out, and built at compile time: a runtime formatter would need a
// buffer sized by hand.
constexpr std::string_view kFifoTail =
    ":uint64_t timestamp;uint64_t timestamp_sample;uint32_t device_id;"
    "float dt;float scale;uint8_t samples;";
constexpr std::string_view kFifoOpen = "int32_t[";
constexpr std::string_view kFifoClose = "] ";
constexpr std::string_view kFifoAxes[] = {"x", "y", "z"};

constexpr size_t kFifoAxesLen = [] {
  size_t len = 0;
  for (std::string_view axis : kFifoAxes) {
    len += kFifoOpen.size() + DecimalDigits(kImuMaxSamples) + kFifoClose.size() +
           axis.size() + 1u;
  }
  return len;
}();

// N carries the literal's length, so each topic name sizes its own storage.
template <size_t N>
constexpr auto MakeFifoFmt(const char (&name)[N]) {
  std::array<char, (N - 1u) + kFifoTail.size() + kFifoAxesLen + 1u> out{};
  size_t at = 0;
  const auto put = [&out, &at](std::string_view text) {
    for (char c : text) {
      out[at++] = c;
    }
  };
  put(std::string_view(name, N - 1u));
  put(kFifoTail);
  for (std::string_view axis : kFifoAxes) {
    put(kFifoOpen);
    at = static_cast<size_t>(
        std::to_chars(out.data() + at, out.data() + out.size(), kImuMaxSamples)
            .ptr -
        out.data());
    put(kFifoClose);
    put(axis);
    put(";");
  }
  return out;
}

constexpr auto kGyroFifoFmt = MakeFifoFmt("sensor_gyro_fifo");
constexpr auto kAccelFifoFmt = MakeFifoFmt("sensor_accel_fifo");
constexpr const char *kFmtSensorGyroFifo = kGyroFifoFmt.data();
constexpr const char *kFmtSensorAccelFifo = kAccelFifoFmt.data();

struct __attribute__((packed)) RcRecord {
  MsgHeader hdr;
  uint16_t msg_id;
  uint64_t timestamp;
  uint16_t channels[16];
  uint16_t roll_us;
  uint16_t pitch_us;
  uint16_t yaw_us;
  uint16_t throttle_us;
};
static_assert(sizeof(RcRecord) == 53);
constexpr char kFmtRc[] =
    "rc_input:uint64_t timestamp;uint16_t[16] channels;uint16_t roll_us;"
    "uint16_t pitch_us;uint16_t yaw_us;uint16_t throttle_us;";

struct __attribute__((packed)) BatteryRecord {
  MsgHeader hdr;
  uint16_t msg_id;
  uint64_t timestamp;
  float voltage;
  float current;
  float mah_drawn;
  uint8_t remaining;
};
static_assert(sizeof(BatteryRecord) == 26);
constexpr char kFmtBattery[] =
    "battery:uint64_t timestamp;float voltage;float current;float mah_drawn;"
    "uint8_t remaining;";

struct __attribute__((packed)) EscTelemetryRecord {
  MsgHeader hdr;
  uint16_t msg_id;
  uint64_t timestamp;
  uint32_t rpm[4];
  float voltage[4];
  float current[4];
  int16_t temperature_c[4];
  uint8_t valid_mask;
};
static_assert(sizeof(EscTelemetryRecord) == 70);
constexpr char kFmtEscTelemetry[] =
    "esc_telemetry:uint64_t timestamp;uint32_t[4] rpm;float[4] voltage;"
    "float[4] current;int16_t[4] temperature_c;uint8_t valid_mask;";

struct __attribute__((packed)) GpsRecord {
  MsgHeader hdr;
  uint16_t msg_id;
  uint64_t timestamp;
  int32_t lat_1e7;
  int32_t lon_1e7;
  int32_t alt_mm;
  uint16_t vel_cms;
  uint16_t hdg_cdeg;
  uint16_t hdop;
  uint8_t fix_type;
  uint8_t num_sats;
};
static_assert(sizeof(GpsRecord) == 33);
constexpr char kFmtGps[] =
    "gps:uint64_t timestamp;int32_t lat_1e7;int32_t lon_1e7;int32_t alt_mm;"
    "uint16_t vel_cms;uint16_t hdg_cdeg;uint16_t hdop;uint8_t fix_type;"
    "uint8_t num_sats;";

struct __attribute__((packed)) ImuHealthRecord {
  MsgHeader hdr;
  uint16_t msg_id;
  uint64_t timestamp;
  uint32_t publish_count;
  uint32_t path_faults;
  uint32_t overruns;
  uint32_t dma_start_fails;
  uint32_t spi_errors;
  uint32_t parse_fails;
  uint32_t dropped_records;
  uint32_t missed_samples;
};
static_assert(sizeof(ImuHealthRecord) == 45);
constexpr char kFmtImuHealth[] =
    "imu_health:uint64_t timestamp;uint32_t publish_count;"
    "uint32_t path_faults;uint32_t overruns;uint32_t dma_start_fails;"
    "uint32_t spi_errors;uint32_t parse_fails;uint32_t dropped_records;"
    "uint32_t missed_samples;";

struct __attribute__((packed)) CrsfLinkRecord {
  MsgHeader hdr;
  uint16_t msg_id;
  uint64_t timestamp;
  uint8_t uplink_rssi_ant1_dbm;
  uint8_t uplink_rssi_ant2_dbm;
  uint8_t uplink_link_quality;
  int8_t uplink_snr_db;
  uint8_t rf_mode;
  uint8_t uplink_tx_power_index;
  uint8_t downlink_rssi_dbm;
  uint8_t downlink_link_quality;
  int8_t downlink_snr_db;
};
static_assert(sizeof(CrsfLinkRecord) == 22);
constexpr char kFmtCrsfLink[] =
    "crsf_link:uint64_t timestamp;uint8_t uplink_rssi_ant1_dbm;"
    "uint8_t uplink_rssi_ant2_dbm;uint8_t uplink_link_quality;"
    "int8_t uplink_snr_db;uint8_t rf_mode;uint8_t uplink_tx_power_index;"
    "uint8_t downlink_rssi_dbm;uint8_t downlink_link_quality;"
    "int8_t downlink_snr_db;";

struct __attribute__((packed)) LoggerRecord {
  MsgHeader hdr;
  uint16_t msg_id;
  uint64_t timestamp;
  uint32_t dropped_bytes;
  uint32_t write_errors;
};
static_assert(sizeof(LoggerRecord) == 21);
constexpr char kFmtLogger[] =
    "logger_status:uint64_t timestamp;uint32_t dropped_bytes;"
    "uint32_t write_errors;";

// One entry per MsgId, the pushed topics first.
// Both tables are indexed by MsgId, so the raw IMU pair leads them only when it
// is in the file at all. Built rather than listed for that reason.
template <typename T>
constexpr auto MakeTopicTable(T gyro_fifo, T accel_fifo, T rc, T battery,
                              T esc, T gps, T imu_health, T crsf, T logger) {
  std::array<T, LogService::kTopicCount> out{};
  size_t at = 0;
  if constexpr (LogService::kRawImuLogEnabled) {
    out[at++] = gyro_fifo;
    out[at++] = accel_fifo;
  }
  out[at++] = rc;
  out[at++] = battery;
  out[at++] = esc;
  out[at++] = gps;
  out[at++] = imu_health;
  out[at++] = crsf;
  out[at++] = logger;
  return out;
}

constexpr auto kFormats = MakeTopicTable<const char *>(
    kFmtSensorGyroFifo, kFmtSensorAccelFifo, kFmtRc, kFmtBattery,
    kFmtEscTelemetry, kFmtGps, kFmtImuHealth, kFmtCrsfLink, kFmtLogger);
constexpr auto kTopicNames = MakeTopicTable<const char *>(
    "sensor_gyro_fifo", "sensor_accel_fifo", "rc_input", "battery",
    "esc_telemetry", "gps", "imu_health", "crsf_link", "logger_status");

// Returns the index from "LOGnnnnn.ULG", or 0 for any other name.
uint32_t LogFileIndex(const char *name) {
  if (std::strncmp(name, "LOG", 3) != 0 || std::strlen(name) != 12 ||
      std::strcmp(name + 8, ".ULG") != 0) {
    return 0;
  }
  uint32_t index = 0;
  for (int i = 3; i < 8; ++i) {
    if (name[i] < '0' || name[i] > '9') {
      return 0;
    }
    index = (index * 10u) + static_cast<uint32_t>(name[i] - '0');
  }
  return index;
}

template <typename Record>
Record MakeRecord(uint16_t msg_id, uint64_t timestamp) {
  Record rec{};
  rec.hdr = {sizeof(Record) - sizeof(MsgHeader), 'D'};
  rec.msg_id = msg_id;
  rec.timestamp = timestamp;
  return rec;
}

}  // namespace

void LogService::Init(const Config &cfg, SharedState &blackboard,
                      FcLink &fc_link) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kLogServiceReinit);
  }
  cfg_ = cfg;
  blackboard_ = &blackboard;
  fc_link_ = &fc_link;
  // MsgId order: the order AppendSlowTopics walks and the format and
  // subscription tables use. Drift here silently mislabels a topic.
  slow_configs_ = {
      cfg_.rc_input,   cfg_.battery,   cfg_.esc_telemetry, cfg_.gps,
      cfg_.imu_health, cfg_.crsf_link, cfg_.logger_status,
  };
  static_assert(kSlowTopicCount == 7,
                "slow_configs_ above lists one entry per scheduled topic");
  initialized_ = true;

  if (!Sdio::GetInstance().CardPresent()) {
    LogSdFailure(*fc_link_, "probe", 0);
    Panic(ErrorCode::Stm32::kSdCardMissing);
  }

  const FRESULT res = f_mount(&fs_, "", 1);
  if (res != FR_OK) {
    LogSdFailure(*fc_link_, "mount", res);
    Panic(ErrorCode::Stm32::kSdCardCorrupted);
  }
  mounted_ = true;

  if (fs_.fsize > kMaxFatSectors) {
    LogSdFailure(*fc_link_, "fat too large", static_cast<int>(fs_.fsize));
    Panic(ErrorCode::Stm32::kSdCardGeometry);
  }

  // Deferred to Poll: Init runs before FcLink exists, so scanning the FAT here
  // spends seconds the ESP32 can only read as an unanswered handshake. The
  // first tick services FcLink ahead of this service.
  prepare_pending_ = true;
}

void LogService::PrepareNextFile() {
  file_ready_ = false;

  DIR dir{};
  FILINFO info{};
  uint32_t max_index = 0;
  const FRESULT dir_res = f_opendir(&dir, "/");
  if (dir_res != FR_OK) {
    LogSdFailure(*fc_link_, "root scan", dir_res);
    Panic(ErrorCode::Stm32::kSdCardCorrupted);
  }
  uint32_t scanned = 0;
  FSIZE_t max_size = 0;
  FRESULT read_res = FR_OK;
  while (true) {
    ClearDirEntry(info);
    read_res = f_readdir(&dir, &info);
    if (read_res != FR_OK || info.fname[0] == '\0') {
      break;
    }
    // FatFs cannot detect a looped cluster chain, so a corrupted directory
    // scans forever; the cap turns that hang into a diagnosable fault.
    if (++scanned > kMaxRootScanEntries) {
      fc_link_->SendLog(
          "sd: root scan runaway at '%s'", info.fname);
      Panic(ErrorCode::Stm32::kSdCardCorrupted);
    }
    const uint32_t index = LogFileIndex(info.fname);
    if (index > max_index) {
      max_index = index;
      max_size = info.fsize;
    }
  }
  f_closedir(&dir);
  if (read_res != FR_OK) {
    LogSdFailure(*fc_link_, "dir read", read_res);
    Panic(ErrorCode::Stm32::kSdCardCorrupted);
  }

  const FSIZE_t bytes = static_cast<FSIZE_t>(cfg_.prealloc_mb) * 1024u * 1024u;
  if (max_index > 0u && max_size == bytes &&
      TryReuseEmptyLog(max_index, bytes)) {
    return;
  }

  FormatLogName(max_index + 1u);

  const FRESULT open_res =
      f_open(&file_, file_name_, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
  if (open_res != FR_OK) {
    LogSdFailure(*fc_link_, "create", open_res);
    Panic(ErrorCode::Stm32::kSdCardCorrupted);
  }

  const FRESULT expand_res = f_expand(&file_, bytes, 1);
  if (expand_res != FR_OK) {
    // The zero-length file stays: its name marks where the sequence stopped.
    // FR_DENIED means no contiguous run this large: a full card, not a
    // broken one.
    f_close(&file_);
    LogSdFailure(*fc_link_, "prealloc", expand_res);
    Panic(expand_res == FR_DENIED ? ErrorCode::Stm32::kSdCardFull
                                  : ErrorCode::Stm32::kSdCardCorrupted);
  }

  // f_expand leaves the size and start cluster in RAM until a sync, so a power
  // cut would strand the run as lost clusters behind a zero-length name.
  const FRESULT sync_res = f_sync(&file_);
  if (sync_res != FR_OK) {
    f_close(&file_);
    LogSdFailure(*fc_link_, "prealloc sync", sync_res);
    Panic(ErrorCode::Stm32::kSdCardCorrupted);
  }

  AdoptOpenFile(bytes);

  fc_link_->SendLog(
      "sd: %s ready, card %lu MB", file_name_,
      static_cast<unsigned long>(
          (static_cast<uint64_t>(Sdio::GetInstance().BlockCount()) *
           Sdio::kBlockBytes) >>
          20));
}

void LogService::FormatLogName(uint32_t index) {
  std::snprintf(file_name_, sizeof(file_name_), "LOG%05lu.ULG",
                static_cast<unsigned long>(index));
}

// f_expand made the run contiguous, so its first absolute sector pins the
// whole range for raw Sdio writes.
void LogService::AdoptOpenFile(FSIZE_t bytes) {
  file_start_lba_ =
      fs_.database + (static_cast<LBA_t>(file_.obj.sclust - 2u) * fs_.csize);
  file_capacity_bytes_ = static_cast<uint32_t>(bytes);
  file_ready_ = true;
}

// A preallocation from a boot that never armed: full size, no ULog magic. It
// came from our own f_expand, so it is still contiguous and reusing it spares
// the card a full FAT rewrite every boot; a crashed flight keeps its magic.
bool LogService::TryReuseEmptyLog(uint32_t index, FSIZE_t bytes) {
  FormatLogName(index);
  if (f_open(&file_, file_name_, FA_WRITE | FA_READ) != FR_OK) {
    return false;
  }
  // The directory entry vouched for the size, so a short read can only be an
  // I/O error, which comes back as a bad FRESULT.
  uint8_t head[sizeof(kUlogMagic)];
  UINT read = 0;
  if (f_read(&file_, head, sizeof(head), &read) != FR_OK ||
      std::memcmp(head, kUlogMagic, sizeof(kUlogMagic)) == 0) {
    f_close(&file_);
    return false;
  }
  AdoptOpenFile(bytes);
  fc_link_->SendLog("sd: %s reused", file_name_);
  return true;
}

// Lifts a source's 32-bit stamp onto the log's 64-bit timeline. Sources are at
// most seconds old, so the wrap-safe delta back from the last poll is exact.
// Zero means never written, so it reads as now.
uint64_t LogService::Stamp64(uint32_t src_us) const {
  if (src_us == 0u) {
    return total_us_;
  }
  // Signed: an interrupt can write a source between this tick's Now64 and the
  // read below, leaving it newer than the poll. Unsigned, that few-microsecond
  // lead wraps to ~2^32 and drives the stamp below zero.
  const int32_t age_us = static_cast<int32_t>(last_now_us_ - src_us);
  if (age_us <= 0) {
    return total_us_;
  }
  return total_us_ - static_cast<uint32_t>(age_us);
}

uint64_t LogService::Now64(uint32_t now_us) {
  if (!time_seeded_) {
    total_us_ = now_us;
    last_now_us_ = now_us;
    time_seeded_ = true;
  }
  total_us_ += static_cast<uint32_t>(now_us - last_now_us_);
  last_now_us_ = now_us;
  return total_us_;
}

namespace {

SensorFifoRecord MakeFifoRecord(uint16_t msg_id, const ImuBurst &burst,
                                uint32_t latency_us, float scale,
                                const int32_t (&axes)[3][kImuMaxSamples]) {
  SensorFifoRecord rec =
      MakeRecord<SensorFifoRecord>(msg_id, burst.timestamp_us + latency_us);
  rec.timestamp_sample = burst.timestamp_us;
  rec.device_id = burst.device_id;
  rec.dt = burst.dt_us;
  rec.scale = scale;
  rec.samples = burst.count;
  std::memcpy(rec.x, axes[0], sizeof(rec.x));
  std::memcpy(rec.y, axes[1], sizeof(rec.y));
  std::memcpy(rec.z, axes[2], sizeof(rec.z));
  return rec;
}

}  // namespace

// Called from the control tick while the mailbox still reads fresh, so the
// interrupt cannot be writing the slot underneath. Sole producer into the ring.
void LogService::PushRawImu() {
  if constexpr (!kRawImuLogEnabled) {
    return;
  }
  if (!logging_) {
    return;
  }
  const ImuBurstSlot &slot = blackboard_->GetImuBurstSlot();
  if (!slot.fresh || slot.burst.count == 0u) {
    return;
  }
  const ImuBurst &burst = slot.burst;
  // `timestamp` is when the record was produced, `timestamp_sample` when the
  // chip took the reading -- PX4's pair, and the gap between them is the FIFO
  // read plus the wait for this tick. Taken as a 32-bit delta so it stays
  // wrap-safe without reading the log's 64-bit clock, which the main tick owns
  // and this runs in PendSV.
  //
  // Signed, and floored at zero: the sample stamp is servo-tracked against the
  // chip's clock and can lead TIM2 by a few hundred microseconds, which as an
  // unsigned delta would be a 4295-second jump instead of a small negative.
  const int32_t elapsed_us =
      static_cast<int32_t>(System::GetInstance().Time().Micros() -
                           static_cast<uint32_t>(burst.timestamp_us));
  const uint32_t latency_us =
      (elapsed_us > 0) ? static_cast<uint32_t>(elapsed_us) : 0u;
  const SensorFifoRecord gyro = MakeFifoRecord(
      kMsgSensorGyroFifo, burst, latency_us, burst.gyro_scale, burst.gyro);
  const SensorFifoRecord accel = MakeFifoRecord(
      kMsgSensorAccelFifo, burst, latency_us, burst.accel_scale, burst.accel);

  // Both or neither: they describe one FIFO read, and a reader that found the
  // accel without the gyro would be reading a gap as a measurement.
  const size_t needed = sizeof(gyro) + sizeof(accel);
  if ((ring_.Capacity() - ring_.Available()) < needed) {
    stats_.dropped_bytes += needed;
    return;
  }
  ring_.PushBlock(reinterpret_cast<const uint8_t *>(&gyro), sizeof(gyro));
  ring_.PushBlock(reinterpret_cast<const uint8_t *>(&accel), sizeof(accel));
}

void LogService::AppendToStaging(const void *data, size_t len) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  while (len > 0u) {
    if (fill_len_ == kStagingBytes) {
      TryStartFlush();
      if (fill_len_ == kStagingBytes) {
        // Unreachable from record paths: AppendSlowTopics refuses a tick
        // that does not fit, StartFlight writes into empty buffers.
        stats_.dropped_bytes += len;
        return;
      }
    }
    const size_t room = kStagingBytes - fill_len_;
    const size_t chunk = (len < room) ? len : room;
    std::memcpy(&staging_[fill_index_][fill_len_], bytes, chunk);
    fill_len_ += chunk;
    log_bytes_ += chunk;
    bytes += chunk;
    len -= chunk;
    if (fill_len_ == kStagingBytes) {
      TryStartFlush();
    }
  }
}

void LogService::AppendUlogMessage(uint8_t type, const void *payload,
                                   uint16_t len) {
  const MsgHeader hdr{len, type};
  AppendToStaging(&hdr, sizeof(hdr));
  AppendToStaging(payload, len);
}

void LogService::AppendDefinitions(uint64_t now64) {
  AppendToStaging(kUlogMagic, sizeof(kUlogMagic));
  AppendToStaging(&now64, sizeof(now64));

  const FlagBits flags{};
  AppendUlogMessage('B', &flags, sizeof(flags));

  for (const char *fmt : kFormats) {
    AppendUlogMessage('F', fmt, static_cast<uint16_t>(std::strlen(fmt)));
  }

  // Info message: {uint8 key_len, "type name", value}.
  constexpr char kSysNameKey[] = "char[7] sys_name";
  constexpr char kSysNameValue[] = "32raven";
  uint8_t info[1 + sizeof(kSysNameKey) - 1 + sizeof(kSysNameValue) - 1];
  info[0] = sizeof(kSysNameKey) - 1;
  std::memcpy(&info[1], kSysNameKey, sizeof(kSysNameKey) - 1);
  std::memcpy(&info[1 + sizeof(kSysNameKey) - 1], kSysNameValue,
              sizeof(kSysNameValue) - 1);
  AppendUlogMessage('I', info, sizeof(info));

  // Subscriptions: {uint8 multi_id, uint16 msg_id, name}.
  for (uint16_t id = 0; id < kMsgCount; ++id) {
    uint8_t sub[3 + 16];
    sub[0] = 0;
    std::memcpy(&sub[1], &id, sizeof(id));
    const size_t name_len = std::strlen(kTopicNames[id]);
    std::memcpy(&sub[3], kTopicNames[id], name_len);
    AppendUlogMessage('A', sub, static_cast<uint16_t>(3 + name_len));
  }
}

void LogService::AppendSlowTopics(uint64_t now64, uint32_t now_us) {
  // All-or-nothing per tick: a record split across a flush that cannot start
  // tears the stream and poisons everything after it. Skipped topics stay due
  // and retry next tick.
  constexpr size_t kWorstCaseBytes =
      sizeof(RcRecord) + sizeof(BatteryRecord) + sizeof(EscTelemetryRecord) +
      sizeof(GpsRecord) + sizeof(ImuHealthRecord) + sizeof(CrsfLinkRecord) +
      sizeof(LoggerRecord);
  static_assert(kWorstCaseBytes < kStagingBytes);
  if (dma_busy_ && (kStagingBytes - fill_len_) < kWorstCaseBytes) {
    return;
  }

  for (size_t slot = 0; slot < kSlowTopicCount; ++slot) {
    if (!slow_sched_.IsDue(slot, now_us)) {
      continue;
    }
    slow_sched_.MarkSent(slot, now_us);

    switch (static_cast<MsgId>(slot + kMsgPushedCount)) {
      case kMsgRc: {
        const RcData &rc = blackboard_->GetRc();
        RcRecord rec = MakeRecord<RcRecord>(kMsgRc, Stamp64(rc.timestamp_us));
        static_assert(sizeof(rec.channels) == sizeof(rc.channels_raw));
        std::memcpy(rec.channels, rc.channels_raw.data(), sizeof(rec.channels));
        rec.roll_us = rc.roll_us;
        rec.pitch_us = rc.pitch_us;
        rec.yaw_us = rc.yaw_us;
        rec.throttle_us = rc.throttle_us;
        AppendToStaging(&rec, sizeof(rec));
        break;
      }
      case kMsgBattery: {
        const BatteryData &bat = blackboard_->GetBattery();
        BatteryRecord rec = MakeRecord<BatteryRecord>(
            kMsgBattery, Stamp64(bat.timestamp_us));
        rec.voltage = bat.voltage;
        rec.current = bat.current.value_or(kMissingFloat);
        rec.mah_drawn = bat.mah_drawn.value_or(kMissingFloat);
        rec.remaining = bat.percentage;
        AppendToStaging(&rec, sizeof(rec));
        break;
      }
      case kMsgEscTelemetry: {
        const EscTelemetryData &esc = blackboard_->GetEscTelemetry();
        // Four motors, four stamps, one slot: picking one dates the record
        // by a motor the reader cannot identify, so this stays on poll time
        // until the record carries all four (#33).
        EscTelemetryRecord rec = MakeRecord<EscTelemetryRecord>(
            kMsgEscTelemetry, now64);
        for (int m = 0; m < 4; ++m) {
          rec.rpm[m] = esc.motors[m].rpm;
          rec.voltage[m] = esc.motors[m].voltage;
          rec.current[m] = esc.motors[m].current.value_or(kMissingFloat);
          rec.temperature_c[m] = esc.motors[m].temperature_c;
        }
        rec.valid_mask = esc.valid_mask;
        AppendToStaging(&rec, sizeof(rec));
        break;
      }
      case kMsgGps: {
        const GpsData &gps = blackboard_->GetGps();
        GpsRecord rec = MakeRecord<GpsRecord>(
            kMsgGps, Stamp64(gps.timestamp_us));
        rec.lat_1e7 = gps.lat;
        rec.lon_1e7 = gps.lon;
        rec.alt_mm = gps.alt;
        rec.vel_cms = gps.vel;
        rec.hdg_cdeg = gps.hdg;
        rec.hdop = gps.hDOP;
        rec.fix_type = gps.fix_type;
        rec.num_sats = gps.num_sats;
        AppendToStaging(&rec, sizeof(rec));
        break;
      }
      case kMsgImuHealth: {
        const ImuHealth &imu = blackboard_->GetImuHealth();
        ImuHealthRecord rec = MakeRecord<ImuHealthRecord>(
            kMsgImuHealth, Stamp64(imu.timestamp_us));
        rec.publish_count = imu.publish_count;
        rec.path_faults = imu.path_faults;
        rec.overruns = imu.overruns;
        // From the bus, which owns them: path_faults above folds them into its
        // total, but the split belongs to SPI2 rather than to the IMU.
        const SpiFaults &spi = blackboard_->GetSystemHealth().imu_spi;
        rec.dma_start_fails = spi.start_refused;
        rec.spi_errors = spi.dma_errors;
        rec.parse_fails = imu.parse_fails;
        rec.dropped_records = imu.dropped_records;
        rec.missed_samples = imu.missed_samples;
        AppendToStaging(&rec, sizeof(rec));
        break;
      }
      case kMsgCrsfLink: {
        const CrsfLinkData &link = blackboard_->GetCrsfLink();
        CrsfLinkRecord rec = MakeRecord<CrsfLinkRecord>(
            kMsgCrsfLink, Stamp64(link.timestamp_us));
        rec.uplink_rssi_ant1_dbm = link.uplink_rssi_ant1_dbm;
        rec.uplink_rssi_ant2_dbm = link.uplink_rssi_ant2_dbm;
        rec.uplink_link_quality = link.uplink_link_quality;
        rec.uplink_snr_db = link.uplink_snr_db;
        rec.rf_mode = link.rf_mode;
        rec.uplink_tx_power_index = link.uplink_tx_power_index;
        rec.downlink_rssi_dbm = link.downlink_rssi_dbm;
        rec.downlink_link_quality = link.downlink_link_quality;
        rec.downlink_snr_db = link.downlink_snr_db;
        AppendToStaging(&rec, sizeof(rec));
        break;
      }
      case kMsgLogger: {
        LoggerRecord rec = MakeRecord<LoggerRecord>(kMsgLogger, now64);
        rec.dropped_bytes = stats_.dropped_bytes;
        rec.write_errors = stats_.write_errors;
        AppendToStaging(&rec, sizeof(rec));
        break;
      }
      case kMsgCount:
        break;
    }
  }
}

void LogService::StageFromRing() {
  // Sampled before draining, so it is the depth the producer actually reached.
  const uint32_t pending = static_cast<uint32_t>(ring_.Available());
  if (pending > stats_.ring_peak_bytes) {
    stats_.ring_peak_bytes = pending;
  }
  while (fill_len_ < kStagingBytes) {
    const auto readable = ring_.ContiguousReadable();
    if (readable.empty()) {
      return;
    }
    const size_t room = kStagingBytes - fill_len_;
    const size_t chunk = (readable.size() < room) ? readable.size() : room;
    std::memcpy(&staging_[fill_index_][fill_len_], readable.data(), chunk);
    fill_len_ += chunk;
    log_bytes_ += chunk;
    ring_.Consume(chunk);
    if (fill_len_ == kStagingBytes) {
      TryStartFlush();
    }
  }
}

void LogService::TryStartFlush() {
  if (dma_busy_ || fill_len_ < kStagingBytes || sink_failed_) {
    return;
  }
  if ((flushed_bytes_ + kStagingBytes) > file_capacity_bytes_) {
    FailSink();
    fc_link_->SendLog("sd: %s full", file_name_);
    return;
  }
  const uint32_t lba = file_start_lba_ + (flushed_bytes_ / Sdio::kBlockBytes);
  if (!Sdio::GetInstance().StartWrite(lba, staging_[fill_index_])) {
    FailSink();
    return;
  }
  dma_busy_ = true;
  flush_start_us_ = System::GetInstance().Time().Micros();
  fill_index_ ^= 1u;
  fill_len_ = 0;
}

void LogService::FinishFlush(bool ok) {
  dma_busy_ = false;
  // The card's own stalls -- erase, wear levelling -- land here as write time,
  // and the ring is all that stands between one and a dropped record. Hence
  // the worst rather than the mean, and a histogram to say how often.
  const uint32_t elapsed_us =
      System::GetInstance().Time().Micros() - flush_start_us_;
  ++stats_.flush_count;
  stats_.flush_total_us += elapsed_us;
  if (elapsed_us > stats_.flush_max_us) {
    stats_.flush_max_us = elapsed_us;
  }
  size_t bucket = 0;
  for (uint32_t edge = 1000u; bucket < 7u && elapsed_us >= edge; edge *= 2u) {
    ++bucket;
  }
  ++stats_.flush_hist[bucket];

  if (ok) {
    flushed_bytes_ += kStagingBytes;
  } else {
    FailSink();
  }
}

// The one wait PollWrite cannot feed the watchdog through, and disarm is
// exactly when a stalled card would otherwise reset the board.
void LogService::DrainFlush() {
  Sdio &sd = Sdio::GetInstance();
  while (dma_busy_) {
    Watchdog::GetInstance().Kick();
    const Sdio::WriteStatus status = sd.PollWrite();
    if (status == Sdio::WriteStatus::kDone) {
      FinishFlush(true);
    } else if (status == Sdio::WriteStatus::kError) {
      FinishFlush(false);
    }
  }
}

void LogService::FailSink() {
  logging_ = false;
  sink_failed_ = true;
  ++stats_.write_errors;
}

void LogService::StartFlight(uint32_t now_us) {
  if (session_open_ || !file_ready_) {
    return;
  }
  session_start_us_ = now_us;
  ring_.Clear();
  fill_index_ = 0;
  fill_len_ = 0;
  dma_busy_ = false;
  sink_failed_ = false;
  log_bytes_ = 0;
  flushed_bytes_ = 0;
  stats_ = {};

  slow_sched_.Init(slow_configs_, slow_states_, 1000u, now_us);

  AppendDefinitions(Now64(now_us));
  session_open_ = true;
  logging_ = true;
}

void LogService::StopFlight() {
  if (!session_open_) {
    return;
  }
  logging_ = false;
  session_open_ = false;

  DrainFlush();

  if (!sink_failed_) {
    StageFromRing();
    // StageFromRing may have started a flush, and the blocking tail write
    // below is refused while one runs.
    DrainFlush();
    // Padded to a sector on the card, then cut back to the exact stream
    // length below, so a reader never sees the padding.
    if (fill_len_ > 0u) {
      const size_t tail = fill_len_;
      std::memset(&staging_[fill_index_][tail], 0, kStagingBytes - tail);
      const size_t padded =
          ((tail + Sdio::kBlockBytes - 1u) / Sdio::kBlockBytes) *
          Sdio::kBlockBytes;
      const uint32_t lba =
          file_start_lba_ + (flushed_bytes_ / Sdio::kBlockBytes);
      if ((flushed_bytes_ + padded) <= file_capacity_bytes_ &&
          Sdio::GetInstance().WriteBlocks(
              lba, std::span{staging_[fill_index_]}.first(padded))) {
        flushed_bytes_ += static_cast<uint32_t>(tail);
      } else {
        FailSink();
      }
    }
  }

  const uint32_t mean_us =
      stats_.flush_count ? stats_.flush_total_us / stats_.flush_count : 0u;
  fc_link_->SendLog(
      "log: %lu B %lu wr mean=%luus max=%luus ring=%lu/%lu drop=%lu err=%lu",
      static_cast<unsigned long>(log_bytes_),
      static_cast<unsigned long>(stats_.flush_count),
      static_cast<unsigned long>(mean_us),
      static_cast<unsigned long>(stats_.flush_max_us),
      static_cast<unsigned long>(stats_.ring_peak_bytes),
      static_cast<unsigned long>(kRingBytes),
      static_cast<unsigned long>(stats_.dropped_bytes),
      static_cast<unsigned long>(stats_.write_errors));
  fc_link_->SendLog("log: ms<1 %u %u %u %u %u %u %u >=64 %u",
                    stats_.flush_hist[0], stats_.flush_hist[1],
                    stats_.flush_hist[2], stats_.flush_hist[3],
                    stats_.flush_hist[4], stats_.flush_hist[5],
                    stats_.flush_hist[6], stats_.flush_hist[7]);

  const uint32_t final_bytes = sink_failed_ ? flushed_bytes_ : log_bytes_;
  // Best effort, never a panic: disarm may be a landing. A file left full-size
  // keeps its magic, so it reads as a flight log, not a reusable preallocation.
  FRESULT fin_res = f_lseek(&file_, final_bytes);
  if (fin_res == FR_OK) {
    fin_res = f_truncate(&file_);
  }
  if (fin_res != FR_OK) {
    LogSdFailure(*fc_link_, "finalize", fin_res);
  }
  const FRESULT close_res = f_close(&file_);
  if (close_res != FR_OK) {
    LogSdFailure(*fc_link_, "finalize close", close_res);
  }
  fc_link_->SendLog(
      "sd: %s closed (%lu KB, %lu dropped)", file_name_,
      static_cast<unsigned long>(final_bytes >> 10),
      static_cast<unsigned long>(stats_.dropped_bytes));

  PrepareNextFile();
}

void LogService::ReleaseCard() {
  StopFlight();
  if (file_ready_) {
    f_close(&file_);
    file_ready_ = false;
  }
  if (mounted_) {
    f_unmount("");
    mounted_ = false;
  }
}

void LogService::RemountAfterMsc() {
  file_ready_ = false;
  mounted_ = false;
  if (!Sdio::GetInstance().Reprobe()) {
    fc_link_->SendLog("sd: no card after msc");
    Panic(ErrorCode::Stm32::kSdCardMissing);
  }
  if (f_mount(&fs_, "", 1) != FR_OK) {
    fc_link_->SendLog("sd: remount failed");
    Panic(ErrorCode::Stm32::kSdCardCorrupted);
  }
  mounted_ = true;
  PrepareNextFile();
}

void LogService::ListLogs(uint8_t first, message::LogListReplyMsg &out) {
  out = {};
  out.first = first;
  if (!mounted_) {
    out.status = static_cast<uint8_t>(message::LogStatus::kBusy);
    return;
  }

  DIR dir{};
  FILINFO info{};
  if (f_opendir(&dir, "/") != FR_OK) {
    out.status = static_cast<uint8_t>(message::LogStatus::kIoError);
    return;
  }
  uint8_t index = 0;
  uint32_t scanned = 0;
  while (true) {
    ClearDirEntry(info);
    if (f_readdir(&dir, &info) != FR_OK || info.fname[0] == '\0') {
      break;
    }
    // Same runaway cap as the boot scan, but this request has an error
    // channel, so the puller gets IoError instead of a panic.
    if (++scanned > kMaxRootScanEntries) {
      LogSdFailure(*fc_link_, "list runaway", FR_OK);
      f_closedir(&dir);
      out.status = static_cast<uint8_t>(message::LogStatus::kIoError);
      return;
    }
    if (LogFileIndex(info.fname) == 0u) {
      continue;
    }
    // The open preallocated file is next flight's, not a log yet.
    if (file_ready_ && std::strcmp(info.fname, file_name_) == 0) {
      continue;
    }
    if (index >= first && out.count < message::kLogListMaxEntries) {
      message::LogListEntry &entry = out.entries[out.count];
      std::memcpy(entry.name, info.fname, message::kLogNameLen);
      entry.size_bytes = static_cast<uint32_t>(info.fsize);
      ++out.count;
    }
    ++index;
  }
  f_closedir(&dir);
  out.total = index;
  out.status = static_cast<uint8_t>(message::LogStatus::kOk);
}

void LogService::ReadLog(const message::LogReadMsg &req,
                         message::LogDataMsg &out) {
  out = {};
  out.offset = req.offset;
  if (!mounted_) {
    out.status = static_cast<uint8_t>(message::LogStatus::kBusy);
    return;
  }

  char name[13] = {};
  std::memcpy(name, req.name, message::kLogNameLen);
  if (read_open_ && std::strcmp(name, read_name_) != 0) {
    f_close(&read_file_);
    read_open_ = false;
  }
  if (!read_open_) {
    if (f_open(&read_file_, name, FA_READ) != FR_OK) {
      out.status = static_cast<uint8_t>(message::LogStatus::kNotFound);
      return;
    }
    std::memcpy(read_name_, name, sizeof(read_name_));
    read_open_ = true;
  }

  const uint16_t len = (req.len > message::kLogDataMaxBytes)
                           ? message::kLogDataMaxBytes
                           : req.len;
  UINT read = 0;
  if (f_lseek(&read_file_, req.offset) != FR_OK ||
      f_read(&read_file_, out.data, len, &read) != FR_OK) {
    f_close(&read_file_);
    read_open_ = false;
    out.status = static_cast<uint8_t>(message::LogStatus::kIoError);
    return;
  }
  out.len = static_cast<uint16_t>(read);
  out.status = static_cast<uint8_t>(message::LogStatus::kOk);
}

void LogService::Poll(uint32_t now_us) {
  if (prepare_pending_) {
    prepare_pending_ = false;
    PrepareNextFile();
  }

  const uint64_t now64 = Now64(now_us);

  if (!logging_) {
    return;
  }

  if (dma_busy_) {
    const Sdio::WriteStatus status = Sdio::GetInstance().PollWrite();
    if (status == Sdio::WriteStatus::kDone) {
      FinishFlush(true);
    } else if (status == Sdio::WriteStatus::kError) {
      FinishFlush(false);
      return;
    }
  }

  StageFromRing();
  AppendSlowTopics(now64, now_us);
  TryStartFlush();
}
