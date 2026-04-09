#include "config_storage.hpp"

#include <cstring>

#include "message.hpp"
#include "panic.hpp"

namespace {

static_assert(ee_schema::layout::kTotalSize <= EE::kCapacity);

struct LegacyRcCalibrationV1 {
  static constexpr uint32_t kMagic = 0x31434352u;
  static constexpr uint32_t kSchemaHash = 0x1F7CC2B2u;

  uint32_t magic;
  uint32_t schema_hash;
  uint32_t size;
  uint16_t min_us[4];
  uint16_t max_us[4];
};

static_assert(sizeof(LegacyRcCalibrationV1) == 28);

bool IsErased(const void *data, size_t len) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < len; ++i) {
    if (bytes[i] != 0xFFu) {
      return false;
    }
  }
  return true;
}

ee_schema::ImuAccelCalibration MakeDefaultImuAccelCalibration() {
  ee_schema::ImuAccelCalibration cal{};
  ee_schema::ImuAccelCalibration::PopulateHeader(cal);
  cal.offsets[0] = 0.0f;
  cal.offsets[1] = 0.0f;
  cal.offsets[2] = 0.0f;
  cal.gains[0] = 1.0f;
  cal.gains[1] = 1.0f;
  cal.gains[2] = 1.0f;
  return cal;
}

ee_schema::RcCalibration MakeDefaultRcCalibration() {
  ee_schema::RcCalibration cal{};
  ee_schema::RcCalibration::PopulateHeader(cal);
  for (size_t i = 0; i < 16; ++i) {
    cal.min_us[i] = 1000;
    cal.max_us[i] = 2000;
    cal.trim_us[i] = 1500;
    cal.rev[i] = 1;
  }
  return cal;
}

ee_schema::RcMap MakeDefaultRcMap(const ee_schema::RcMap &fallback) {
  ee_schema::RcMap map = fallback;
  ee_schema::RcMap::PopulateHeader(map);
  return map;
}

message::RcCalibrationConfigMsg ToRcCalibrationConfig(
    const ee_schema::RcCalibration &cal) {
  message::RcCalibrationConfigMsg cfg{};
  static_assert(sizeof(cfg.min_us) == sizeof(cal.min_us));
  static_assert(sizeof(cfg.max_us) == sizeof(cal.max_us));
  static_assert(sizeof(cfg.trim_us) == sizeof(cal.trim_us));
  static_assert(sizeof(cfg.rev) == sizeof(cal.rev));
  std::memcpy(cfg.min_us, cal.min_us, sizeof(cfg.min_us));
  std::memcpy(cfg.max_us, cal.max_us, sizeof(cfg.max_us));
  std::memcpy(cfg.trim_us, cal.trim_us, sizeof(cfg.trim_us));
  std::memcpy(cfg.rev, cal.rev, sizeof(cfg.rev));
  return cfg;
}

message::RcMapConfigMsg ToRcMapConfig(const ee_schema::RcMap &map) {
  return {
      .roll = map.roll_channel,
      .pitch = map.pitch_channel,
      .yaw = map.yaw_channel,
      .throttle = map.throttle_channel,
  };
}

bool IsLegacyRcCalibrationV1(const ee_schema::RcCalibration &cal) {
  LegacyRcCalibrationV1 legacy{};
  std::memcpy(&legacy, &cal, sizeof(legacy));
  return legacy.magic == LegacyRcCalibrationV1::kMagic &&
         legacy.schema_hash == LegacyRcCalibrationV1::kSchemaHash &&
         legacy.size == sizeof(LegacyRcCalibrationV1);
}

bool IsRcCalibrationValid(const ee_schema::RcCalibration &cal) {
  return message::IsRcCalibrationConfigValid(ToRcCalibrationConfig(cal));
}

bool IsRcMapValid(const ee_schema::RcMap &map) {
  return message::IsRcMapConfigValid(ToRcMapConfig(map));
}

}  // namespace

ee_schema::ImuAccelCalibration ConfigStorage::LoadOrInitImuAccelCalibration(
    EE &ee) {
  ee_schema::ImuAccelCalibration cal{};
  if (!ee.ReadObject(cal, ee_schema::layout::kImuAccelCalibrationOffset)) {
    Panic(ErrorCode::kEepromInvalidConfig);
  }

  if (IsErased(&cal, sizeof(cal))) {
    cal = MakeDefaultImuAccelCalibration();
    if (!ee.WriteObject(cal, ee_schema::layout::kImuAccelCalibrationOffset)) {
      Panic(ErrorCode::kEepromWriteFailed);
    }
    return cal;
  }

  if (!ee_schema::ImuAccelCalibration::IsExactSchema(cal)) {
    Panic(ErrorCode::kEepromSchemaMismatch);
  }

  return cal;
}

bool ConfigStorage::SaveImuAccelCalibration(
    EE &ee, const ee_schema::ImuAccelCalibration &cal) {
  ee_schema::ImuAccelCalibration to_write = cal;
  ee_schema::ImuAccelCalibration::PopulateHeader(to_write);
  return ee.WriteObject(to_write,
                        ee_schema::layout::kImuAccelCalibrationOffset);
}

ee_schema::RcCalibration ConfigStorage::LoadOrInitRcCalibration(EE &ee) {
  ee_schema::RcCalibration cal{};
  if (!ee.ReadObject(cal, ee_schema::layout::kRcCalibrationOffset)) {
    Panic(ErrorCode::kEepromInvalidConfig);
  }

  if (IsErased(&cal, sizeof(cal))) {
    cal = MakeDefaultRcCalibration();
    if (!ee.WriteObject(cal, ee_schema::layout::kRcCalibrationOffset)) {
      Panic(ErrorCode::kEepromWriteFailed);
    }
    return cal;
  }

  if (!ee_schema::RcCalibration::IsExactSchema(cal)) {
    if (!IsLegacyRcCalibrationV1(cal)) {
      Panic(ErrorCode::kEepromSchemaMismatch);
    }

    LegacyRcCalibrationV1 legacy{};
    std::memcpy(&legacy, &cal, sizeof(legacy));
    ee_schema::RcCalibration migrated = MakeDefaultRcCalibration();
    for (size_t i = 0; i < 4; ++i) {
      migrated.min_us[i] = legacy.min_us[i];
      migrated.max_us[i] = legacy.max_us[i];
      migrated.trim_us[i] =
          (uint16_t)((legacy.min_us[i] + legacy.max_us[i]) / 2u);
    }
    if (!ee.WriteObject(migrated, ee_schema::layout::kRcCalibrationOffset)) {
      Panic(ErrorCode::kEepromWriteFailed);
    }
    return migrated;
  }

  if (!IsRcCalibrationValid(cal)) {
    cal = MakeDefaultRcCalibration();
    if (!ee.WriteObject(cal, ee_schema::layout::kRcCalibrationOffset)) {
      Panic(ErrorCode::kEepromWriteFailed);
    }
  }

  return cal;
}

bool ConfigStorage::SaveRcCalibration(EE &ee,
                                      const ee_schema::RcCalibration &cal) {
  if (!IsRcCalibrationValid(cal)) {
    return false;
  }
  ee_schema::RcCalibration to_write = cal;
  ee_schema::RcCalibration::PopulateHeader(to_write);
  return ee.WriteObject(to_write, ee_schema::layout::kRcCalibrationOffset);
}

ee_schema::RcMap ConfigStorage::LoadOrInitRcMap(
    EE &ee, const ee_schema::RcMap &default_map) {
  if (!IsRcMapValid(default_map)) {
    Panic(ErrorCode::kRcReceiverInvalidConfig);
  }

  ee_schema::RcMap map{};
  if (!ee.ReadObject(map, ee_schema::layout::kRcMapOffset)) {
    Panic(ErrorCode::kEepromInvalidConfig);
  }

  if (IsErased(&map, sizeof(map))) {
    map = MakeDefaultRcMap(default_map);
    if (!ee.WriteObject(map, ee_schema::layout::kRcMapOffset)) {
      Panic(ErrorCode::kEepromWriteFailed);
    }
    return map;
  }

  if (!ee_schema::RcMap::IsExactSchema(map)) {
    Panic(ErrorCode::kEepromSchemaMismatch);
  }

  if (!IsRcMapValid(map)) {
    map = MakeDefaultRcMap(default_map);
    if (!ee.WriteObject(map, ee_schema::layout::kRcMapOffset)) {
      Panic(ErrorCode::kEepromWriteFailed);
    }
  }

  return map;
}

bool ConfigStorage::SaveRcMap(EE &ee, const ee_schema::RcMap &map) {
  if (!IsRcMapValid(map)) {
    return false;
  }
  ee_schema::RcMap to_write = map;
  ee_schema::RcMap::PopulateHeader(to_write);
  return ee.WriteObject(to_write, ee_schema::layout::kRcMapOffset);
}
