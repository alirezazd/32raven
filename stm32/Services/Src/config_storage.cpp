#include "config_storage.hpp"

#include <cstring>

#include "panic.hpp"

namespace {

static_assert(ee_schema::layout::kTotalSize <= EE::kCapacity);

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
  for (size_t i = 0; i < 4; ++i) {
    cal.min_us[i] = 1000;
    cal.max_us[i] = 2000;
  }
  return cal;
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
    Panic(ErrorCode::kEepromSchemaMismatch);
  }

  return cal;
}

bool ConfigStorage::SaveRcCalibration(EE &ee,
                                      const ee_schema::RcCalibration &cal) {
  ee_schema::RcCalibration to_write = cal;
  ee_schema::RcCalibration::PopulateHeader(to_write);
  return ee.WriteObject(to_write, ee_schema::layout::kRcCalibrationOffset);
}
