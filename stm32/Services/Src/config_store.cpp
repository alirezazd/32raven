#include "config_store.hpp"

#include <cstring>

#include "panic.hpp"

namespace {

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

} // namespace

ee_schema::ImuAccelCalibration
ConfigStore::LoadOrInitImuAccelCalibration(EE &ee) {
  ee_schema::ImuAccelCalibration cal{};
  if (!ee.ReadObject(cal, 0)) {
    Panic(ErrorCode::kEepromInvalidConfig);
  }

  if (IsErased(&cal, sizeof(cal))) {
    cal = MakeDefaultImuAccelCalibration();
    if (!ee.WriteObject(cal, 0)) {
      Panic(ErrorCode::kEepromWriteFailed);
    }
    return cal;
  }

  if (!ee_schema::ImuAccelCalibration::IsExactSchema(cal)) {
    Panic(ErrorCode::kEepromSchemaMismatch);
  }

  return cal;
}

bool ConfigStore::SaveImuAccelCalibration(
    EE &ee, const ee_schema::ImuAccelCalibration &cal) {
  ee_schema::ImuAccelCalibration to_write = cal;
  ee_schema::ImuAccelCalibration::PopulateHeader(to_write);
  return ee.WriteObject(to_write, 0);
}
