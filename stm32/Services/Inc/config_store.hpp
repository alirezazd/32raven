#pragma once

#include "ee.hpp"
#include "ee_schema.hpp"

class ConfigStore {
public:
  static ee_schema::ImuAccelCalibration LoadOrInitImuAccelCalibration(EE &ee);
  static bool SaveImuAccelCalibration(EE &ee,
                                      const ee_schema::ImuAccelCalibration &cal);
};
