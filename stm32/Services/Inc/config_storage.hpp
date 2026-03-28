#pragma once

#include "ee.hpp"
#include "ee_schema.hpp"

class ConfigStorage {
public:
  static ee_schema::ImuAccelCalibration LoadOrInitImuAccelCalibration(EE &ee);
  static ee_schema::RcCalibration LoadOrInitRcCalibration(EE &ee);
  static bool
  SaveImuAccelCalibration(EE &ee, const ee_schema::ImuAccelCalibration &cal);
  static bool SaveRcCalibration(EE &ee, const ee_schema::RcCalibration &cal);
};
