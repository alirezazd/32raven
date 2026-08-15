// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "ee.hpp"
#include "ee_schema.hpp"

class EeConfigStorage {
 public:
  static ee_schema::ImuAccelCalibration LoadOrInitImuAccelCalibration(EE &ee);
  static ee_schema::RcCalibration LoadOrInitRcCalibration(EE &ee);
  static ee_schema::RcMap LoadOrInitRcMap(EE &ee,
                                          const ee_schema::RcMap &default_map);
  static bool SaveImuAccelCalibration(
      EE &ee, const ee_schema::ImuAccelCalibration &cal);
  static bool SaveRcCalibration(EE &ee, const ee_schema::RcCalibration &cal);
  static bool SaveRcMap(EE &ee, const ee_schema::RcMap &map);
};
