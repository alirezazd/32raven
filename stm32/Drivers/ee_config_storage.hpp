// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "ee.hpp"
#include "ee_schema.hpp"
#include "error_code.hpp"
#include "panic.hpp"

// The records in the EE image, found by tag. The image is a run of records
// back to back, each starting with ee_schema::RecordHeader, and ends at the
// first erased header; a record's place in it is wherever the walk finds it,
// so nothing here holds an offset.
class EeConfigStorage {
 public:
  // Boot, before any record is read. A record this firmware no longer knows
  // is dropped and the rest are packed -- one rewrite, so a torn one leaves
  // the previous image the newest and this runs again.
  static void Normalize(EE &ee);

  static ee_schema::ImuAccelCalibration LoadOrInitImuAccelCalibration(EE &ee);
  static ee_schema::ImuGyroCalibration LoadOrInitImuGyroCalibration(EE &ee);
  static ee_schema::MagnetometerCalibration LoadOrInitMagnetometerCalibration(
      EE &ee);
  static ee_schema::BoardTrim LoadOrInitBoardTrim(EE &ee);
  static ee_schema::RcMap LoadOrInitRcMap(EE &ee,
                                          const ee_schema::RcMap &default_map);
  static bool SaveImuAccelCalibration(
      EE &ee, const ee_schema::ImuAccelCalibration &cal);
  static bool SaveImuGyroCalibration(EE &ee,
                                     const ee_schema::ImuGyroCalibration &cal);
  static bool SaveMagnetometerCalibration(
      EE &ee, const ee_schema::MagnetometerCalibration &cal);
  static bool SaveBoardTrim(EE &ee, const ee_schema::BoardTrim &trim);
  static bool SaveRcMap(EE &ee, const ee_schema::RcMap &map);

 private:
  // The stored image as walked: where each known record sits, where the run
  // ends, and whether anything unknown was passed on the way.
  struct Walk {
    struct Entry {
      uint32_t magic;
      EE::Segment place;
    };
    Entry found[ee_schema::kKnownRecordCount]{};
    size_t found_count = 0;
    size_t end = 0;
    bool foreign = false;

    std::optional<size_t> Find(uint32_t magic) const;
  };

  static Walk WalkImage(const EE &ee);

  // Found: read and checked against the shape this firmware compiled. Not
  // found: the fallback, appended where the run ends.
  template <typename T>
  static T LoadOrInit(EE &ee, const T &fallback) {
    T record{};
    const Walk walk = WalkImage(ee);
    const std::optional<size_t> at = walk.Find(T::kMagic);
    if (!at.has_value()) {
      record = fallback;
      T::PopulateHeader(record);
      if (!ee.WriteObject(record, walk.end)) {
        Panic(ErrorCode::Stm32::kEepromWriteFailed);
      }
      return record;
    }
    if (!ee.ReadObject(record, *at)) {
      Panic(ErrorCode::Stm32::kEepromInvalidConfig);
    }
    if (!T::IsExactSchema(record)) {
      Panic(ErrorCode::Stm32::kEepromSchemaMismatch);
    }
    return record;
  }

  template <typename T>
  static bool Save(EE &ee, const T &record) {
    T to_write = record;
    T::PopulateHeader(to_write);
    const Walk walk = WalkImage(ee);
    return ee.WriteObject(to_write, walk.Find(T::kMagic).value_or(walk.end));
  }
};
