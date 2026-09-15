// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "ee_config_storage.hpp"

namespace {

static_assert(ee_schema::kTotalSize <= EE::kCapacity);

constexpr uint32_t kErasedMagic = 0xFFFFFFFFu;

const ee_schema::KnownRecord *FindKnown(uint32_t magic) {
  for (const ee_schema::KnownRecord &known : ee_schema::kKnownRecords) {
    if (known.magic == magic) {
      return &known;
    }
  }
  return nullptr;
}

ee_schema::ImuAccelCalibration DefaultImuAccelCalibration() {
  ee_schema::ImuAccelCalibration cal{};
  cal.gains[0] = 1.0f;
  cal.gains[1] = 1.0f;
  cal.gains[2] = 1.0f;
  return cal;
}

ee_schema::MagnetometerCalibration DefaultMagnetometerCalibration() {
  ee_schema::MagnetometerCalibration cal{};
  cal.diag[0] = 1.0f;
  cal.diag[1] = 1.0f;
  cal.diag[2] = 1.0f;
  return cal;
}

}  // namespace

std::optional<size_t> EeConfigStorage::Walk::Find(uint32_t magic) const {
  for (size_t i = 0; i < found_count; ++i) {
    if (found[i].magic == magic) {
      return found[i].place.offset;
    }
  }
  return std::nullopt;
}

EeConfigStorage::Walk EeConfigStorage::WalkImage(const EE &ee) {
  Walk walk{};
  const uint32_t image_size = ee.Size();
  uint32_t offset = 0;
  while (offset + sizeof(ee_schema::RecordHeader) <= image_size) {
    ee_schema::RecordHeader header{};
    if (!ee.ReadObject(header, offset)) {
      Panic(ErrorCode::Stm32::kEepromInvalidConfig);
    }
    if (header.magic == kErasedMagic) {
      break;
    }
    // The journal's CRC covers these bytes, so a header that cannot be
    // stepped over was written that way, by a firmware with a bug.
    if (header.size < sizeof(ee_schema::RecordHeader) ||
        (header.size % 4u) != 0u || offset + header.size > image_size) {
      Panic(ErrorCode::Stm32::kEepromInvalidConfig);
    }
    if (FindKnown(header.magic) == nullptr) {
      walk.foreign = true;
    } else {
      if (walk.Find(header.magic).has_value() ||
          walk.found_count >= ee_schema::kKnownRecordCount) {
        Panic(ErrorCode::Stm32::kEepromInvalidConfig);
      }
      walk.found[walk.found_count++] =
          Walk::Entry{header.magic, EE::Segment{offset, header.size}};
    }
    offset += header.size;
  }
  walk.end = offset;
  return walk;
}

void EeConfigStorage::Normalize(EE &ee) {
  const Walk walk = WalkImage(ee);
  if (!walk.foreign) {
    return;
  }
  if (walk.found_count == 0u) {
    ee.Format();
    return;
  }

  // The known records, in the order they were found, and nothing else.
  EE::Segment kept[ee_schema::kKnownRecordCount];
  for (size_t i = 0; i < walk.found_count; ++i) {
    kept[i] = walk.found[i].place;
  }
  if (!ee.Rewrite(std::span<const EE::Segment>(kept, walk.found_count))) {
    Panic(ErrorCode::Stm32::kEepromWriteFailed);
  }
}

ee_schema::ImuAccelCalibration EeConfigStorage::LoadOrInitImuAccelCalibration(
    EE &ee) {
  return LoadOrInit(ee, DefaultImuAccelCalibration());
}

ee_schema::ImuGyroCalibration EeConfigStorage::LoadOrInitImuGyroCalibration(
    EE &ee) {
  return LoadOrInit(ee, ee_schema::ImuGyroCalibration{});
}

ee_schema::MagnetometerCalibration
EeConfigStorage::LoadOrInitMagnetometerCalibration(EE &ee) {
  return LoadOrInit(ee, DefaultMagnetometerCalibration());
}

ee_schema::BoardTrim EeConfigStorage::LoadOrInitBoardTrim(EE &ee) {
  return LoadOrInit(ee, ee_schema::BoardTrim{});
}

ee_schema::RcMap EeConfigStorage::LoadOrInitRcMap(
    EE &ee, const ee_schema::RcMap &default_map) {
  return LoadOrInit(ee, default_map);
}

bool EeConfigStorage::SaveImuAccelCalibration(
    EE &ee, const ee_schema::ImuAccelCalibration &cal) {
  return Save(ee, cal);
}

bool EeConfigStorage::SaveImuGyroCalibration(
    EE &ee, const ee_schema::ImuGyroCalibration &cal) {
  return Save(ee, cal);
}

bool EeConfigStorage::SaveMagnetometerCalibration(
    EE &ee, const ee_schema::MagnetometerCalibration &cal) {
  return Save(ee, cal);
}

bool EeConfigStorage::SaveBoardTrim(EE &ee, const ee_schema::BoardTrim &trim) {
  return Save(ee, trim);
}

bool EeConfigStorage::SaveRcMap(EE &ee, const ee_schema::RcMap &map) {
  return Save(ee, map);
}
