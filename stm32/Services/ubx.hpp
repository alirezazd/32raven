// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

// The UBX NAV payloads M10Service decodes, one packed struct per message, each
// pinned to its UBX payload length. Kept free of every other include so the SIL
// can serialize the same bytes from the same definitions.

struct M10PVTData {
  uint32_t iTOW;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t tAcc;
  int32_t nano;
  uint8_t fixType;
  uint8_t flags;
  uint8_t flags2;
  uint8_t numSV;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  uint32_t hAcc;
  uint32_t vAcc;
  int32_t velN;
  int32_t velE;
  int32_t velD;
  int32_t gSpeed;
  int32_t headMot;
  uint32_t sAcc;
  uint32_t headAcc;
  uint16_t pDOP;
  uint8_t reserved1[6];
  int32_t headVeh;
  int16_t magDec;
  uint16_t magAcc;
} __attribute__((packed));

static_assert(sizeof(M10PVTData) == 92,
              "M10PVTData size must match UBX NAV-PVT payload");

struct M10DOPData {
  uint32_t iTOW;
  uint16_t gDOP;
  uint16_t pDOP;
  uint16_t tDOP;
  uint16_t vDOP;
  uint16_t hDOP;
  uint16_t nDOP;
  uint16_t eDOP;
} __attribute__((packed));

static_assert(sizeof(M10DOPData) == 18,
              "M10DOPData size must match UBX NAV-DOP payload");

struct M10COVData {
  uint32_t iTOW;
  uint8_t version;
  uint8_t posCovValid;
  uint8_t velCovValid;
  uint8_t reserved1;
  float posCovNN;
  float posCovNE;
  float posCovND;
  float posCovEE;
  float posCovED;
  float posCovDD;
  float velCovNN;
  float velCovNE;
  float velCovND;
  float velCovEE;
  float velCovED;
  float velCovDD;
} __attribute__((packed));

static_assert(sizeof(M10COVData) == 56,
              "M10COVData size must match UBX NAV-COV payload");
