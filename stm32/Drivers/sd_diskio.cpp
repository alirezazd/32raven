// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

// FatFs's disk access layer, mapped onto the Sdio driver. Bench-tier only:
// callers are FatFs metadata work or WiFi file reads -- the flight path
// writes raw sectors through Sdio directly and never arrives here.

#include <cstring>
#include <span>

// diskio.h reads ff.h's integer types and does not include it itself.
// clang-format off
#include "ff.h"
#include "diskio.h"
// clang-format on
#include "sdio.hpp"

namespace {

// FatFs hands out whatever buffer its caller held; DMA moves words. Odd
// addresses take the one-sector detour through here instead of failing.
alignas(4) uint8_t g_bounce[Sdio::kBlockBytes];

bool Aligned4(const void *p) {
  return (reinterpret_cast<uintptr_t>(p) & 0x3u) == 0u;
}

}  // namespace

extern "C" {

DSTATUS disk_status(BYTE pdrv) {
  if (pdrv != 0 || !Sdio::GetInstance().CardPresent()) {
    return STA_NOINIT;
  }
  return 0;
}

DSTATUS disk_initialize(BYTE pdrv) {
  // Sdio::Init probes at boot and MSC exit re-probes; repeating the 400 kHz
  // identification on every f_mount would buy nothing.
  return disk_status(pdrv);
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
  if (pdrv != 0) {
    return RES_PARERR;
  }
  Sdio &sd = Sdio::GetInstance();
  if (Aligned4(buff)) {
    const std::span dst{buff, count * Sdio::kBlockBytes};
    return sd.ReadBlocks(sector, dst) == Outcome::kOk ? RES_OK : RES_ERROR;
  }
  for (UINT i = 0; i < count; ++i) {
    if (sd.ReadBlocks(sector + i, g_bounce) != Outcome::kOk) {
      return RES_ERROR;
    }
    std::memcpy(buff + (i * Sdio::kBlockBytes), g_bounce, Sdio::kBlockBytes);
  }
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
  if (pdrv != 0) {
    return RES_PARERR;
  }
  Sdio &sd = Sdio::GetInstance();
  if (Aligned4(buff)) {
    const std::span src{buff, count * Sdio::kBlockBytes};
    return sd.WriteBlocks(sector, src) == Outcome::kOk ? RES_OK : RES_ERROR;
  }
  for (UINT i = 0; i < count; ++i) {
    std::memcpy(g_bounce, buff + (i * Sdio::kBlockBytes), Sdio::kBlockBytes);
    if (sd.WriteBlocks(sector + i, g_bounce) != Outcome::kOk) {
      return RES_ERROR;
    }
  }
  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
  if (pdrv != 0) {
    return RES_PARERR;
  }
  switch (cmd) {
    case CTRL_SYNC:
      // WriteBlocks returns only after the card leaves its busy phase.
      return RES_OK;
    case GET_SECTOR_COUNT:
      *static_cast<LBA_t *>(buff) = Sdio::GetInstance().BlockCount();
      return RES_OK;
    case GET_BLOCK_SIZE:
      *static_cast<DWORD *>(buff) = 1;
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

}  // extern "C"
