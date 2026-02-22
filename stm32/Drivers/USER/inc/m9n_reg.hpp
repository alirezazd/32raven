#pragma once

#include <cstddef>
#include <cstdint>

// UBX-CFG-VALSET fixed header fields
constexpr uint8_t kValsetVersion = 0x00;

namespace UBX { // NOLINT
static constexpr uint8_t kSync1 = 0xB5;
static constexpr uint8_t kSync2 = 0x62;
static constexpr uint8_t kClsNav = 0x01;
static constexpr uint8_t kIdNavPvt = 0x07;
static constexpr uint8_t kIdNavDop = 0x04;
static constexpr uint8_t kIdNavCov = 0x36;
static constexpr uint8_t kIdNavEoe = 0x61;
static constexpr uint8_t kClsCfg = 0x06;
static constexpr uint8_t kIdCfgValset = 0x8A;
static constexpr uint8_t kIdCfgValget = 0x8B;
static constexpr uint8_t kClsAck = 0x05;
static constexpr uint8_t kIdAckAck = 0x01;
static constexpr uint8_t kIdAckNak = 0x00;
} // namespace UBX

// Layers bitfield in VALSET:
// 0x01 = RAM only (volatile). Keep minimal and safe.
// 0x07 = RAM | BBR | Flash (persistent across reboots and battery removal)
constexpr uint8_t kValsetLayerRam = 0x01;
constexpr uint8_t kValsetLayerAll = 0x07;

// Layers enum in VALGET (different from VALSET!):
// 0 = RAM, 1 = BBR, 2 = Flash, 7 = Default
constexpr uint8_t kValgetLayerRam = 0;
constexpr uint8_t kValgetLayerBbr = 1;
constexpr uint8_t kValgetLayerFlash = 2;
constexpr uint8_t kValgetLayerDefault = 7;

// Key IDs you are using
constexpr uint32_t kKeyCfgRateMeasMs = 0x30210001; // CFG-RATE-MEAS
constexpr uint32_t kKeyCfgDynModel = 0x20110021;   // CFG-NAVSPG-DYNMODEL
constexpr uint32_t kKeyGpsEnable = 0x1031001f;     // CFG-SIGNAL-GPS_ENA
constexpr uint32_t kKeyGloEnable = 0x10310025;     // CFG-SIGNAL-GLO_ENA
constexpr uint32_t kKeyGalEnable = 0x10310021;     // CFG-SIGNAL-GAL_ENA
constexpr uint32_t kKeyBdsEnable = 0x10310022;     // CFG-SIGNAL-BDS_ENA
constexpr uint32_t kKeySbasEnable = 0x10310020;    // CFG-SIGNAL-SBAS_ENA

constexpr uint32_t kKeyItfmEnable = 0x10410013; // CFG-ITFM-ENABLE

constexpr uint32_t kKeyMsgoutNavPvtUart1 =
    0x20910007; // CFG-MSGOUT-UBX_NAV_PVT_UART1
constexpr uint32_t kKeyMsgoutNavDopUart1 =
    0x20910039; // CFG-MSGOUT-UBX_NAV_DOP_UART1
constexpr uint32_t kKeyMsgoutNavCovUart1 =
    0x20910083; // CFG-MSGOUT-UBX_NAV_COV_UART1
constexpr uint32_t kKeyMsgoutNavEoeUart1 =
    0x20910160;                                // CFG-MSGOUT-UBX_NAV_EOE_UART1
const uint32_t kKeyUart1Baudrate = 0x40520001; // CFG-UART1-BAUDRATE
constexpr uint32_t kKeyUart1OutprotUbx = 0x10740001;  // CFG-UART1OUTPROT-UBX
constexpr uint32_t kKeyUart1OutprotNmea = 0x10740002; // CFG-UART1OUTPROT-NMEA

// Timepulse (TP1) Configuration
constexpr uint32_t kKeyCfgTp1Ena = 0x10050007;    // CFG-TP-TP1_ENA
constexpr uint32_t kKeyCfgTp1Period = 0x40050002; // CFG-TP-PERIOD_TP1 (us)
constexpr uint32_t kKeyCfgTp1Len = 0x40050004;    // CFG-TP-LEN_TP1 (us)
constexpr uint32_t kKeyCfgTp1TimeGrid =
    0x2005000c; // CFG-TP-TIMEGRID_TP1 (1=GPS)
constexpr uint32_t kKeyCfgTp1SyncGnss = 0x10050008;   // CFG-TP-SYNC_GNSS_TP1
constexpr uint32_t kKeyCfgTp1UseLocked = 0x10050009;  // CFG-TP-USE_LOCKED_TP1
constexpr uint32_t kKeyCfgTp1AlignToTow = 0x1005000a; // CFG-TP-ALIGN_TO_TOW_TP1
constexpr uint32_t kKeyCfgTp1Pol = 0x1005000b;        // CFG-TP-POL_TP1

constexpr uint32_t kKeyCfgTp1PeriodLock = 0x40050003; // CFG-TP-PERIOD_LOCK_TP1
constexpr uint32_t kKeyCfgTp1LenLock = 0x40050005;    // CFG-TP-LEN_LOCK_TP1
