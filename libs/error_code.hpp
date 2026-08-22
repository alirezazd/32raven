// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include <cstdint>

// ErrorCode is split per-MCU so each enum class only carries values relevant
// to one side. Numeric ranges are disjoint (Common: 0x0xxxx, Stm32: 0x1xxxx,
// Esp32: 0x2xxxx) so a uint32_t-on-the-wire panic from one side can be
// dispatched and decoded on the other via GetMessage(uint32_t).
// Dead values are pruned aggressively. `scripts/lint/check_error_codes.py` runs at
// build time and fails the build if any enumerator here has zero callsites
// (excluding `error_code.cpp` itself, which is required to switch over them).
struct ErrorCode {
// The enums initialize their first entry from these, so the domain boundaries
// GetMessage(uint32_t) dispatches on cannot drift from the values themselves.
static constexpr uint32_t kStm32Base = 0x10000;
static constexpr uint32_t kEsp32Base = 0x20000;

enum class Common : uint32_t {
  kOk = 0x00000,
  kUnknown,
  kUnknownCommand,
  kCommandInvalidPacket,
  kFcLinkInvalidGyroCalibrationIdConfig,
  kFcLinkInvalidRcCalibrationConfig,
};

enum class Stm32 : uint32_t {
  // Init
  kSystemReinit = kStm32Base,
  kRccOscConfigFailed,
  kRccClockConfigFailed,
  kSpiInitFailed,
  kTimInitFailed,
  kDshotInitFailed,
  kDshotClockMismatch,
  kDshotPeriodUnrepresentable,
  kAdcInitFailed,
  kGpioReinit,
  kGpioInvalidPort,
  kEscTelemetryInitFailed,
  kEscServiceInitFailed,
  kEscInputTypeNotDshot,
  kEscDirectionReversed,
  kEsc3dModeEnabled,
  kCrsfLinkInitFailed,
  kDshotCodecInvalidArg,
  // GPS
  kGpsNotResponding,
  kGpsConfigTimepulseFailed,
  kGpsVerifyProtocolFailed,
  kGpsVerifyNavPvtFailed,
  kGpsVerifyNavDopFailed,
  kGpsVerifyNavCovFailed,
  kGpsVerifyNavEoeFailed,
  kGpsVerifyRateFailed,
  kGpsVerifyDynModelFailed,
  kGpsVerifyConstellationFailed,
  kGpsVerifyItfmFailed,
  kGpsValsetFrameOverflow,
  // IMU
  kImuWhoAmIFail,
  kImuOverrun,
  kImuDroppedFrame,
  kImuNotInitialized,
  kImuReinit,
  kImuInvalidSampleDetected,
  // EEPROM
  kEepromNotInitialized,
  kEepromReinit,
  kEepromDeviceNotFound,
  kEepromInvalidConfig,
  kEepromFormatFailed,
  kEepromWriteFailed,
  kEepromSchemaMismatch,
  // Misc
  kRcReceiverInvalidConfig,
  kRcReceiverInvalidThrottleMin,
  kMixerInvalidConfig,
  kAhrsInvalidConfig,
  kAttitudeControllerInvalidConfig,
  kHseClockFailure,  // HSE lost (CSS): fell back to HSI, motors disarmed
  // Appended only. These values travel in PanicMsg.error_code and are decoded
  // on the ESP32, which flashes independently of the STM32 — renumbering an
  // existing entry makes a mismatched pair report the wrong fault.
  kUsbInitFailed,
  kUsbReinit,
  kMspServiceReinit,
  kFourWayServiceReinit,
  kUartSoftReinit,
  kEscBootloaderReinit,
  kSentinelReinit,
  kSentinelInvalidConfig,
  kSdioReinit,
  kLogServiceReinit,
  kMscServiceReinit,
  kSdCardMissing,
  kSdCardCorrupted,
  kSdCardFull,
  kHardFault,
  kCommandHandlerReinit,
  kFcLinkReinit,
  kSensorCalServiceReinit,
};

enum class Esp32 : uint32_t {
  // Networking
  kTcpServerStartFailed = kEsp32Base,
  kTcpServerAcceptFailed,
  kTcpServerError,
  kUdpServerInvalidOverflowThreshold,
  kUdpServerUploadOverflow,
  kUdpServerDownloadOverflow,
  kUsbCdcStartFailed,
  // WiFi
  kWifiNvsInitFailed,
  kWifiNetifInitFailed,
  kWifiEventLoopFailed,
  kWifiInitFailed,
  kWifiSetStorageFailed,
  // Mavlink
  kMavlinkInitFailed,
  kMavlinkPanicSendFailed,
  // FcLink (ESP32 side)
  kFcLinkInitFailed,
  kFcLinkRxQueueFull,
  kFcLinkInvalidPacketMagic1,
  kFcLinkInvalidPacketMagic2,
  kFcLinkInvalidPacketLength,
  kFcLinkInvalidPacketCrc,
  kFcLinkTxSerializeFailed,
  kFcLinkHandshakeFailed,
  kFcLinkInvalidRcMapConfig,
  kFcLinkRcMapSetFailed,
  kFcLinkRcCalibrationSetFailed,
  // LED / buzzer / button
  kLedTaskCreateFailed,
  kLedTimerInitFailed,
  kLedChannelInitFailed,
  kLedFadeInstallFailed,
  kBuzzerInvalidConfig,
  kBuzzerTimerInitFailed,
  kBuzzerChannelInitFailed,
  kBuzzerSetFreqFailed,
  kBuzzerSetDutyFailed,
  kBuzzerInvalidArg,
  kButtonGpioConfigFailed,
  // ESP-IDF I2C/UART wrappers
  kI2cParamConfigFailed,
  kI2cInitFailed,
  kI2cInvalidArg,
  kI2cOperationFailed,
  kUartParamConfigFailed,
  kUartSetPinFailed,
  kUartDriverInstallFailed,
  kUartInvalidNumber,
  kUartNotInitialized,
  kUartInvalidArg,
  kUartOperationFailed,
  kUartFlushFailed,
  // Programmer / OTA
  kProgrammerUartNull,
  kProgrammerHandshakeFailed,
  kProgrammerBufferOverflow,
  kProgrammerEraseFailed,
  kProgrammerWriteFailed,
  kProgrammerReadFailed,
  kProgrammerVerifyFailed,
  kProgrammerOtaPartitionNotFound,
  kProgrammerOtaBeginFailed,
  kProgrammerOtaWriteFailed,
  kProgrammerOtaEndFailed,
  kProgrammerOtaSetBootFailed,
  kProgrammerTimedOut,
  // UI
  kDisplayPanelInitFailed,
  kUiInitFailed,
  kTonePlayerInitFailed,
  kSystemReinit,
};
};

// Per-domain message lookups. Each takes its own enum class so misuse is a
// compile error rather than a runtime mystery.
const char *GetMessage(ErrorCode::Common code);
const char *GetMessage(ErrorCode::Stm32 code);
const char *GetMessage(ErrorCode::Esp32 code);

// Wire-side dispatch: panic codes are serialized as uint32_t over fc_link, so
// the receiving MCU resolves them by numeric range to the right domain.
const char *GetMessage(uint32_t raw_code);
