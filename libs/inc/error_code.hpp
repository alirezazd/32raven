#pragma once
#include <cstdint>

// ErrorCode is split per-MCU so each enum class only carries values relevant
// to one side. Numeric ranges are disjoint (Common: 0x0xxxx, Stm32: 0x1xxxx,
// Esp32: 0x2xxxx) so a uint32_t-on-the-wire panic from one side can be
// dispatched and decoded on the other via GetMessage(uint32_t).
//
// Dead values are pruned aggressively. `scripts/lint/check_error_codes.py` runs at
// build time and fails the build if any enumerator here has zero callsites
// (excluding `error_code.cpp` itself, which is required to switch over them).
struct ErrorCode {
enum class Common : uint32_t {
  kOk = 0x00000,
  kUnknown,
  kUnknownCommand,
  kCommandInvalidPacket,
  kFcLinkInvalidGyroCalibrationIdConfig,
  kFcLinkInvalidRcCalibrationConfig,
};

enum class Stm32 : uint32_t {
  // Init / HAL
  kHalErrorHandler = 0x10000,
  kSystemReinit,
  kRccOscConfigFailed,
  kRccClockConfigFailed,
  kSpiInitFailed,
  kUartInitFailed,
  kTimInitFailed,
  kDshotInitFailed,
  kAdcInitFailed,
  kGpioConfigFailed,
  kGpioReinit,
  kGpioInvalidPort,
  kEscTelemetryInitFailed,
  kEscServiceInitFailed,
  kCrsfLinkInitFailed,
  kDshotCodecInvalidArg,
  // GPS
  kGpsNotResponding,
  kGpsConfigTimepulseFailed,
  kGpsConfigTimepulseBufferError,
  kGpsVerifyProtocolFailed,
  kGpsVerifyNavPvtFailed,
  kGpsVerifyNavDopFailed,
  kGpsVerifyNavCovFailed,
  kGpsVerifyNavEoeFailed,
  kGpsVerifyRateFailed,
  kGpsVerifyDynModelFailed,
  kGpsVerifyConstellationFailed,
  kGpsVerifyItfmFailed,
  // IMU
  kImuWhoAmIFail,
  kImuOverrun,
  kImuDroppedFrame,
  kImuInvalidOdr,
  kImuOdrMismatch,
  kImuNotInitialized,
  kImuReinit,
  kImuInvalidSampleDetected,
  kImuCalibrationInvalidConfig,
  kImuCalibrationMotionDetected,
  kInvalidFifoWatermarkRecords,
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
};

enum class Esp32 : uint32_t {
  // Networking
  kTcpServerStartFailed = 0x20000,
  kTcpServerAcceptFailed,
  kTcpServerError,
  kUdpServerInvalidOverflowThreshold,
  kUdpServerUploadOverflow,
  kUdpServerDownloadOverflow,
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
