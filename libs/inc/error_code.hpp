#pragma once
#include <cstdint>

// ErrorCode is split per-MCU so each enum class only carries values relevant
// to one side. Numeric ranges are disjoint (Common: 0x0xxxx, Stm32: 0x1xxxx,
// Esp32: 0x2xxxx) so a uint32_t-on-the-wire panic from one side can be
// dispatched and decoded on the other via GetMessage(uint32_t).
struct ErrorCode {
enum class Common : uint32_t {
  kOk = 0x00000,
  kUnknown,
  kUnknownCommand,
  kSystemReinit,
  kCommandInitFailed,
  kCommandInvalidPacket,
  kCommandInvalidRcMapConfig,
  kCommandInvalidRcCalibrationConfig,
  kCommandInvalidGyroCalibrationIdConfig,
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
  kI2cInitFailed,
  kTimInitFailed,
  kDshotInitFailed,
  kAdcInitFailed,
  kDmaInitFailed,
  kGpioInitFailed,
  kGpioConfigFailed,
  kGpioReinit,
  kGpioInvalidPort,
  kEscTelemetryInitFailed,
  kEscServiceInitFailed,
  kCrsfLinkInitFailed,
  kDshotCodecInvalidArg,
  // FcLink (STM32 side)
  kFcLinkInitFailed,
  kFcLinkRxQueueFull,
  kFcLinkInvalidPacketMagic1,
  kFcLinkInvalidPacketMagic2,
  kFcLinkInvalidPacketLength,
  kFcLinkInvalidPacketCrc,
  kFcLinkTxSerializeFailed,
  kFcLinkHandshakeFailed,
  kFcLinkRcMapRequestFailed,
  kFcLinkInvalidRcMapConfig,
  kFcLinkRcCalibrationRequestFailed,
  kFcLinkGyroCalibrationIdRequestFailed,
  kFcLinkRcMapSetFailed,
  kFcLinkRcCalibrationSetFailed,
  // GPS
  kGpsNotResponding,
  kGpsConfigProtocolFailed,
  kGpsConfigNavPvtFailed,
  kGpsConfigNavDopFailed,
  kGpsConfigNavCovFailed,
  kGpsConfigNavEoeFailed,
  kGpsConfigRateFailed,
  kGpsConfigDynModelFailed,
  kGpsConfigConstellationFailed,
  kGpsConfigItfmFailed,
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
  kGpsVerifyTimepulseFailed,
  // IMU
  kImuWhoAmIFail,
  kImuFifoUnsupported,
  kImuFifoReadTooLarge,
  kImuOverrun,
  kImuDroppedFrame,
  kImuInvalidOdr,
  kImuOdrMismatch,
  kImuNotInitialized,
  kImuReinit,
  kImuDmaBufferTooSmall,
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
  kEepromNoValidSlot,
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
  kUdpServerInitFailed,
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
  kMavlinkTaskAlreadyRunning,
  kMavlinkTaskStopOutsidePanic,
  kMavlinkStackOverflow,
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
  kFcLinkRcMapRequestFailed,
  kFcLinkInvalidRcMapConfig,
  kFcLinkRcCalibrationRequestFailed,
  kFcLinkGyroCalibrationIdRequestFailed,
  kFcLinkRcMapSetFailed,
  kFcLinkRcCalibrationSetFailed,
  // LED / buzzer / button
  kLedTaskCreateFailed,
  kLedGpioConfigFailed,
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
