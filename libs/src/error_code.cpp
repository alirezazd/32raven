#include "error_code.hpp"

#ifndef NO_ERROR_MESSAGES

const char *GetMessage(ErrorCode::Common code) {
  switch (code) {
    case ErrorCode::Common::kOk:
      return "OK";
    case ErrorCode::Common::kUnknown:
      return "Unknown Error";
    case ErrorCode::Common::kUnknownCommand:
      return "Unknown Command Received in Command Handler";
    case ErrorCode::Common::kCommandInvalidPacket:
      return "Command Handler Invalid Packet";
    case ErrorCode::Common::kFcLinkInvalidGyroCalibrationIdConfig:
      return "FcLink Invalid Gyro Calibration ID Config";
    case ErrorCode::Common::kFcLinkInvalidRcCalibrationConfig:
      return "FcLink Invalid RC Calibration Config";
  }
  return "Unknown error";
}

const char *GetMessage(ErrorCode::Stm32 code) {
  switch (code) {
    case ErrorCode::Stm32::kHalErrorHandler:
      return "HAL Error_Handler (CubeMX MSP)";
    case ErrorCode::Stm32::kSystemReinit:
      return "STM32 System re-init";
    case ErrorCode::Stm32::kRccOscConfigFailed:
      return "STM32 RCC OscConfig failed";
    case ErrorCode::Stm32::kRccClockConfigFailed:
      return "STM32 RCC ClockConfig failed";
    case ErrorCode::Stm32::kSpiInitFailed:
      return "STM32 SPI init failed";
    case ErrorCode::Stm32::kTimInitFailed:
      return "STM32 TIM init failed";
    case ErrorCode::Stm32::kDshotInitFailed:
      return "STM32 DShot init failed";
    case ErrorCode::Stm32::kAdcInitFailed:
      return "STM32 ADC init failed";
    case ErrorCode::Stm32::kGpioReinit:
      return "GPIO Driver Re-init";
    case ErrorCode::Stm32::kGpioInvalidPort:
      return "GPIO Invalid Port";
    case ErrorCode::Stm32::kEscTelemetryInitFailed:
      return "STM32 ESC telemetry init failed";
    case ErrorCode::Stm32::kEscServiceInitFailed:
      return "STM32 ESC service init failed";
    case ErrorCode::Stm32::kCrsfLinkInitFailed:
      return "STM32 CRSF link init failed";
    case ErrorCode::Stm32::kDshotCodecInvalidArg:
      return "STM32 DShot codec invalid argument";
    case ErrorCode::Stm32::kGpsNotResponding:
      return "GPS Not Responding";
    case ErrorCode::Stm32::kGpsConfigTimepulseFailed:
      return "GPS Timepulse Config Failed";
    case ErrorCode::Stm32::kGpsConfigTimepulseBufferError:
      return "GPS TP1 Buffer Mismatch";
    case ErrorCode::Stm32::kGpsVerifyProtocolFailed:
      return "GPS Verify Protocol Failed";
    case ErrorCode::Stm32::kGpsVerifyNavPvtFailed:
      return "GPS Verify NavPvt Failed";
    case ErrorCode::Stm32::kGpsVerifyNavDopFailed:
      return "GPS Verify NavDop Failed";
    case ErrorCode::Stm32::kGpsVerifyNavCovFailed:
      return "GPS Verify NavCov Failed";
    case ErrorCode::Stm32::kGpsVerifyNavEoeFailed:
      return "GPS Verify NavEoe Failed";
    case ErrorCode::Stm32::kGpsVerifyRateFailed:
      return "GPS Verify Rate Failed";
    case ErrorCode::Stm32::kGpsVerifyDynModelFailed:
      return "GPS Verify DynModel Failed";
    case ErrorCode::Stm32::kGpsVerifyConstellationFailed:
      return "GPS Verify Constellation Failed";
    case ErrorCode::Stm32::kGpsVerifyItfmFailed:
      return "GPS Verify ITFM Failed";
    case ErrorCode::Stm32::kImuWhoAmIFail:
      return "IMU WhoAmI Failed";
    case ErrorCode::Stm32::kImuOverrun:
      return "IMU Path Overrun";
    case ErrorCode::Stm32::kImuDroppedFrame:
      return "IMU dropped frame";
    case ErrorCode::Stm32::kImuInvalidOdr:
      return "IMU invalid ODR";
    case ErrorCode::Stm32::kImuOdrMismatch:
      return "IMU ODR mismatch";
    case ErrorCode::Stm32::kImuNotInitialized:
      return "IMU Not Initialized";
    case ErrorCode::Stm32::kImuReinit:
      return "IMU re-init";
    case ErrorCode::Stm32::kImuInvalidSampleDetected:
      return "IMU invalid sample detected";
    case ErrorCode::Stm32::kImuCalibrationInvalidConfig:
      return "IMU calibration invalid config";
    case ErrorCode::Stm32::kImuCalibrationMotionDetected:
      return "Motion detected during IMU calibration";
    case ErrorCode::Stm32::kInvalidFifoWatermarkRecords:
      return "Invalid FIFO watermark records";
    case ErrorCode::Stm32::kEepromNotInitialized:
      return "EEPROM emulation not initialized";
    case ErrorCode::Stm32::kEepromReinit:
      return "EEPROM emulation re-init";
    case ErrorCode::Stm32::kEepromDeviceNotFound:
      return "EEPROM device not found";
    case ErrorCode::Stm32::kEepromInvalidConfig:
      return "EEPROM emulation invalid config";
    case ErrorCode::Stm32::kEepromFormatFailed:
      return "EEPROM emulation format failed";
    case ErrorCode::Stm32::kEepromWriteFailed:
      return "EEPROM emulation write failed";
    case ErrorCode::Stm32::kEepromSchemaMismatch:
      return "EEPROM schema mismatch";
    case ErrorCode::Stm32::kRcReceiverInvalidConfig:
      return "RC receiver invalid config";
  }
  return "Unknown error";
}

const char *GetMessage(ErrorCode::Esp32 code) {
  switch (code) {
    case ErrorCode::Esp32::kTcpServerStartFailed:
      return "TCP Server Start Failed";
    case ErrorCode::Esp32::kTcpServerAcceptFailed:
      return "TCP Server Accept Failed";
    case ErrorCode::Esp32::kTcpServerError:
      return "TCP Server Error";
    case ErrorCode::Esp32::kUdpServerInvalidOverflowThreshold:
      return "UDP Server Invalid Overflow Threshold";
    case ErrorCode::Esp32::kUdpServerUploadOverflow:
      return "UDP Server Upload Overflow";
    case ErrorCode::Esp32::kUdpServerDownloadOverflow:
      return "UDP Server Download Overflow";
    case ErrorCode::Esp32::kUsbCdcStartFailed:
      return "USB CDC Start Failed";
    case ErrorCode::Esp32::kWifiNvsInitFailed:
      return "WiFi NVS Init Failed";
    case ErrorCode::Esp32::kWifiNetifInitFailed:
      return "WiFi Netif Init Failed";
    case ErrorCode::Esp32::kWifiEventLoopFailed:
      return "WiFi Event Loop Failed";
    case ErrorCode::Esp32::kWifiInitFailed:
      return "WiFi Init Failed";
    case ErrorCode::Esp32::kWifiSetStorageFailed:
      return "WiFi Set Storage Failed";
    case ErrorCode::Esp32::kMavlinkInitFailed:
      return "Mavlink Init Failed";
    case ErrorCode::Esp32::kMavlinkPanicSendFailed:
      return "Mavlink Panic Status Send Failed";
    case ErrorCode::Esp32::kFcLinkInitFailed:
      return "FcLink Init Failed";
    case ErrorCode::Esp32::kFcLinkRxQueueFull:
      return "FcLink RX Queue Full";
    case ErrorCode::Esp32::kFcLinkInvalidPacketMagic1:
      return "FcLink Invalid Packet Magic Byte 1";
    case ErrorCode::Esp32::kFcLinkInvalidPacketMagic2:
      return "FcLink Invalid Packet Magic Byte 2";
    case ErrorCode::Esp32::kFcLinkInvalidPacketLength:
      return "FcLink Invalid Packet Length";
    case ErrorCode::Esp32::kFcLinkInvalidPacketCrc:
      return "FcLink Invalid Packet CRC";
    case ErrorCode::Esp32::kFcLinkTxSerializeFailed:
      return "FcLink TX Serialize Failed";
    case ErrorCode::Esp32::kFcLinkHandshakeFailed:
      return "FcLink Handshake Failed";
    case ErrorCode::Esp32::kFcLinkInvalidRcMapConfig:
      return "FcLink Invalid RC Map Config";
    case ErrorCode::Esp32::kFcLinkRcMapSetFailed:
      return "FcLink RC Map Set Failed";
    case ErrorCode::Esp32::kFcLinkRcCalibrationSetFailed:
      return "FcLink RC Calibration Set Failed";
    case ErrorCode::Esp32::kLedTaskCreateFailed:
      return "LED Task Create Failed";
    case ErrorCode::Esp32::kLedTimerInitFailed:
      return "LED Timer Init Failed";
    case ErrorCode::Esp32::kLedChannelInitFailed:
      return "LED Channel Init Failed";
    case ErrorCode::Esp32::kLedFadeInstallFailed:
      return "LED Fade Install Failed";
    case ErrorCode::Esp32::kBuzzerInvalidConfig:
      return "Buzzer Invalid Config";
    case ErrorCode::Esp32::kBuzzerTimerInitFailed:
      return "Buzzer Timer Init Failed";
    case ErrorCode::Esp32::kBuzzerChannelInitFailed:
      return "Buzzer Channel Init Failed";
    case ErrorCode::Esp32::kBuzzerSetFreqFailed:
      return "Buzzer Set Frequency Failed";
    case ErrorCode::Esp32::kBuzzerSetDutyFailed:
      return "Buzzer Set Duty Failed";
    case ErrorCode::Esp32::kBuzzerInvalidArg:
      return "Buzzer Invalid Argument";
    case ErrorCode::Esp32::kButtonGpioConfigFailed:
      return "Button GPIO Config Failed";
    case ErrorCode::Esp32::kI2cParamConfigFailed:
      return "I2C Param Config Failed";
    case ErrorCode::Esp32::kI2cInitFailed:
      return "I2C Init Failed";
    case ErrorCode::Esp32::kI2cInvalidArg:
      return "I2C Invalid Argument";
    case ErrorCode::Esp32::kI2cOperationFailed:
      return "I2C Operation Failed";
    case ErrorCode::Esp32::kUartParamConfigFailed:
      return "UART Param Config Failed";
    case ErrorCode::Esp32::kUartSetPinFailed:
      return "UART Set Pin Failed";
    case ErrorCode::Esp32::kUartDriverInstallFailed:
      return "UART Driver Install Failed";
    case ErrorCode::Esp32::kUartInvalidNumber:
      return "UART Invalid Number";
    case ErrorCode::Esp32::kUartNotInitialized:
      return "UART Not Initialized";
    case ErrorCode::Esp32::kUartInvalidArg:
      return "UART Invalid Argument";
    case ErrorCode::Esp32::kUartOperationFailed:
      return "UART Operation Failed";
    case ErrorCode::Esp32::kUartFlushFailed:
      return "UART Flush Failed";
    case ErrorCode::Esp32::kProgrammerUartNull:
      return "Programmer UART Null";
    case ErrorCode::Esp32::kProgrammerHandshakeFailed:
      return "Programmer Handshake Failed";
    case ErrorCode::Esp32::kProgrammerBufferOverflow:
      return "Programmer Buffer Overflow";
    case ErrorCode::Esp32::kProgrammerEraseFailed:
      return "Programmer Erase Failed";
    case ErrorCode::Esp32::kProgrammerWriteFailed:
      return "Programmer Write Failed";
    case ErrorCode::Esp32::kProgrammerReadFailed:
      return "Programmer Read Failed";
    case ErrorCode::Esp32::kProgrammerVerifyFailed:
      return "Programmer Verify Failed";
    case ErrorCode::Esp32::kProgrammerOtaPartitionNotFound:
      return "Programmer OTA Partition Not Found";
    case ErrorCode::Esp32::kProgrammerOtaBeginFailed:
      return "Programmer OTA Begin Failed";
    case ErrorCode::Esp32::kProgrammerOtaWriteFailed:
      return "Programmer OTA Write Failed";
    case ErrorCode::Esp32::kProgrammerOtaEndFailed:
      return "Programmer OTA End Failed";
    case ErrorCode::Esp32::kProgrammerOtaSetBootFailed:
      return "Programmer OTA Set Boot Failed";
    case ErrorCode::Esp32::kProgrammerTimedOut:
      return "Programmer Timed Out";
    case ErrorCode::Esp32::kDisplayPanelInitFailed:
      return "Display Panel Init Failed";
    case ErrorCode::Esp32::kUiInitFailed:
      return "Display Service Init Failed";
    case ErrorCode::Esp32::kTonePlayerInitFailed:
      return "TonePlayer Init Failed";
  }
  return "Unknown error";
}

const char *GetMessage(uint32_t raw_code) {
  if (raw_code >= 0x20000) {
    return GetMessage(static_cast<ErrorCode::Esp32>(raw_code));
  }
  if (raw_code >= 0x10000) {
    return GetMessage(static_cast<ErrorCode::Stm32>(raw_code));
  }
  return GetMessage(static_cast<ErrorCode::Common>(raw_code));
}

#else  // NO_ERROR_MESSAGES

const char *GetMessage(ErrorCode::Common) { return "Error"; }
const char *GetMessage(ErrorCode::Stm32) { return "Error"; }
const char *GetMessage(ErrorCode::Esp32) { return "Error"; }
const char *GetMessage(uint32_t) { return "Error"; }

#endif
