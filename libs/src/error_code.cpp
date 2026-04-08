#include "error_code.hpp"

const char *GetMessage(ErrorCode code) {
#ifndef NO_ERROR_MESSAGES
  switch (code) {
    case ErrorCode::kOk:
      return "OK";
    case ErrorCode::kUnknown:
      return "Unknown Error";
    case ErrorCode::kSystemReinit:
      return "System Re-init";
    case ErrorCode::kTcpServerStartFailed:
      return "TCP Server Start Failed";
    case ErrorCode::kTcpServerAcceptFailed:
      return "TCP Server Accept Failed";
    case ErrorCode::kLedTaskCreateFailed:
      return "LED Task Create Failed";
    case ErrorCode::kMavlinkInitFailed:
      return "Mavlink Init Failed";
    case ErrorCode::kFcLinkInitFailed:
      return "FcLink Init Failed";
    case ErrorCode::kFcLinkRxQueueFull:
      return "FcLink RX Queue Full";
    case ErrorCode::kFcLinkInvalidPacketMagic1:
      return "FcLink Invalid Packet Magic Byte 1";
    case ErrorCode::kFcLinkInvalidPacketMagic2:
      return "FcLink Invalid Packet Magic Byte 2";
    case ErrorCode::kFcLinkInvalidPacketLength:
      return "FcLink Invalid Packet Length";
    case ErrorCode::kFcLinkInvalidPacketCrc:
      return "FcLink Invalid Packet CRC";
    case ErrorCode::kFcLinkTxSerializeFailed:
      return "FcLink TX Serialize Failed";
    case ErrorCode::kCommandInitFailed:
      return "Command Init Failed";
    case ErrorCode::kLedGpioConfigFailed:
      return "LED GPIO Config Failed";
    case ErrorCode::kLedTimerInitFailed:
      return "LED Timer Init Failed";
    case ErrorCode::kLedChannelInitFailed:
      return "LED Channel Init Failed";
    case ErrorCode::kLedFadeInstallFailed:
      return "LED Fade Install Failed";
    case ErrorCode::kBuzzerInvalidConfig:
      return "Buzzer Invalid Config";
    case ErrorCode::kBuzzerTimerInitFailed:
      return "Buzzer Timer Init Failed";
    case ErrorCode::kBuzzerChannelInitFailed:
      return "Buzzer Channel Init Failed";
    case ErrorCode::kBuzzerSetFreqFailed:
      return "Buzzer Set Frequency Failed";
    case ErrorCode::kBuzzerSetDutyFailed:
      return "Buzzer Set Duty Failed";
    case ErrorCode::kButtonGpioConfigFailed:
      return "Button GPIO Config Failed";
    case ErrorCode::kWifiNvsInitFailed:
      return "WiFi NVS Init Failed";
    case ErrorCode::kWifiNetifInitFailed:
      return "WiFi Netif Init Failed";
    case ErrorCode::kWifiEventLoopFailed:
      return "WiFi Event Loop Failed";
    case ErrorCode::kWifiInitFailed:
      return "WiFi Init Failed";
    case ErrorCode::kWifiSetStorageFailed:
      return "WiFi Set Storage Failed";
    case ErrorCode::kI2cParamConfigFailed:
      return "I2C Param Config Failed";
    case ErrorCode::kI2cInitFailed:
      return "I2C Init Failed";
    case ErrorCode::kI2cInvalidArg:
      return "I2C Invalid Argument";
    case ErrorCode::kI2cOperationFailed:
      return "I2C Operation Failed";
    case ErrorCode::kUartParamConfigFailed:
      return "UART Param Config Failed";
    case ErrorCode::kUartSetPinFailed:
      return "UART Set Pin Failed";
    case ErrorCode::kUartDriverInstallFailed:
      return "UART Driver Install Failed";
    case ErrorCode::kUartInvalidNumber:
      return "UART Invalid Number";
    case ErrorCode::kUartNotInitialized:
      return "UART Not Initialized";
    case ErrorCode::kUartInvalidArg:
      return "UART Invalid Argument";
    case ErrorCode::kUartOperationFailed:
      return "UART Operation Failed";
    case ErrorCode::kUartFlushFailed:
      return "UART Flush Failed";
    case ErrorCode::kProgrammerUartNull:
      return "Programmer UART Null";
    case ErrorCode::kProgrammerHandshakeFailed:
      return "Programmer Handshake Failed";
    case ErrorCode::kProgrammerBufferOverflow:
      return "Programmer Buffer Overflow";
    case ErrorCode::kProgrammerEraseFailed:
      return "Programmer Erase Failed";
    case ErrorCode::kProgrammerWriteFailed:
      return "Programmer Write Failed";
    case ErrorCode::kProgrammerReadFailed:
      return "Programmer Read Failed";
    case ErrorCode::kProgrammerVerifyFailed:
      return "Programmer Verify Failed";
    case ErrorCode::kProgrammerOtaPartitionNotFound:
      return "Programmer OTA Partition Not Found";
    case ErrorCode::kProgrammerOtaBeginFailed:
      return "Programmer OTA Begin Failed";
    case ErrorCode::kProgrammerOtaWriteFailed:
      return "Programmer OTA Write Failed";
    case ErrorCode::kProgrammerOtaEndFailed:
      return "Programmer OTA End Failed";
    case ErrorCode::kProgrammerOtaSetBootFailed:
      return "Programmer OTA Set Boot Failed";
    case ErrorCode::kProgrammerTimedOut:
      return "Programmer Timed Out";
    case ErrorCode::kStm32GpioInitFailed:
      return "STM32 GPIO Init Failed";
    case ErrorCode::kDisplayPanelInitFailed:
      return "Display Panel Init Failed";
    case ErrorCode::kUiInitFailed:
      return "Display Service Init Failed";
    case ErrorCode::kTonePlayerInitFailed:
      return "TonePlayer Init Failed";
    case ErrorCode::kBuzzerInvalidArg:
      return "Buzzer Invalid Argument";
    case ErrorCode::kGpioReinit:
      return "GPIO Driver Re-init";
    case ErrorCode::kGpioInvalidPort:
      return "GPIO Invalid Port";
    case ErrorCode::kFcLinkHandshakeFailed:
      return "FcLink Handshake Failed";
    case ErrorCode::kTcpServerError:
      return "TCP Server Error";
    case ErrorCode::kGpsNotResponding:
      return "GPS Not Responding";
    case ErrorCode::kGpsConfigProtocolFailed:
      return "GPS Protocol Config Failed";
    case ErrorCode::kGpsConfigNavPvtFailed:
      return "GPS NavPvt Config Failed";
    case ErrorCode::kGpsConfigNavDopFailed:
      return "GPS NavDop Config Failed";
    case ErrorCode::kGpsConfigNavCovFailed:
      return "GPS NavCov Config Failed";
    case ErrorCode::kGpsConfigNavEoeFailed:
      return "GPS NavEoe Config Failed";
    case ErrorCode::kGpsConfigRateFailed:
      return "GPS Rate Config Failed";
    case ErrorCode::kGpsConfigDynModelFailed:
      return "GPS DynModel Config Failed";
    case ErrorCode::kGpsConfigConstellationFailed:
      return "GPS Constellation Config Failed";
    case ErrorCode::kGpsConfigItfmFailed:
      return "GPS ITFM Config Failed";
    case ErrorCode::kGpsConfigTimepulseFailed:
      return "GPS Timepulse Config Failed";
    case ErrorCode::kGpsConfigTimepulseBufferError:
      return "GPS TP1 Buffer Mismatch";
    case ErrorCode::kGpsVerifyProtocolFailed:
      return "GPS Verify Protocol Failed";
    case ErrorCode::kGpsVerifyNavPvtFailed:
      return "GPS Verify NavPvt Failed";
    case ErrorCode::kGpsVerifyNavDopFailed:
      return "GPS Verify NavDop Failed";
    case ErrorCode::kGpsVerifyNavCovFailed:
      return "GPS Verify NavCov Failed";
    case ErrorCode::kGpsVerifyNavEoeFailed:
      return "GPS Verify NavEoe Failed";
    case ErrorCode::kGpsVerifyRateFailed:
      return "GPS Verify Rate Failed";
    case ErrorCode::kGpsVerifyDynModelFailed:
      return "GPS Verify DynModel Failed";
    case ErrorCode::kGpsVerifyConstellationFailed:
      return "GPS Verify Constellation Failed";
    case ErrorCode::kGpsVerifyItfmFailed:
      return "GPS Verify ITFM Failed";
    case ErrorCode::kGpsVerifyTimepulseFailed:
      return "GPS Verify Timepulse Failed";
    case ErrorCode::kUdpServerInvalidOverflowThreshold:
      return "UDP Server Invalid Overflow Threshold";
    case ErrorCode::kUdpServerUploadOverflow:
      return "UDP Server Upload Overflow";
    case ErrorCode::kUdpServerDownloadOverflow:
      return "UDP Server Download Overflow";
    case ErrorCode::kImuWhoAmIFail:
      return "IMU WhoAmI Failed";
    case ErrorCode::kImuFifoUnsupported:
      return "IMU FIFO Unsupported";
    case ErrorCode::kImuFifoReadTooLarge:
      return "IMU FIFO Read Too Large";
    case ErrorCode::kImuOverrun:
      return "IMU Path Overrun";
    case ErrorCode::kImuDroppedFrame:
      return "IMU dropped frame";
    case ErrorCode::kImuInvalidOdr:
      return "IMU invalid ODR";
    case ErrorCode::kImuOdrMismatch:
      return "IMU ODR mismatch";
    case ErrorCode::kImuNotInitialized:
      return "IMU Not Initialized";
    case ErrorCode::kImuReinit:
      return "IMU re-init";
    case ErrorCode::kImuDmaBufferTooSmall:
      return "IMU DMA buffer too small";
    case ErrorCode::kImuInvalidSampleDetected:
      return "IMU invalid sample detected";
    case ErrorCode::kImuCalibrationInvalidConfig:
      return "IMU calibration invalid config";
    case ErrorCode::kImuCalibrationMotionDetected:
      return "Motion detected during IMU calibration";
    case ErrorCode::kInvalidFifoWatermarkRecords:
      return "Invalid FIFO watermark records";
    case ErrorCode::kEepromNotInitialized:
      return "EEPROM emulation not initialized";
    case ErrorCode::kEepromReinit:
      return "EEPROM emulation re-init";
    case ErrorCode::kEepromInvalidConfig:
      return "EEPROM emulation invalid config";
    case ErrorCode::kEepromFormatFailed:
      return "EEPROM emulation format failed";
    case ErrorCode::kEepromNoValidSlot:
      return "EEPROM emulation no valid slot";
    case ErrorCode::kEepromWriteFailed:
      return "EEPROM emulation write failed";
    case ErrorCode::kEepromSchemaMismatch:
      return "EEPROM schema mismatch";
    default:
      return "Unknown error";
  }
#else
  return "Error";
#endif
}
