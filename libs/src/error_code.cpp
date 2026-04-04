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
    case ErrorCode::kProgrammerUartNull:
      return "Programmer UART Null";
    case ErrorCode::kStm32GpioInitFailed:
      return "STM32 GPIO Init Failed";
    case ErrorCode::kDisplayPanelInitFailed:
      return "Display Panel Init Failed";
    case ErrorCode::kDisplayManagerInitFailed:
      return "Display Service Init Failed";
    case ErrorCode::kTonePlayerInitFailed:
      return "TonePlayer Init Failed";
    case ErrorCode::kBuzzerInvalidArg:
      return "Buzzer Invalid Argument";
    case ErrorCode::kGpioReinit:
      return "GPIO Driver Re-init";
    case ErrorCode::kGpioInvalidPort:
      return "GPIO Invalid Port";
    case ErrorCode::kTcpServerError:
      return "TCP Server Error";
    case ErrorCode::kM9nNotResponding:
      return "GPS Not Responding";
    case ErrorCode::kM9nConfigProtocolFailed:
      return "M9N Proto Config Failed";
    case ErrorCode::kM9nConfigNavPvtFailed:
      return "M9N NavPvt Config Failed";
    case ErrorCode::kM9nConfigNavDopFailed:
      return "M9N NavDop Config Failed";
    case ErrorCode::kM9nConfigNavCovFailed:
      return "M9N NavCov Config Failed";
    case ErrorCode::kM9nConfigNavEoeFailed:
      return "M9N NavEoe Config Failed";
    case ErrorCode::kM9nConfigRateFailed:
      return "M9N Rate Config Failed";
    case ErrorCode::kM9nConfigDynModelFailed:
      return "M9N DynModel Config Failed";
    case ErrorCode::kM9nConfigConstellationFailed:
      return "M9N Constellation Config Failed";
    case ErrorCode::kM9nConfigItfmFailed:
      return "M9N ITFM Config Failed";
    case ErrorCode::kM9nConfigTimepulseFailed:
      return "M9N Timepulse Config Failed";
    case ErrorCode::kM9nConfigTimepulseBufferError:
      return "M9N TP1 Buffer Mismatch";
    case ErrorCode::kM9nVerifyProtocolFailed:
      return "M9N Verify Proto Failed";
    case ErrorCode::kM9nVerifyNavPvtFailed:
      return "M9N Verify NavPvt Failed";
    case ErrorCode::kM9nVerifyNavDopFailed:
      return "M9N Verify NavDop Failed";
    case ErrorCode::kM9nVerifyNavCovFailed:
      return "M9N Verify NavCov Failed";
    case ErrorCode::kM9nVerifyNavEoeFailed:
      return "M9N Verify NavEoe Failed";
    case ErrorCode::kM9nVerifyRateFailed:
      return "M9N Verify Rate Failed";
    case ErrorCode::kM9nVerifyDynModelFailed:
      return "M9N Verify DynModel Failed";
    case ErrorCode::kM9nVerifyConstellationFailed:
      return "M9N Verify Constellation Failed";
    case ErrorCode::kM9nVerifyItfmFailed:
      return "M9N Verify ITFM Failed";
    case ErrorCode::kM9nVerifyTimepulseFailed:
      return "M9N Verify Timepulse Failed";
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
