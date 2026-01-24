#pragma once
#include "button.hpp"
#include "command_handler.hpp"
#include "fc_link.hpp"
#include "led.hpp"
#include "mavlink.hpp"
#include "programmer.hpp"
#include "tcp_server.hpp"
#include "uart.hpp"
#include "wifi.hpp"

enum class ErrorCode {
  kOk = 0,
  // LED
  kLedTimerInitFailed,
  kLedChannelInitFailed,
  kLedFadeInstallFailed,
  // Button
  kButtonGpioConfigFailed,
  // WiFi
  kWifiNvsInitFailed,
  kWifiNetifInitFailed,
  kWifiEventLoopFailed,
  kWifiInitFailed,
  kWifiSetStorageFailed,
  // UART
  kUartInvalidNumber,
  kUartParamConfigFailed,
  kUartSetPinFailed,
  kUartDriverInstallFailed,
  // Programmer
  kProgrammerUartNull,
  // TCP Server (none currently, uses ErrorHandler)
};

class System {
public:
  static System &GetInstance() {
    static System instance;
    return instance;
  }

  ::LED &Led();
  ::Button &Button();
  ::WifiController &Wifi();
  ::TcpServer &Tcp();
  ::Mavlink &Mavlink();
  ::FcLink &FcLink();
  ::CommandHandler &CommandHandler();
  ::Uart &Uart(::Uart::Id id);
  ::Uart &Uart(); // Default to STM32 for backward compatibility
  ::Programmer &Programmer();
  void Init();
  void StopNetwork();
  void StartNetwork();

private:
  ::Uart uarts_[(int)::Uart::Id::kCount];
  bool initialized_ = false;

  System() = default;
  ~System() = default;
  System(const System &) = delete;
  System &operator=(const System &) = delete;
};
