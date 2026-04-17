#pragma once
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#include "button.hpp"
#include "buzzer.hpp"
#include "command_handler.hpp"
#include "fc_link.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include "mavlink.hpp"
#include "panic.hpp"
#include "programmer.hpp"
#include "ssd1306_panel.hpp"
#include "tcp_server.hpp"
#include "timebase.hpp"
#include "tone_player.hpp"
#include "uart.hpp"
#include "udp_server.hpp"
#include "ui.hpp"
#include "wifi.hpp"

class System {
 public:
  static System &GetInstance() {
    static System instance;
    return instance;
  }

  // Component identifiers used by InitComponent().
  enum class Component {
    kLed,
    kBuzzer,
    kTonePlayer,
    kButton,
    kDisplayI2c,
    kDisplayPanel,
    kUi,
    kWifi,
    kTcpServer,
    kUdpServer,
    kFcLinkUart,
    kProgrammer,
    kMavlink,
    kFcLink,
    kCommandHandler
  };

  ::LED &Led() { return ::LED::GetInstance(); }
  ::TonePlayer &TonePlayer() { return ::TonePlayer::GetInstance(); }
  ::Buzzer &Buzzer() { return ::Buzzer::GetInstance(); }
  ::Button &Button() { return ::Button::GetInstance(); }
  ::Timebase &Timebase() { return ::Timebase::GetInstance(); }
  ::I2cDisplay &DisplayI2c() { return ::I2cDisplay::GetInstance(); }
  ::Ssd1306Panel &DisplayPanel() { return ::Ssd1306Panel::GetInstance(); }
  ::Ui &Ui() { return ::Ui::GetInstance(); }

  ::WifiController &Wifi() { return ::WifiController::GetInstance(); }
  ::TcpServer &Tcp() { return ::TcpServer::GetInstance(); }
  ::UdpServer &Udp() { return ::UdpServer::GetInstance(); }
  ::Mavlink &Mavlink() { return ::Mavlink::GetInstance(); }
  ::FcLink &FcLink() { return ::FcLink::GetInstance(); }
  ::CommandHandler &CommandHandler() { return ::CommandHandler::GetInstance(); }
  ::UartFcLink &FcLinkUart() { return ::UartFcLink::GetInstance(); }
  ::Programmer &Programmer() { return ::Programmer::GetInstance(); }
  void Init();
  void StopNetwork();
  void StartNetwork();
  void SetMainTaskHandle(TaskHandle_t task_handle);
  void Halt();

 private:
  void InitComponent(Component c);
  System() = default;
  ~System() = default;
  System(const System &) = delete;
  System &operator=(const System &) = delete;

  TaskHandle_t main_task_handle_ = nullptr;
};

inline System &Sys() { return System::GetInstance(); }
