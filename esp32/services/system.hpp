#pragma once
#include "button.hpp"
#include "buzzer.hpp"
#include "command_handler.hpp"
#include "fc_link.hpp"
#include "led.hpp"
#include "mavlink.hpp"
#include "panic.hpp"
#include "programmer.hpp"
#include "tcp_server.hpp"
#include "tone_player.hpp"
#include "uart.hpp"
#include "wifi.hpp"

class System {
public:
  static System &GetInstance() {
    static System instance;
    return instance;
  }

  // INITIALIZATION ORDER: reorder these enum values to change init sequence.
  enum class Component {
    kLed = 0,
    kBuzzer,
    kTonePlayer,
    kButton,
    kWifi,
    kTcpServer,
    kFcLinkUart,
    kEp2Uart,
    kProgrammer,
    kMavlink,
    kFcLink,
    kCommandHandler,
    kCount
  };

  ::LED &Led() { return ::LED::GetInstance(); }
  ::TonePlayer &TonePlayer() { return ::TonePlayer::GetInstance(); }
  ::Buzzer &Buzzer() { return ::Buzzer::GetInstance(); }
  ::Button &Button() { return ::Button::GetInstance(); }

  ::WifiController &Wifi() { return ::WifiController::GetInstance(); }
  ::TcpServer &Tcp() { return ::TcpServer::GetInstance(); }
  ::Mavlink &Mavlink() { return ::Mavlink::GetInstance(); }
  ::FcLink &FcLink() { return ::FcLink::GetInstance(); }
  ::CommandHandler &CommandHandler() { return ::CommandHandler::GetInstance(); }
  ::UartFcLink &FcLinkUart() { return ::UartFcLink::GetInstance(); }
  ::UartEp2 &Ep2Uart() { return ::UartEp2::GetInstance(); }
  ::Programmer &Programmer() { return ::Programmer::GetInstance(); }
  void Init();
  void StopNetwork();
  void StartNetwork();

private:
  void InitComponent(Component c);

  bool initialized_ = false;

  System() = default;
  ~System() = default;
  System(const System &) = delete;
  System &operator=(const System &) = delete;
};
