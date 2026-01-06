#pragma once
#include "button.hpp"
#include "led.hpp"
#include "programmer.hpp"
#include "tcp_server.hpp"
#include "uart.hpp"
#include "wifi.hpp"

class System {
public:
  static System &GetInstance() {
    static System instance;
    return instance;
  }

  LED &Led();
  ::Button &Button();
  ::WifiController &Wifi();
  ::TcpServer &Tcp();
  ::Uart &Uart();
  ::Programmer &Programmer();
  void Init();

private:
  bool initialized_ = false;

  System() = default;
  ~System() = default;
  System(const System &) = delete;
  System &operator=(const System &) = delete;
};
