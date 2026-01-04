#pragma once
#include "button.hpp"
#include "http_server.hpp"
#include "led.hpp"
#include "programmer.hpp"
#include "uart.hpp"
#include "wifi.hpp"

class System {
public:
  static System &GetInstance() {
    static System instance;
    return instance;
  }

  LED &Led();
  Button &Button();
  WifiController &Wifi();
  HttpServer &Http();
  Uart &Uart();
  Programmer &Programmer();

private:
  void Init();
  bool initialized_ = false;

  System() = default;
  ~System() = default;
  System(const System &) = delete;
  System &operator=(const System &) = delete;
};
