#pragma once
#include "button.hpp"
#include "http_server.hpp"
#include "led.hpp"
#include "uart.hpp"
#include "wifi.hpp"

class System {
public:
  static System &getInstance() {
    static System instance;
    return instance;
  }

  static void init() { getInstance()._init(); }

  LED &led();
  Button &button();
  WifiController &wifi();
  HttpServer &http();
  Uart &uart();

private:
  void _init();

  bool initialized_ = false;

  System() = default;
  ~System() = default;
  System(const System &) = delete;
  System &operator=(const System &) = delete;
};
