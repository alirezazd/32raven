#pragma once
#include "programmer.hpp"
#include "system.hpp"
#include "uart.hpp"
#include <cstdint>

class EP2 {
public:
  static EP2 &GetInstance() {
    static EP2 instance;
    return instance;
  }

private:
  friend class System;
  void Init(Uart *uart);
  Uart *uart_ = nullptr;
  bool initialized_ = false;
};