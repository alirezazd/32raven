#pragma once
#include "state_machine.hpp"

class Programmer {
 public:
  static Programmer &getInstance() {
    static Programmer instance;
    return instance;
  }

  static void init() { getInstance()._init(); }

  void BeginSession();

 private:
  void _init();
  bool initialized_ = false;

  Programmer() = default;
  ~Programmer() = default;
  Programmer(const Programmer &) = delete;
  Programmer &operator=(const Programmer &) = delete;
};