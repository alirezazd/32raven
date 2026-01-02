#ifndef CORE_SYSTEM_HPP
#define CORE_SYSTEM_HPP

#include "board.h"

class System {
public:
  static System &getInstance() {
    static System instance;
    return instance;
  }

  struct Config {
    RCC_OscInitTypeDef osc;
    RCC_ClkInitTypeDef clk;
    uint32_t flashLatency;
    uint32_t voltageScaling;
  };

  static void init(const Config &config) { getInstance()._init(config); }
  void _init(const Config &config);

private:
  bool initialized_ = false;
  System();
  ~System() {}
  System(const System &) = delete;
  System &operator=(const System &) = delete;
  void SystemClock_Config(const Config &config);
};
#endif // CORE_SYSTEM_HPP