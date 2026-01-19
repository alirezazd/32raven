#include "ep2_receiver.hpp"

static constexpr char kTag[] = "ep2";

void EP2::Init(Uart *uart) {
  uart_ = uart;
  initialized_ = true;
}
