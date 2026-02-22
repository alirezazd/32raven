#pragma once

#include "message.hpp"
#include <cstddef>

namespace Epistole {

/**
 * @brief Generic Command Dispatcher
 *
 * Maps MsgId to a Handler function.
 *
 * @tparam Context The application context type (e.g. AppContext)
 */
template <typename Context> class Dispatcher {
public:
  // Handler signature
  using Handler = void (*)(Context &ctx, const message::Packet &pkt);

  struct Entry {
    message::MsgId id;
    Handler handler;
  };

  /**
   * @brief Construct a Dispatcher
   *
   * @param entries Pointer to static array of Entries
   * @param count Number of entries
   */
  constexpr Dispatcher(const Entry *entries, size_t count)
      : entries_(entries), count_(count) {}

  /**
   * @brief Find and execute handler for packet
   *
   * @param ctx Application Context
   * @param pkt Received Packet
   * @return true if handler found and executed
   */
  bool Dispatch(Context &ctx, const message::Packet &pkt) const {
    for (size_t i = 0; i < count_; ++i) {
      // Check ID match
      if ((uint8_t)entries_[i].id == (uint8_t)pkt.header.id) {
        if (entries_[i].handler) {
          entries_[i].handler(ctx, pkt);
          return true;
        }
      }
    }
    return false;
  }

private:
  const Entry *entries_;
  size_t count_;
};

} // namespace Epistole
