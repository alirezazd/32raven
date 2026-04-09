#pragma once

#include <atomic>
#include <cstddef>
#include <cstring>

template <typename T, size_t Size> class RingBuffer {
public:
  static_assert(Size > 0, "Size must be greater than 0");

  bool Push(const T &item) {
    size_t next_head = (head_.load(std::memory_order_relaxed) + 1);
    if (next_head >= Size) {
      next_head = 0;
    }

    if (next_head == tail_.load(std::memory_order_acquire)) {
      return false; // Full
    }

    buffer_[head_.load(std::memory_order_relaxed)] = item;
    head_.store(next_head, std::memory_order_release);
    return true;
  }

  bool Pop(T &item) {
    size_t current_tail = tail_.load(std::memory_order_relaxed);
    if (current_tail == head_.load(std::memory_order_acquire)) {
      return false; // Empty
    }

    item = buffer_[current_tail];
    size_t next_tail = current_tail + 1;
    if (next_tail >= Size) {
      next_tail = 0;
    }
    tail_.store(next_tail, std::memory_order_release);
    return true;
  }

  bool Peek(T &item) const {
    size_t current_tail = tail_.load(std::memory_order_relaxed);
    if (current_tail == head_.load(std::memory_order_acquire)) {
      return false; // Empty
    }
    item = buffer_[current_tail];
    return true;
  }

  bool IsEmpty() const {
    return head_.load(std::memory_order_acquire) ==
           tail_.load(std::memory_order_acquire);
  }

  bool IsFull() const {
    size_t next_head = (head_.load(std::memory_order_relaxed) + 1);
    if (next_head >= Size) {
      next_head = 0;
    }
    return next_head == tail_.load(std::memory_order_acquire);
  }

  size_t Available() const {
    size_t h = head_.load(std::memory_order_acquire);
    size_t t = tail_.load(std::memory_order_acquire);
    if (h >= t) {
      return h - t;
    } else {
      return Size - (t - h);
    }
  }

  size_t Capacity() const { return Size - 1; }

  // Returns number of items written
  size_t PushBlock(const T *p, size_t n) {
    if (n == 0)
      return 0;

    const size_t kAvail = Capacity() - Available();
    if (n > kAvail) {
      n = kAvail; // Partial write
    }
    if (n == 0)
      return 0;

    size_t h = head_.load(std::memory_order_relaxed);
    size_t first_chunk = Size - h;
    if (n < first_chunk) {
      first_chunk = n;
    }

    std::memcpy(&buffer_[h], p, first_chunk * sizeof(T));

    if (n > first_chunk) {
      size_t second_chunk = n - first_chunk;
      std::memcpy(&buffer_[0], p + first_chunk, second_chunk * sizeof(T));
    }

    size_t next_head = h + n;
    if (next_head >= Size) {
      next_head -= Size;
    }
    head_.store(next_head, std::memory_order_release);
    return n;
  }

  size_t ContiguousReadable(const T *&ptr) const {
    size_t t = tail_.load(std::memory_order_acquire);
    size_t h = head_.load(std::memory_order_acquire);
    if (t == h) {
      ptr = nullptr;
      return 0;
    }
    ptr = &buffer_[t];
    if (h > t)
      return h - t;
    return Size - t;
  }

  void Consume(size_t n) {
    if (n == 0)
      return;
    size_t t = tail_.load(std::memory_order_relaxed);
    t += n;
    if (t >= Size)
      t -= Size;
    tail_.store(t, std::memory_order_release);
  }

  void Clear() {
    tail_.store(head_.load(std::memory_order_acquire),
                std::memory_order_release);
  }

private:
  T buffer_[Size];
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
};
