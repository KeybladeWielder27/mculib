// ring_buffer.hpp - fixed-capacity FIFO ring buffer, microcontroller agnostic.
//
// - Header-only, C++11, no heap, no exceptions, no RTTI, no platform headers.
// - Capacity N must be a power of two (checked at compile time).
// - Single-producer / single-consumer safe WITHOUT locks, provided that:
//     * exactly one context calls the producer functions (push, write), and
//       exactly one context calls the consumer functions (pop, peek, read,
//       discard, clear), and
//     * reads/writes of IndexT are atomic on your target. The default IndexT is
//       the smallest unsigned type that fits, which is 8 bits (atomic) for
//       N <= 128. On an 8-bit MCU with N > 128, wrap calls in a critical
//       section or pick a smaller N.
//   Typical use: UART RX interrupt pushes, main loop pops.
// - If you need multiple producers or consumers, guard it externally.
//
// Usage:
//   mcu::RingBuffer<uint8_t, 64> rx;
//   rx.push(byte);
//   uint8_t b;
//   if (rx.pop(b)) { ... }

#ifndef MCU_RING_BUFFER_HPP
#define MCU_RING_BUFFER_HPP

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__) || defined(__clang__)
#define MCU_COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")
#else
#define MCU_COMPILER_BARRIER() ((void)0)
#endif

namespace mcu {

namespace detail {
template <bool Cond, typename A, typename B> struct Select { typedef A type; };
template <typename A, typename B> struct Select<false, A, B> { typedef B type; };
}  // namespace detail

template <typename T, size_t N,
          typename IndexT = typename detail::Select<
              (N <= 128), uint8_t,
              typename detail::Select<(N <= 32768), uint16_t, uint32_t>::type>::type>
class RingBuffer {
  static_assert(N >= 2, "capacity must be at least 2");
  static_assert((N & (N - 1)) == 0, "capacity must be a power of two");
  static_assert(sizeof(IndexT) * 8 > 0 && (size_t)((IndexT)~(IndexT)0) >= (2 * N - 1),
                "IndexT too small: must hold 2*N-1");

 public:
  typedef T value_type;
  typedef IndexT index_type;

  // buf_ is zeroed once so compilers that cannot see through the volatile
  // indices (GCC 11 at -O2) do not warn about reading uninitialized slots.
  RingBuffer() : buf_(), head_(0), tail_(0) {}

  static constexpr size_t capacity() { return N; }

  size_t size() const { return (IndexT)(head_ - tail_); }
  size_t free_space() const { return N - size(); }
  bool empty() const { return head_ == tail_; }
  bool full() const { return size() == N; }

  // ---- producer side ----

  // Returns false (and drops the item) if full.
  bool push(const T& item) {
    if (full()) return false;
    buf_[head_ & kMask] = item;
    MCU_COMPILER_BARRIER();  // data must be visible before the index moves
    head_ = (IndexT)(head_ + 1);
    return true;
  }

  // Writes up to n items; returns how many were actually written.
  size_t write(const T* src, size_t n) {
    size_t room = free_space();
    if (n > room) n = room;
    IndexT h = head_;
    for (size_t i = 0; i < n; ++i) {
      buf_[h & kMask] = src[i];
      h = (IndexT)(h + 1);
    }
    MCU_COMPILER_BARRIER();
    head_ = h;
    return n;
  }

  // ---- consumer side ----

  // Returns false if empty.
  bool pop(T& out) {
    if (empty()) return false;
    out = buf_[tail_ & kMask];
    MCU_COMPILER_BARRIER();  // finish reading before releasing the slot
    tail_ = (IndexT)(tail_ + 1);
    return true;
  }

  // Look at the oldest item without removing it.
  bool peek(T& out) const {
    if (empty()) return false;
    out = buf_[tail_ & kMask];
    return true;
  }

  // Reads up to n items; returns how many were actually read.
  size_t read(T* dst, size_t n) {
    size_t avail = size();
    if (n > avail) n = avail;
    IndexT t = tail_;
    for (size_t i = 0; i < n; ++i) {
      dst[i] = buf_[t & kMask];
      t = (IndexT)(t + 1);
    }
    MCU_COMPILER_BARRIER();
    tail_ = t;
    return n;
  }

  // Drops up to n oldest items; returns how many were dropped.
  size_t discard(size_t n) {
    size_t avail = size();
    if (n > avail) n = avail;
    tail_ = (IndexT)(tail_ + n);
    return n;
  }

  // Empties the buffer. Consumer-side call: moves tail up to head.
  void clear() { tail_ = head_; }

 private:
  static constexpr IndexT kMask = (IndexT)(N - 1);

  T buf_[N];
  volatile IndexT head_;  // free-running; written only by the producer
  volatile IndexT tail_;  // free-running; written only by the consumer
};

}  // namespace mcu

#endif  // MCU_RING_BUFFER_HPP
