// filters.hpp - small integer signal filters for microcontrollers.
//
//   MovingAverage<T,N>   average of the last N samples (running sum, O(1))
//   Ema<T,Shift>         exponential moving average, alpha = 1/2^Shift, no division
//   MedianFilter<T,N>    median of the last N samples (kills spikes)
//   SlewLimiter<T>       limits how fast a value may rise or fall
//   Schmitt<T>           comparator with hysteresis
//   RunningStats<T>      count, min, max and mean since reset
//
// All of them are integer only (no floating point, no heap), work for any
// 8/16/32/64-bit integer type T, and take one sample per update() call:
//
//   mcu::MovingAverage<uint16_t, 8> avg;
//   uint16_t smooth = avg.update(analogRead(A0));
//
// Rounding is to nearest (halves away from zero) unless noted. Not interrupt
// safe: call update() from one context.
//
// Header-only, C++11. Needs fixed.hpp.

#ifndef MCU_FILTERS_HPP
#define MCU_FILTERS_HPP

#include <stddef.h>
#include <stdint.h>

#include "fixed.hpp"

namespace mcu {

// ---------------------------------------------------------------------------
// MovingAverage: mean of the last N samples. Until N samples have arrived it
// averages what it has (so the first output equals the first input).
//
// `Acc` holds the running sum; the default is wide enough for N samples of any
// value of T when N <= 65535 for 8/16-bit T, and for 32-bit T up to N = 2^31.
// ---------------------------------------------------------------------------
template <typename T, size_t N, typename Acc = typename detail::IntTraits<T>::wide>
class MovingAverage {
  static_assert(N >= 1, "N must be at least 1");

 public:
  MovingAverage() : sum_(0), head_(0), count_(0) {}

  // Adds a sample and returns the new average.
  T update(T x) {
    if (count_ == N) {
      sum_ -= (Acc)buf_[head_];
    } else {
      ++count_;
    }
    buf_[head_] = x;
    sum_ += (Acc)x;
    head_ = (head_ + 1 == N) ? 0 : head_ + 1;
    return value();
  }

  // Current average (0 before the first sample).
  T value() const {
    if (count_ == 0) return 0;
    return (T)detail::div_round<Acc>(sum_, (Acc)count_);
  }

  bool full() const { return count_ == N; }
  size_t count() const { return count_; }
  static constexpr size_t capacity() { return N; }
  Acc sum() const { return sum_; }  // exact sum of the samples held

  void reset() {
    sum_ = 0;
    head_ = 0;
    count_ = 0;
  }

 private:
  T buf_[N];
  Acc sum_;
  size_t head_;
  size_t count_;
};

// ---------------------------------------------------------------------------
// Ema: exponential moving average (a one-pole low-pass filter) with
// alpha = 1 / 2^Shift. Bigger Shift = smoother and slower: Shift 1 weighs the
// new sample by 1/2, Shift 4 by 1/16. The time constant is about 2^Shift
// samples.
//
// The first sample primes the filter (the output starts at that sample, not at
// zero). The output converges exactly to a constant input, from above or
// below, and never overshoots it.
//
// Uses an arithmetic right shift on signed accumulators, which every
// compiler for the usual MCUs implements.
// ---------------------------------------------------------------------------
template <typename T, unsigned Shift, typename Acc = typename detail::IntTraits<T>::wide>
class Ema {
  static_assert(Shift >= 1 && Shift <= 30, "Shift must be 1..30");
  static_assert(sizeof(Acc) * 8 >= sizeof(T) * 8 + Shift + 1,
                "Acc too narrow for T and Shift; use a wider Acc");

 public:
  Ema() : acc_(0), primed_(false) {}

  T update(T x) {
    if (!primed_) {
      acc_ = (Acc)x * scale();
      primed_ = true;
    } else {
      const Acc y = out();
      acc_ = (Acc)(acc_ + ((Acc)x - y));
    }
    return value();
  }

  T value() const { return (T)out(); }
  bool primed() const { return primed_; }

  // Start from a known value instead of waiting for the first sample.
  void reset(T initial) {
    acc_ = (Acc)initial * scale();
    primed_ = true;
  }
  // Forget everything; the next sample primes the filter again.
  void reset() {
    acc_ = 0;
    primed_ = false;
  }

 private:
  static constexpr Acc scale() { return (Acc)((Acc)1 << Shift); }
  static constexpr Acc half() { return (Acc)((Acc)1 << (Shift - 1)); }
  Acc out() const { return (Acc)((Acc)(acc_ + half()) >> Shift); }

  Acc acc_;
  bool primed_;
};

// ---------------------------------------------------------------------------
// MedianFilter: median of the last N samples. Removes isolated spikes without
// blurring steps. Cost per update is O(N), so keep N small (3..15).
//
// For an even number of samples (including while filling) the lower of the
// two middle values is returned, so the output is always a real sample.
// ---------------------------------------------------------------------------
template <typename T, size_t N>
class MedianFilter {
  static_assert(N >= 1, "N must be at least 1");

 public:
  MedianFilter() : head_(0), count_(0) {}

  T update(T x) {
    size_t m;  // samples that stay in sorted_ while x is inserted
    if (count_ < N) {
      m = count_;
      ++count_;
    } else {
      // Full: remove the oldest sample from the sorted list.
      const T old = ring_[head_];
      size_t i = 0;
      while (sorted_[i] != old) ++i;  // always found
      for (; i + 1 < N; ++i) sorted_[i] = sorted_[i + 1];
      m = N - 1;
    }
    ring_[head_] = x;
    head_ = (head_ + 1 == N) ? 0 : head_ + 1;
    // Insert x among the m sorted survivors.
    while (m > 0 && sorted_[m - 1] > x) {
      sorted_[m] = sorted_[m - 1];
      --m;
    }
    sorted_[m] = x;
    return value();
  }

  T value() const { return count_ == 0 ? (T)0 : sorted_[(count_ - 1) / 2]; }

  bool full() const { return count_ == N; }
  size_t count() const { return count_; }
  void reset() {
    head_ = 0;
    count_ = 0;
  }

 private:
  T ring_[N];
  T sorted_[N];
  size_t head_;
  size_t count_;
};

// ---------------------------------------------------------------------------
// SlewLimiter: output follows the input but may rise by at most `max_up` and
// fall by at most `max_down` per update. The first sample passes through.
// Steps are given in T's unsigned counterpart so the full range of T works.
// ---------------------------------------------------------------------------
template <typename T>
class SlewLimiter {
  typedef typename detail::IntTraits<T>::unsigned_type U;

 public:
  SlewLimiter(U max_up, U max_down) : up_(max_up), down_(max_down), y_(0), primed_(false) {}
  explicit SlewLimiter(U max_step) : up_(max_step), down_(max_step), y_(0), primed_(false) {}

  T update(T x) {
    if (!primed_) {
      y_ = x;
      primed_ = true;
    } else if (x > y_) {
      const U diff = (U)((U)x - (U)y_);
      y_ = diff > up_ ? (T)(U)((U)y_ + up_) : x;
    } else if (x < y_) {
      const U diff = (U)((U)y_ - (U)x);
      y_ = diff > down_ ? (T)(U)((U)y_ - down_) : x;
    }
    return y_;
  }

  T value() const { return y_; }
  void set_limits(U max_up, U max_down) {
    up_ = max_up;
    down_ = max_down;
  }
  void reset() {
    y_ = 0;
    primed_ = false;
  }
  void reset(T value) {
    y_ = value;
    primed_ = true;
  }

 private:
  U up_;
  U down_;
  T y_;
  bool primed_;
};

// ---------------------------------------------------------------------------
// Schmitt: threshold with hysteresis. Turns on when the input reaches `high`
// and off when it falls to `low`; in between it keeps its state.
// ---------------------------------------------------------------------------
template <typename T>
class Schmitt {
 public:
  Schmitt(T low, T high, bool initial = false)
      : low_(low < high ? low : high), high_(low < high ? high : low), state_(initial) {}

  bool update(T x) {
    if (state_) {
      if (x <= low_) state_ = false;
    } else {
      if (x >= high_) state_ = true;
    }
    return state_;
  }

  bool state() const { return state_; }
  void reset(bool state = false) { state_ = state; }

 private:
  T low_;
  T high_;
  bool state_;
};

// ---------------------------------------------------------------------------
// RunningStats: count, minimum, maximum and mean of all samples since reset.
// The sum is 64-bit, so it is exact for about 4 billion samples of any T.
// ---------------------------------------------------------------------------
template <typename T>
class RunningStats {
  typedef typename detail::IntBySize<8, detail::IntTraits<T>::is_signed != 0>::wide Sum;

 public:
  RunningStats() : count_(0), min_(0), max_(0), sum_(0) {}

  void update(T x) {
    if (count_ == 0) {
      min_ = max_ = x;
    } else {
      if (x < min_) min_ = x;
      if (x > max_) max_ = x;
    }
    if (count_ != 0xFFFFFFFFu) ++count_;
    sum_ += (Sum)x;
  }

  uint32_t count() const { return count_; }
  T min() const { return min_; }  // 0 before the first sample
  T max() const { return max_; }
  T mean() const {  // rounded; 0 before the first sample
    return count_ == 0 ? (T)0 : (T)detail::div_round<Sum>(sum_, (Sum)count_);
  }
  Sum sum() const { return sum_; }

  void reset() {
    count_ = 0;
    min_ = max_ = 0;
    sum_ = 0;
  }

 private:
  uint32_t count_;
  T min_;
  T max_;
  Sum sum_;
};

}  // namespace mcu

#endif  // MCU_FILTERS_HPP
