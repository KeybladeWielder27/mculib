// timers.hpp - small non-blocking timing helpers, microcontroller agnostic.
//
//   Stopwatch   measure how long something took
//   Throttle    allow an action at most once per interval (rate limit)
//   Backoff     growing delays for retries: 100, 200, 400 ... capped
//   RetryTimer  Backoff plus a deadline: "try again when it is time"
//
// Like everything in this collection they take the current time as a
// parameter (a wrapping 32-bit millisecond count), so they work with any
// clock and can be tested by hand. They never block.
//
// For a one-shot or single periodic timer see Timeout in hal/clock.hpp; for
// many periodic jobs see scheduler.hpp.
//
// Header-only, C++11, no heap.

#ifndef MCU_TIMERS_HPP
#define MCU_TIMERS_HPP

#include <stddef.h>
#include <stdint.h>

#include "hal/clock.hpp"

namespace mcu {

// ---------------------------------------------------------------------------
// Stopwatch. Measures up to ~49 days; beyond that the count wraps.
// ---------------------------------------------------------------------------
class Stopwatch {
 public:
  Stopwatch() : start_(0), frozen_(0), running_(false) {}

  void start(uint32_t now) {
    start_ = now;
    frozen_ = 0;
    running_ = true;
  }

  // Freezes the reading and returns it.
  uint32_t stop(uint32_t now) {
    if (running_) {
      frozen_ = elapsed_ms(now, start_);
      running_ = false;
    }
    return frozen_;
  }

  // Milliseconds since start() (or the frozen value after stop()).
  uint32_t elapsed(uint32_t now) const { return running_ ? elapsed_ms(now, start_) : frozen_; }

  // Returns the elapsed time and starts timing again from `now`.
  uint32_t lap(uint32_t now) {
    const uint32_t e = elapsed(now);
    if (running_) start_ = now;
    return e;
  }

  bool running() const { return running_; }
  void reset() {
    running_ = false;
    frozen_ = 0;
  }

 private:
  uint32_t start_;
  uint32_t frozen_;
  bool running_;
};

// ---------------------------------------------------------------------------
// Throttle: lets an action through at most once per `interval_ms`. The first
// request is allowed immediately.
//
//   mcu::Throttle log_limit(1000);
//   if (log_limit.allow(now)) print_status();     // at most once a second
// ---------------------------------------------------------------------------
class Throttle {
 public:
  explicit Throttle(uint32_t interval_ms) : interval_(interval_ms), next_(0), armed_(false) {}

  // True if the action may happen now; if so the interval starts over.
  bool allow(uint32_t now) {
    if (armed_ && !time_reached(now, next_)) return false;
    armed_ = true;
    next_ = (uint32_t)(now + interval_);
    return true;
  }

  // Milliseconds until allow() would say yes (0 if it would now).
  uint32_t wait_ms(uint32_t now) const {
    if (!armed_ || time_reached(now, next_)) return 0;
    return (uint32_t)(next_ - now);
  }

  // Applies from the next allowed action; the window already running keeps its end time.
  void set_interval(uint32_t interval_ms) { interval_ = interval_ms; }

  // Forget the last action: the next request is allowed at once.
  void reset() { armed_ = false; }

 private:
  uint32_t interval_;
  uint32_t next_;
  bool armed_;
};

// Reduces `delay_ms` by a pseudo-random amount of up to `percent` percent
// (0..100), using the caller's random number `rnd`. Spreads out retries so
// that many devices do not all try again at the same instant. The result is
// never longer than `delay_ms`, and only rnd's low 16 bits are used.
inline uint32_t jitter_down(uint32_t delay_ms, uint32_t rnd, uint8_t percent) {
  if (percent > 100) percent = 100;
  // span = floor(delay * percent / 100), without 64-bit math
  const uint32_t span = (delay_ms / 100u) * percent + ((delay_ms % 100u) * percent) / 100u;
  const uint32_t r = rnd & 0xFFFFu;
  const uint32_t offset = (span >> 16) * r + (((span & 0xFFFFu) * r) >> 16);  // span * r / 65536
  return delay_ms - offset;
}

// ---------------------------------------------------------------------------
// Backoff: a growing sequence of delays. next() returns the current delay and
// then grows it by `factor`, never past `max_ms`.
//
//   mcu::Backoff b(100, 5000);        // 100, 200, 400, ..., 3200, 5000, 5000, ...
// ---------------------------------------------------------------------------
class Backoff {
 public:
  Backoff(uint32_t initial_ms, uint32_t max_ms, uint8_t factor = 2)
      : initial_(initial_ms),
        max_(max_ms < initial_ms ? initial_ms : max_ms),
        factor_(factor < 2 ? 2 : factor),
        cur_(initial_ms),
        calls_(0) {}

  uint32_t next() {
    const uint32_t d = cur_;
    if (calls_ != 0xFFFFFFFFu) ++calls_;
    if (cur_ >= max_ || cur_ > max_ / factor_) {
      cur_ = max_;
    } else {
      cur_ *= factor_;
    }
    return d;
  }

  uint32_t peek() const { return cur_; }          // what next() will return
  uint32_t count() const { return calls_; }       // how many times next() was called
  void reset() {
    cur_ = initial_;
    calls_ = 0;
  }

 private:
  uint32_t initial_;
  uint32_t max_;
  uint8_t factor_;
  uint32_t cur_;
  uint32_t calls_;
};

// ---------------------------------------------------------------------------
// RetryTimer: tells you when to try again, with backoff and an optional limit.
//
//   mcu::RetryTimer link(500, 30000);   // 0.5 s, doubling, up to 30 s
//   link.start(now);                    // begin; first attempt is due at once
//   ...
//   if (link.due(now)) {
//     if (try_connect()) link.succeeded();
//     else               link.failed(now);        // next attempt after the backoff
//   }
//   if (link.gave_up()) { ... }                   // only with max_attempts set
//
// `max_attempts` of 0 means retry forever. Pass a random number to failed() and
// call set_jitter_percent() to spread retries out.
// ---------------------------------------------------------------------------
class RetryTimer {
 public:
  RetryTimer(uint32_t initial_ms, uint32_t max_ms, uint8_t factor = 2, uint32_t max_attempts = 0)
      : backoff_(initial_ms, max_ms, factor),
        next_(0),
        failures_(0),
        max_attempts_(max_attempts),
        jitter_(0),
        active_(false),
        gave_up_(false) {}

  void set_jitter_percent(uint8_t percent) { jitter_ = percent > 100 ? 100 : percent; }

  // Begins a series of attempts. The first one is due immediately.
  void start(uint32_t now) {
    backoff_.reset();
    next_ = now;
    failures_ = 0;
    active_ = true;
    gave_up_ = false;
  }

  // True when an attempt should be made now.
  bool due(uint32_t now) const { return active_ && !gave_up_ && time_reached(now, next_); }

  // Record a failed attempt: schedules the next one after the backoff delay,
  // or gives up once `max_attempts` have failed.
  void failed(uint32_t now, uint32_t rnd = 0) {
    if (!active_ || gave_up_) return;
    ++failures_;
    if (max_attempts_ != 0 && failures_ >= max_attempts_) {
      gave_up_ = true;
      return;
    }
    next_ = (uint32_t)(now + jitter_down(backoff_.next(), rnd, jitter_));
  }

  // Record success: stops the series and resets the backoff.
  void succeeded() {
    backoff_.reset();
    failures_ = 0;
    active_ = false;
    gave_up_ = false;
  }

  bool active() const { return active_ && !gave_up_; }
  bool gave_up() const { return gave_up_; }
  uint32_t failures() const { return failures_; }

  // Milliseconds until the next attempt is due: 0 if due now, kNever if idle or given up.
  uint32_t wait_ms(uint32_t now) const {
    if (!active()) return kNever;
    return time_reached(now, next_) ? 0 : (uint32_t)(next_ - now);
  }

 private:
  Backoff backoff_;
  uint32_t next_;
  uint32_t failures_;
  uint32_t max_attempts_;
  uint8_t jitter_;
  bool active_;
  bool gave_up_;
};

}  // namespace mcu

#endif  // MCU_TIMERS_HPP
