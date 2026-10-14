// clock.hpp - millisecond time source and non-blocking timeouts, microcontroller agnostic.
//
// Libraries never call millis() or a vendor timer directly. They either take
// a `const mcu::Clock&`, or (even simpler) take the current time as a plain
// uint32_t parameter. Both make code testable on a PC with a ManualClock.
//
// Time is an unsigned 32-bit millisecond counter that wraps every ~49.7 days.
// All comparisons here are wrap-safe as long as the intervals involved are
// shorter than about 24.8 days (2^31 ms).
//
// Header-only, C++11, no heap, no platform headers.

#ifndef MCU_CLOCK_HPP
#define MCU_CLOCK_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

// Milliseconds elapsed from `since` to `now`, correct across counter wrap.
inline uint32_t elapsed_ms(uint32_t now, uint32_t since) { return (uint32_t)(now - since); }

// True once `now` has reached or passed `deadline`, correct across counter wrap.
inline bool time_reached(uint32_t now, uint32_t deadline) {
  return (uint32_t)(now - deadline) < 0x80000000u;
}

// "No deadline": returned where a time span is asked for and nothing is pending.
constexpr uint32_t kNever = 0xFFFFFFFFu;

// ---------------------------------------------------------------------------
// Clock interface. Implement it, or use FunctionClock to wrap an existing
// function such as Arduino millis().
// ---------------------------------------------------------------------------
class Clock {
 public:
  virtual uint32_t millis() const = 0;

 protected:
  Clock() {}
  ~Clock() {}
};

// Wraps any `uint32_t f()` function:
//   mcu::FunctionClock clk(millis);                       // Arduino
//   static uint32_t ms() { return (uint32_t)(esp_timer_get_time() / 1000); }
//   mcu::FunctionClock clk(ms);                           // ESP-IDF
//   mcu::FunctionClock clk(HAL_GetTick);                  // STM32 HAL
class FunctionClock final : public Clock {
 public:
  typedef uint32_t (*Fn)();
  explicit FunctionClock(Fn fn) : fn_(fn) {}
  uint32_t millis() const override { return fn_(); }

 private:
  Fn fn_;
};

// A clock you move by hand. For tests, simulations and deterministic replays.
class ManualClock final : public Clock {
 public:
  explicit ManualClock(uint32_t start_ms = 0) : now_(start_ms) {}
  uint32_t millis() const override { return now_; }
  void set(uint32_t ms) { now_ = ms; }
  void advance(uint32_t ms) { now_ = (uint32_t)(now_ + ms); }

 private:
  uint32_t now_;
};

// ---------------------------------------------------------------------------
// Timeout: a one-shot or periodic non-blocking timer. It stores no clock; you
// pass the current time in, so it works with any time source.
//
//   mcu::Timeout t;
//   t.start(clk.millis(), 500);
//   ...
//   if (t.expired(clk.millis())) { ... }
//
// Periodic use, with no drift:
//   t.start(now, 1000);
//   if (t.poll_periodic(now)) { /* runs once per second */ }
// ---------------------------------------------------------------------------
class Timeout {
 public:
  Timeout() : deadline_(0), period_(0), active_(false) {}

  // Arms the timer to expire `duration_ms` after `now`.
  void start(uint32_t now, uint32_t duration_ms) {
    deadline_ = (uint32_t)(now + duration_ms);
    period_ = duration_ms;
    active_ = true;
  }

  // Re-arms with the same duration, counted from `now`.
  void restart(uint32_t now) { deadline_ = (uint32_t)(now + period_); active_ = true; }

  void stop() { active_ = false; }

  // A stopped timer is never expired.
  bool active() const { return active_; }
  bool expired(uint32_t now) const { return active_ && time_reached(now, deadline_); }

  // Milliseconds left, or 0 if expired or stopped.
  uint32_t remaining(uint32_t now) const {
    if (!active_ || time_reached(now, deadline_)) return 0;
    return (uint32_t)(deadline_ - now);
  }

  // For periodic work. Returns true once per period, keeping the schedule
  // anchored to the original start so the average rate does not drift. If the
  // caller fell more than a whole period behind, it skips ahead instead of
  // firing a burst of catch-up events.
  bool poll_periodic(uint32_t now) {
    if (!expired(now)) return false;
    deadline_ = (uint32_t)(deadline_ + period_);
    if (time_reached(now, deadline_)) deadline_ = (uint32_t)(now + period_);
    return true;
  }

 private:
  uint32_t deadline_;
  uint32_t period_;
  bool active_;
};

}  // namespace mcu

#endif  // MCU_CLOCK_HPP
