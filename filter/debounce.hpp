// debounce.hpp - switch and button debouncing, microcontroller agnostic.
//
//   SampleDebouncer  count-based: call at a fixed rate (e.g. every 1 ms tick)
//   Debouncer        time-based: call whenever you like, passing the time
//   Button           Debouncer plus press / release / click / long-press /
//                    multi-click events
//
//   mcu::Button btn;                                  // active low, 20 ms debounce
//   ...
//   uint8_t ev = btn.update(digitalRead(PIN), clock.millis());
//   if (ev & mcu::Button::kClick)     { ... }
//   if (ev & mcu::Button::kLongPress) { ... }
//
// Time is a wrapping 32-bit millisecond count passed in by the caller
// (see hal/clock.hpp). Nothing blocks. Header-only, C++11, no heap.

#ifndef MCU_DEBOUNCE_HPP
#define MCU_DEBOUNCE_HPP

#include <stddef.h>
#include <stdint.h>

#include "hal/clock.hpp"

namespace mcu {

// ---------------------------------------------------------------------------
// SampleDebouncer: an integrator. Each update(raw) moves a counter one step
// toward 0 or `threshold`; the output flips only when the counter reaches the
// end. So the raw input must agree for `threshold` consecutive samples to
// change the output, and a glitch is forgiven gradually rather than reset.
// Call it at a steady rate.
// ---------------------------------------------------------------------------
class SampleDebouncer {
 public:
  explicit SampleDebouncer(uint16_t threshold, bool initial = false)
      : threshold_(threshold == 0 ? 1 : threshold),
        count_(initial ? (threshold == 0 ? 1 : threshold) : 0),
        state_(initial),
        changed_(false) {}

  // Feed one raw sample; returns the debounced state.
  bool update(bool raw) {
    changed_ = false;
    if (raw) {
      if (count_ < threshold_) ++count_;
    } else if (count_ > 0) {
      --count_;
    }
    if (!state_ && count_ == threshold_) {
      state_ = true;
      changed_ = true;
    } else if (state_ && count_ == 0) {
      state_ = false;
      changed_ = true;
    }
    return state_;
  }

  bool state() const { return state_; }
  bool changed() const { return changed_; }  // the last update() changed the output
  bool rose() const { return changed_ && state_; }
  bool fell() const { return changed_ && !state_; }

  void reset(bool state = false) {
    state_ = state;
    count_ = state ? threshold_ : 0;
    changed_ = false;
  }

 private:
  uint16_t threshold_;
  uint16_t count_;
  bool state_;
  bool changed_;
};

// ---------------------------------------------------------------------------
// Debouncer: time-based. The output changes once the raw input has differed
// from it continuously for `press_ms` (going true) or `release_ms` (going
// false). Any return to the current state cancels the change in progress.
// A threshold of 0 makes that direction immediate.
// ---------------------------------------------------------------------------
class Debouncer {
 public:
  explicit Debouncer(uint32_t press_ms, uint32_t release_ms, bool initial = false)
      : press_ms_(press_ms),
        release_ms_(release_ms),
        since_(0),
        changed_at_(0),
        state_(initial),
        pending_(false),
        changed_(false),
        timed_(false) {}

  explicit Debouncer(uint32_t both_ms = 20)
      : press_ms_(both_ms),
        release_ms_(both_ms),
        since_(0),
        changed_at_(0),
        state_(false),
        pending_(false),
        changed_(false),
        timed_(false) {}

  // Feed a raw reading taken at time `now`; returns the debounced state.
  bool update(bool raw, uint32_t now) {
    changed_ = false;
    if (!timed_) {
      timed_ = true;
      changed_at_ = now;
    }
    if (raw == state_) {
      pending_ = false;
    } else {
      if (!pending_) {
        pending_ = true;
        since_ = now;
      }
      if (elapsed_ms(now, since_) >= (raw ? press_ms_ : release_ms_)) {
        state_ = raw;
        pending_ = false;
        changed_ = true;
        changed_at_ = now;
      }
    }
    return state_;
  }

  bool state() const { return state_; }
  bool changed() const { return changed_; }
  bool rose() const { return changed_ && state_; }
  bool fell() const { return changed_ && !state_; }

  // How long the output has been in its current state (since the last change,
  // or since the first update() if it has not changed yet).
  uint32_t held_ms(uint32_t now) const { return timed_ ? elapsed_ms(now, changed_at_) : 0; }

  void set_thresholds(uint32_t press_ms, uint32_t release_ms) {
    press_ms_ = press_ms;
    release_ms_ = release_ms;
  }

  void reset(bool state = false) {
    state_ = state;
    pending_ = false;
    changed_ = false;
    timed_ = false;
  }

 private:
  uint32_t press_ms_;
  uint32_t release_ms_;
  uint32_t since_;
  uint32_t changed_at_;
  bool state_;
  bool pending_;
  bool changed_;
  bool timed_;
};

// ---------------------------------------------------------------------------
// Button: debounced button with events. update() returns a bit mask of events
// that happened on this call (usually 0):
//
//   kPress      the button went down (after debouncing)
//   kRelease    the button came up
//   kClick      a press and release without a long press. With multi-click
//               enabled it is reported once the click sequence ends, and
//               clicks() tells how many there were
//   kLongPress  held for `long_press_ms`; reported once, while still held.
//               The release that follows is not a click
//
// Without multi-click (multi_click_ms == 0) a click is reported on release.
// With it, clicks closer together than multi_click_ms are counted as one
// sequence and kClick arrives multi_click_ms after the last release.
// A press that becomes a long press cancels any clicks pending before it.
//
// `active_low` is for the usual wiring with a pull-up (pressed reads 0).
// long_press_ms == 0 disables long press.
// ---------------------------------------------------------------------------
class Button {
 public:
  enum Event : uint8_t { kPress = 1, kRelease = 2, kClick = 4, kLongPress = 8 };

  explicit Button(bool active_low = true, uint32_t debounce_ms = 20, uint32_t long_press_ms = 800,
                  uint32_t multi_click_ms = 0)
      : deb_(debounce_ms, debounce_ms, false),
        active_low_(active_low),
        long_ms_(long_press_ms),
        multi_ms_(multi_click_ms),
        press_at_(0),
        release_at_(0),
        clicks_(0),
        pending_clicks_(0),
        long_fired_(false) {}

  // Feed the raw pin level read at time `now`. Returns a mask of Event bits.
  uint8_t update(bool raw_level, uint32_t now) {
    uint8_t ev = 0;
    const bool pressed = active_low_ ? !raw_level : raw_level;
    deb_.update(pressed, now);

    if (deb_.rose()) {
      ev |= kPress;
      press_at_ = now;
      long_fired_ = false;
    } else if (deb_.fell()) {
      ev |= kRelease;
      if (long_fired_) {
        pending_clicks_ = 0;
      } else if (multi_ms_ == 0) {
        ev |= kClick;
        clicks_ = 1;
        pending_clicks_ = 0;
      } else {
        if (pending_clicks_ != 255) ++pending_clicks_;
        release_at_ = now;
      }
    }

    if (deb_.state()) {
      if (!long_fired_ && long_ms_ != 0 && elapsed_ms(now, press_at_) >= long_ms_) {
        ev |= kLongPress;
        long_fired_ = true;
        pending_clicks_ = 0;
      }
    } else if (pending_clicks_ != 0 && elapsed_ms(now, release_at_) >= multi_ms_) {
      ev |= kClick;
      clicks_ = pending_clicks_;
      pending_clicks_ = 0;
    }
    return ev;
  }

  bool down() const { return deb_.state(); }                   // debounced: is it pressed
  uint32_t held_ms(uint32_t now) const {                      // how long it has been down (0 if up)
    return deb_.state() ? elapsed_ms(now, press_at_) : 0;
  }
  uint8_t clicks() const { return clicks_; }                  // click count of the last kClick
  bool long_pressed() const { return deb_.state() && long_fired_; }

  void reset() {
    deb_.reset(false);
    clicks_ = 0;
    pending_clicks_ = 0;
    long_fired_ = false;
  }

 private:
  Debouncer deb_;
  bool active_low_;
  uint32_t long_ms_;
  uint32_t multi_ms_;
  uint32_t press_at_;
  uint32_t release_at_;
  uint8_t clicks_;
  uint8_t pending_clicks_;
  bool long_fired_;
};

}  // namespace mcu

#endif  // MCU_DEBOUNCE_HPP
