// scheduler.hpp - a cooperative, non-blocking task scheduler, microcontroller agnostic.
//
// Run many small periodic and one-shot jobs from one main loop, with no RTOS,
// no interrupts and no heap. Time is passed in, so any clock works and tests
// can drive it by hand.
//
//   mcu::Scheduler<8> sched;                       // room for 8 tasks
//
//   static void blink(void* ctx, uint32_t now) { toggle_led(); }
//   static void sample(void* ctx, uint32_t now) { read_sensor(); }
//
//   sched.add_periodic(clock.millis(), 500, blink);        // every 500 ms
//   sched.add_periodic(clock.millis(), 20, sample, &state); // every 20 ms, with context
//   sched.add_once(clock.millis(), 3000, start_radio);     // once, in 3 s
//
//   for (;;) {
//     sched.run(clock.millis());                   // runs whatever is due
//     // optionally sleep until sched.time_until_next(clock.millis())
//   }
//
// Rules of the road:
//   * Cooperative: a callback runs to completion. Keep them short; a slow one
//     delays everything after it. Nothing here interrupts a callback.
//   * Each run() call runs every due task at most once, in slot order (the
//     order tasks were added, reusing freed slots). So a task with period 0
//     ("every pass") cannot starve the others.
//   * Callbacks may add, cancel, pause and reconfigure tasks, including their
//     own. A task added from inside a callback is first considered on the
//     next run() call, never in the pass that created it.
//   * Not interrupt safe. Call it only from the main loop; from an ISR, set a
//     flag and let a task act on it.
//   * Time is the usual wrapping 32-bit millisecond counter: all comparisons
//     are wrap-safe as long as run() is called at least every ~24 days.
//
// Periodic modes:
//   Rate::kFixedRate  (default) runs stay on the original time grid. If a run
//                     is late, the next run is still at the next grid point,
//                     and any grid points that were missed are skipped (counted
//                     in skipped()) rather than replayed in a burst.
//   Rate::kFixedDelay the next run is `period` after this one actually ran, so
//                     the spacing is never shorter than `period`, and the
//                     grid drifts when callbacks run late.
//
// Header-only, C++11, no heap, no exceptions, no RTTI. Needs hal/clock.hpp.

#ifndef MCU_SCHEDULER_HPP
#define MCU_SCHEDULER_HPP

#include <stddef.h>
#include <stdint.h>

#include "hal/clock.hpp"

namespace mcu {

// A task callback. `ctx` is the pointer given when the task was added; `now`
// is the time passed to run().
typedef void (*TaskFn)(void* ctx, uint32_t now);

// Identifies a task. Stale handles (of cancelled or finished tasks) are
// detected and rejected, even after the slot is reused.
typedef uint16_t TaskId;
constexpr TaskId kNoTask = 0xFFFF;  // returned when the scheduler is full

// For add_periodic(): wait one full period before the first run.
constexpr uint32_t kFirstRunAfterPeriod = 0xFFFFFFFFu;

enum class Rate : uint8_t { kFixedRate, kFixedDelay };

// Adapts a member function `void T::Method(uint32_t now)` to a TaskFn:
//   sched.add_periodic(now, 100, &mcu::member_task<Sensor, &Sensor::poll>, &sensor);
template <typename T, void (T::*Method)(uint32_t)>
void member_task(void* ctx, uint32_t now) {
  (static_cast<T*>(ctx)->*Method)(now);
}

template <size_t N>
class Scheduler {
  static_assert(N >= 1 && N <= 255, "Scheduler holds between 1 and 255 tasks");

 public:
  Scheduler() : in_run_(false) {}

  // ---- adding tasks ----

  // Runs every `period_ms`. The first run is `first_delay_ms` after `now`, by
  // default one period later (pass 0 to run on the next run() call).
  // A period of 0 means "on every run() call". Returns kNoTask if full.
  TaskId add_periodic(uint32_t now, uint32_t period_ms, TaskFn fn, void* ctx = nullptr,
                      Rate rate = Rate::kFixedRate,
                      uint32_t first_delay_ms = kFirstRunAfterPeriod) {
    const uint32_t delay = (first_delay_ms == kFirstRunAfterPeriod) ? period_ms : first_delay_ms;
    return allocate(now, delay, period_ms, true, rate, fn, ctx);
  }

  // Runs once, `delay_ms` after `now`, then frees its slot.
  TaskId add_once(uint32_t now, uint32_t delay_ms, TaskFn fn, void* ctx = nullptr) {
    return allocate(now, delay_ms, 0, false, Rate::kFixedRate, fn, ctx);
  }

  // ---- managing tasks (all return false for a stale or invalid id) ----

  bool cancel(TaskId id) {
    Slot* s = lookup(id);
    if (s == nullptr) return false;
    s->used = false;
    return true;
  }

  bool active(TaskId id) const { return lookup(id) != nullptr; }

  // A paused task keeps its slot but does not run. It remembers how long was
  // left, and resume() continues from there.
  bool pause(TaskId id, uint32_t now) {
    Slot* s = lookup(id);
    if (s == nullptr) return false;
    if (!s->paused) {
      s->deadline = time_reached(now, s->deadline) ? 0 : (uint32_t)(s->deadline - now);
      s->paused = true;  // while paused, `deadline` holds the time remaining
    }
    return true;
  }

  bool resume(TaskId id, uint32_t now) {
    Slot* s = lookup(id);
    if (s == nullptr) return false;
    if (s->paused) {
      s->deadline = (uint32_t)(now + s->deadline);
      s->paused = false;
    }
    return true;
  }

  bool paused(TaskId id) const {
    const Slot* s = lookup(id);
    return s != nullptr && s->paused;
  }

  // Changes the period of a periodic task. It applies from the next time the
  // task is rescheduled, so the run already pending keeps its time.
  bool set_period(TaskId id, uint32_t period_ms) {
    Slot* s = lookup(id);
    if (s == nullptr || !s->periodic) return false;
    s->period = period_ms;
    return true;
  }

  // Makes the next run happen `delay_ms` after `now`, and unpauses the task.
  bool restart(TaskId id, uint32_t now, uint32_t delay_ms) {
    Slot* s = lookup(id);
    if (s == nullptr) return false;
    s->deadline = (uint32_t)(now + delay_ms);
    s->paused = false;
    return true;
  }

  // How many scheduled periods were skipped because the task ran too late.
  uint32_t skipped(TaskId id) const {
    const Slot* s = lookup(id);
    return s != nullptr ? s->skipped : 0;
  }

  void clear() {
    for (size_t i = 0; i < N; ++i) slots_[i].used = false;
  }

  // ---- running ----

  // Runs every task that is due, each at most once. Returns how many ran.
  // Calling run() from inside a callback does nothing and returns 0.
  size_t run(uint32_t now) {
    if (in_run_) return 0;
    in_run_ = true;
    size_t ran = 0;
    for (size_t i = 0; i < N; ++i) {
      Slot& s = slots_[i];
      if (!s.used || s.paused || s.deferred || !time_reached(now, s.deadline)) continue;
      const TaskFn fn = s.fn;
      void* const ctx = s.ctx;
      // Reschedule (or free) BEFORE calling, so the callback may cancel or
      // reconfigure its own task and that has the last word.
      if (s.periodic) {
        reschedule(s, now);
      } else {
        s.used = false;
      }
      fn(ctx, now);
      ++ran;
    }
    for (size_t i = 0; i < N; ++i) slots_[i].deferred = false;
    in_run_ = false;
    return ran;
  }

  size_t run(const Clock& clock) { return run(clock.millis()); }

  // Milliseconds until the next task is due: 0 if one is due now, kNever if
  // nothing is scheduled (paused tasks do not count). Use it to sleep.
  uint32_t time_until_next(uint32_t now) const {
    uint32_t best = kNever;
    for (size_t i = 0; i < N; ++i) {
      const Slot& s = slots_[i];
      if (!s.used || s.paused) continue;
      if (time_reached(now, s.deadline)) return 0;
      const uint32_t d = (uint32_t)(s.deadline - now);
      if (d < best) best = d;
    }
    return best;
  }

  size_t count() const {
    size_t n = 0;
    for (size_t i = 0; i < N; ++i)
      if (slots_[i].used) ++n;
    return n;
  }

  static constexpr size_t capacity() { return N; }

 private:
  struct Slot {
    TaskFn fn = nullptr;
    void* ctx = nullptr;
    uint32_t deadline = 0;  // when due; the time remaining while paused
    uint32_t period = 0;
    uint32_t skipped = 0;
    uint8_t gen = 0;
    bool used = false;
    bool periodic = false;
    bool fixed_delay = false;
    bool paused = false;
    bool deferred = false;  // added during the current run() pass
  };

  static TaskId make_id(size_t index, uint8_t gen) {
    return (TaskId)(((unsigned)gen << 8) | (unsigned)index);
  }

  Slot* lookup(TaskId id) {
    const size_t index = (size_t)(id & 0xFF);
    if (index >= N) return nullptr;
    Slot& s = slots_[index];
    return (s.used && s.gen == (uint8_t)(id >> 8)) ? &s : nullptr;
  }
  const Slot* lookup(TaskId id) const {
    const size_t index = (size_t)(id & 0xFF);
    if (index >= N) return nullptr;
    const Slot& s = slots_[index];
    return (s.used && s.gen == (uint8_t)(id >> 8)) ? &s : nullptr;
  }

  TaskId allocate(uint32_t now, uint32_t delay, uint32_t period, bool periodic, Rate rate,
                  TaskFn fn, void* ctx) {
    if (fn == nullptr) return kNoTask;
    for (size_t i = 0; i < N; ++i) {
      Slot& s = slots_[i];
      if (s.used) continue;
      s.gen = (uint8_t)(s.gen + 1);
      s.fn = fn;
      s.ctx = ctx;
      s.deadline = (uint32_t)(now + delay);
      s.period = period;
      s.skipped = 0;
      s.used = true;
      s.periodic = periodic;
      s.fixed_delay = (rate == Rate::kFixedDelay);
      s.paused = false;
      s.deferred = in_run_;
      return make_id(i, s.gen);
    }
    return kNoTask;
  }

  // Sets the next deadline of a periodic task that is due at `now`.
  static void reschedule(Slot& s, uint32_t now) {
    if (s.fixed_delay || s.period == 0) {
      s.deadline = (uint32_t)(now + s.period);
      return;
    }
    // Stay on the original grid, skipping grid points that were missed.
    const uint32_t late = (uint32_t)(now - s.deadline);
    const uint32_t missed = late / s.period;
    s.deadline = (uint32_t)(s.deadline + (missed + 1u) * s.period);
    s.skipped += missed;
  }

  Slot slots_[N];
  bool in_run_;
};

}  // namespace mcu

#endif  // MCU_SCHEDULER_HPP
