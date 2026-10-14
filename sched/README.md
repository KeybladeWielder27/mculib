# sched

Non-blocking timing for firmware: a cooperative task scheduler, plus small helpers for stopwatches,
rate limiting and retry backoff. No RTOS, no interrupts, no heap. Time is passed in, so any clock works
and tests can drive it by hand.

Header-only, C++11, no exceptions, no RTTI. Needs `hal/clock.hpp`.

| File | What it is |
|---|---|
| `scheduler.hpp` | `Scheduler<N>`: periodic and one-shot tasks in a fixed number of slots |
| `timers.hpp` | `Stopwatch`, `Throttle`, `Backoff`, `RetryTimer`, `jitter_down()` |

For a single periodic or one-shot timer, `Timeout` in `hal/clock.hpp` is lighter. Reach for `Scheduler`
when you have several jobs.

## Scheduler

```cpp
#include "sched/scheduler.hpp"

static void blink(void* ctx, uint32_t now)  { toggle_led(); }
static void sample(void* ctx, uint32_t now) { static_cast<Sensor*>(ctx)->read(); }

mcu::Scheduler<8> sched;                                  // room for 8 tasks

sched.add_periodic(clock.millis(), 500, blink);           // every 500 ms
sched.add_periodic(clock.millis(), 20, sample, &sensor);  // every 20 ms, with a context pointer
sched.add_once(clock.millis(), 3000, start_radio);        // once, in 3 seconds

for (;;) {
  sched.run(clock.millis());                              // runs whatever is due
}
```

Member functions work too:

```cpp
sched.add_periodic(now, 100, &mcu::member_task<Sensor, &Sensor::poll>, &sensor);  // void Sensor::poll(uint32_t now)
```

### API

| Call | Notes |
|---|---|
| `add_periodic(now, period_ms, fn, ctx, rate, first_delay_ms)` | first run one period later by default; `first_delay_ms = 0` runs on the next pass. A period of 0 means every pass |
| `add_once(now, delay_ms, fn, ctx)` | runs once, then frees its slot |
| `cancel(id)`, `active(id)` | |
| `pause(id, now)`, `resume(id, now)`, `paused(id)` | a paused task remembers how long was left |
| `set_period(id, ms)` | applies from the next reschedule |
| `restart(id, now, delay_ms)` | next run `delay_ms` from now; also unpauses |
| `run(now)` or `run(clock)` | returns how many tasks ran |
| `time_until_next(now)` | ms until the next task is due, 0 if one is due now, `kNever` if none. Use it to sleep |
| `skipped(id)`, `count()`, `capacity()`, `clear()` | |

`add_*` returns a `TaskId`, or `kNoTask` if the scheduler is full. Handles are checked: a cancelled or finished
task's id is rejected even after its slot is reused, so a stale `cancel(id)` can never hit someone else's task.

### Periodic modes

| `Rate::kFixedRate` (default) | `Rate::kFixedDelay` |
|---|---|
| Runs stay on the original time grid. If a run is late, the next one is still at the next grid point | The next run is `period` after this one actually ran |
| Missed grid points are skipped, not replayed in a burst, and counted in `skipped(id)` | Spacing is never shorter than `period`, and the schedule drifts when callbacks run late |
| Good for sampling and clocks | Good for "wait this long after finishing" jobs |

### Rules

- **Cooperative.** A callback runs to completion and delays everything after it, so keep them short.
- Each `run()` runs every due task **at most once**, in the order tasks were added (freed slots are reused). A
  period-0 task therefore cannot starve the others.
- Callbacks may add, cancel, pause and reconfigure tasks, including their own. A task added from inside a callback
  is first considered on the **next** `run()`, never in the pass that created it. Calling `run()` from a callback
  does nothing.
- **Not interrupt safe.** Use it from the main loop only. From an ISR, set a flag and let a task act on it.
- Time is the wrapping 32-bit millisecond counter. Everything is wrap-safe as long as `run()` is called at least
  every ~24 days.

### Sleeping between tasks

```cpp
uint32_t wait = sched.time_until_next(clock.millis());
if (wait != mcu::kNever && wait > 0) sleep_ms(wait < 1000 ? wait : 1000);   // your platform's low-power wait
```

## Timer helpers

All take the current time as a parameter, never block, and are wrap-safe.

```cpp
mcu::Stopwatch sw;                       // how long did that take?
sw.start(now);  ...  uint32_t ms = sw.stop(now);     // also: elapsed(now), lap(now)

mcu::Throttle log_limit(1000);           // at most once a second; the first request goes through
if (log_limit.allow(now)) print_status();

mcu::Backoff b(100, 5000);               // 100, 200, 400, ... 3200, 5000, 5000   (factor 2 by default)
uint32_t delay = b.next();
```

### RetryTimer

Backoff plus a deadline: tells you when to try again.

```cpp
mcu::RetryTimer link(500, 30000);        // 0.5 s, doubling, up to 30 s. Pass a 4th argument to give up after N failures
link.start(now);                         // the first attempt is due at once

if (link.due(now)) {
  if (try_connect()) link.succeeded();   // stops and resets the backoff
  else               link.failed(now);   // schedules the next attempt
}
if (link.gave_up()) { /* report */ }
```

To stop many devices retrying at the same instant, call `link.set_jitter_percent(25)` and pass a random number
to `failed(now, rnd)`. Jitter only ever shortens the wait. `jitter_down(delay, rnd, percent)` is also available on
its own.

## Test

From this folder:

```
g++ -std=c++11 -Wall -Wextra -Werror -I.. test_sched.cpp -o test_sched && ./test_sched
```

The tests cover fixed-rate and fixed-delay behavior including late runs, counter wrap-around, period-0 tasks,
capacity and stale handles, pause/resume and reconfiguration, callbacks that cancel, add and re-enter, a 50,000-step
randomized run checked against the live-handle count, and every timer helper including wrap-around.
