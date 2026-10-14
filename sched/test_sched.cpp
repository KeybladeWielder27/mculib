// Host-side tests for scheduler.hpp and timers.hpp. Build and run from this folder:
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_sched.cpp -o test_sched && ./test_sched
// (-I.. puts the root of the collection on the include path.)

#include <stdio.h>
#include <string.h>

#include "hal/clock.hpp"
#include "sched/scheduler.hpp"
#include "sched/timers.hpp"

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                          \
  do {                                                       \
    ++g_checks;                                              \
    if (!(cond)) {                                           \
      ++g_failures;                                          \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                        \
  } while (0)

static uint32_t g_rng = 31337;
static uint32_t rnd32() {
  g_rng = g_rng * 1664525u + 1013904223u;
  return g_rng;
}

struct Counter {
  int n;
  uint32_t last_now;
};
static void count_fn(void* ctx, uint32_t now) {
  Counter* c = static_cast<Counter*>(ctx);
  ++c->n;
  c->last_now = now;
}

// ---------------------------------------------------------------------------
// Scheduler: periodic and one-shot
// ---------------------------------------------------------------------------
static void test_periodic_basic() {
  mcu::Scheduler<4> s;
  Counter c = {0, 0};
  mcu::TaskId id = s.add_periodic(0, 100, count_fn, &c);
  CHECK(id != mcu::kNoTask);
  CHECK(s.count() == 1);

  size_t total = 0;
  for (uint32_t t = 0; t <= 1000; ++t) {
    size_t r = s.run(t);
    total += r;
    if (r) {
      CHECK(t > 0 && t % 100 == 0);
      CHECK(c.last_now == t);
    }
  }
  CHECK(c.n == 10);
  CHECK(total == 10);
  CHECK(s.skipped(id) == 0);
}

static void test_first_delay() {
  mcu::Scheduler<4> s;
  Counter c = {0, 0};
  s.add_periodic(1000, 100, count_fn, &c, mcu::Rate::kFixedRate, 0);  // run at once
  CHECK(s.run(1000) == 1);
  CHECK(s.run(1099) == 0);
  CHECK(s.run(1100) == 1);

  mcu::Scheduler<4> s2;
  Counter d = {0, 0};
  s2.add_periodic(2000, 100, count_fn, &d, mcu::Rate::kFixedRate, 30);  // first run after 30 ms
  CHECK(s2.run(2029) == 0);
  CHECK(s2.run(2030) == 1 && d.n == 1);
  CHECK(s2.run(2129) == 0);
  CHECK(s2.run(2130) == 1 && d.n == 2);  // then every 100 from the first run
}

static void test_once() {
  mcu::Scheduler<4> s;
  Counter c = {0, 0};
  mcu::TaskId id = s.add_once(0, 250, count_fn, &c);
  CHECK(s.active(id));
  CHECK(s.run(249) == 0);
  CHECK(s.run(250) == 1);
  CHECK(c.n == 1);
  CHECK(s.count() == 0);       // slot freed
  CHECK(!s.active(id));
  CHECK(!s.cancel(id));        // a finished task's handle is dead
  CHECK(s.run(100000) == 0);
  CHECK(c.n == 1);
}

static void test_late_fixed_rate_keeps_grid() {
  mcu::Scheduler<4> s;
  Counter c = {0, 0};
  mcu::TaskId id = s.add_periodic(0, 100, count_fn, &c);
  CHECK(s.run(250) == 1);   // due at 100, runs at 250: one grid point (200) missed
  CHECK(c.n == 1);
  CHECK(s.skipped(id) == 1);
  CHECK(s.run(299) == 0);   // next is on the grid at 300, not 350
  CHECK(s.run(300) == 1);
  CHECK(s.skipped(id) == 1);

  // very late: many grid points skipped, still one run, still on the grid
  CHECK(s.run(1000) == 1);  // was due at 400; skips 400..900 minus the run itself
  CHECK(s.skipped(id) == 1 + 6);
  CHECK(s.run(1099) == 0);
  CHECK(s.run(1100) == 1);
  CHECK(c.n == 4);
}

static void test_fixed_delay() {
  mcu::Scheduler<4> s;
  Counter c = {0, 0};
  mcu::TaskId id = s.add_periodic(0, 100, count_fn, &c, mcu::Rate::kFixedDelay);
  CHECK(s.run(250) == 1);   // due at 100, ran at 250
  CHECK(s.run(349) == 0);   // next is 100 after the actual run
  CHECK(s.run(350) == 1);
  CHECK(s.skipped(id) == 0);
}

static void test_wraparound() {
  mcu::Scheduler<4> s;
  Counter c = {0, 0};
  const uint32_t start = 0xFFFFFF00u;  // 256 ms before the counter wraps
  s.add_periodic(start, 100, count_fn, &c);
  for (uint32_t i = 0; i <= 1000; ++i) s.run((uint32_t)(start + i));
  CHECK(c.n == 10);

  // one-shot across the wrap
  Counter d = {0, 0};
  s.add_once(0xFFFFFFF0u, 32, count_fn, &d);
  CHECK(s.run(0xFFFFFFFFu) == 0);
  CHECK(s.run(0x0000000Fu) == 0);
  CHECK(s.run(0x00000010u) == 1);
}

static void test_period_zero_runs_every_pass() {
  mcu::Scheduler<4> s;
  Counter c = {0, 0};
  s.add_periodic(0, 0, count_fn, &c, mcu::Rate::kFixedRate, 0);
  for (int i = 1; i <= 5; ++i) {
    CHECK(s.run(7) == 1);  // exactly once per call, even though always due
    CHECK(c.n == i);
  }
}

static void test_several_tasks() {
  mcu::Scheduler<4> s;
  Counter a = {0, 0}, b = {0, 0}, once = {0, 0};
  s.add_periodic(0, 100, count_fn, &a);
  s.add_periodic(0, 250, count_fn, &b);
  s.add_once(0, 500, count_fn, &once);
  CHECK(s.count() == 3);
  for (uint32_t t = 0; t <= 1000; ++t) s.run(t);
  CHECK(a.n == 10);
  CHECK(b.n == 4);
  CHECK(once.n == 1);
  CHECK(s.count() == 2);

  // several due at once run in one pass, in the order they were added
  mcu::Scheduler<4> s2;
  struct Order { char seq[8]; int n; } order = {{0}, 0};
  struct Tag {
    Order* o;
    char c;
  };
  static const struct { void (*fn)(void*, uint32_t); } unused = {nullptr};
  (void)unused;
  Tag ta = {&order, 'a'}, tb = {&order, 'b'}, tc = {&order, 'c'};
  auto record = [](void* ctx, uint32_t) {
    Tag* t = static_cast<Tag*>(ctx);
    t->o->seq[t->o->n++] = t->c;
  };
  s2.add_once(0, 10, record, &ta);
  s2.add_once(0, 10, record, &tb);
  s2.add_once(0, 10, record, &tc);
  CHECK(s2.run(10) == 3);
  CHECK(order.n == 3 && order.seq[0] == 'a' && order.seq[1] == 'b' && order.seq[2] == 'c');
}

// ---------------------------------------------------------------------------
// Scheduler: handles, capacity, management
// ---------------------------------------------------------------------------
static void test_capacity_and_stale_handles() {
  mcu::Scheduler<3> s;
  Counter c = {0, 0};
  mcu::TaskId a = s.add_periodic(0, 10, count_fn, &c);
  mcu::TaskId b = s.add_periodic(0, 10, count_fn, &c);
  mcu::TaskId d = s.add_periodic(0, 10, count_fn, &c);
  CHECK(a != mcu::kNoTask && b != mcu::kNoTask && d != mcu::kNoTask);
  CHECK(s.add_periodic(0, 10, count_fn, &c) == mcu::kNoTask);  // full
  CHECK(s.add_once(0, 10, count_fn, &c) == mcu::kNoTask);
  CHECK(s.count() == 3 && s.capacity() == 3);

  CHECK(s.cancel(b));
  CHECK(s.count() == 2);
  mcu::TaskId e = s.add_periodic(0, 10, count_fn, &c);  // reuses b's slot
  CHECK(e != mcu::kNoTask && e != b);
  CHECK(!s.active(b));       // the old handle must not touch the new task
  CHECK(!s.cancel(b));
  CHECK(!s.pause(b, 0));
  CHECK(!s.set_period(b, 5));
  CHECK(s.active(e));
  CHECK(s.count() == 3);

  CHECK(!s.active(mcu::kNoTask));
  CHECK(!s.cancel(mcu::kNoTask));
  CHECK(!s.cancel(0x0063));  // slot index out of range

  CHECK(s.add_once(0, 10, nullptr) == mcu::kNoTask);  // null callback refused
  s.clear();
  CHECK(s.count() == 0);
  CHECK(!s.active(a) && !s.active(e));
}

static void test_pause_resume() {
  mcu::Scheduler<4> s;
  Counter c = {0, 0};
  mcu::TaskId id = s.add_periodic(0, 100, count_fn, &c);
  CHECK(s.pause(id, 40));  // 60 ms left
  CHECK(s.paused(id));
  CHECK(s.run(100) == 0);
  CHECK(s.run(100000) == 0);
  CHECK(s.time_until_next(100000) == mcu::kNever);  // paused tasks do not count
  CHECK(s.resume(id, 500));
  CHECK(!s.paused(id));
  CHECK(s.run(559) == 0);
  CHECK(s.run(560) == 1);   // 500 + the 60 ms that were left
  CHECK(c.n == 1);
  CHECK(s.run(660) == 1);   // and then every 100

  // pausing twice and resuming twice are harmless
  CHECK(s.pause(id, 700) && s.pause(id, 9999));
  CHECK(s.resume(id, 1000) && s.resume(id, 5000));

  // pausing an overdue task: it becomes due as soon as it resumes
  mcu::Scheduler<2> t;
  Counter d = {0, 0};
  mcu::TaskId tid = t.add_once(0, 10, count_fn, &d);
  CHECK(t.pause(tid, 50));
  CHECK(t.resume(tid, 80));
  CHECK(t.run(80) == 1);
}

static void test_set_period_and_restart() {
  mcu::Scheduler<4> s;
  Counter c = {0, 0};
  mcu::TaskId id = s.add_periodic(0, 100, count_fn, &c);
  CHECK(s.run(100) == 1);
  CHECK(s.set_period(id, 50));        // the pending run (at 200) keeps its time
  CHECK(s.run(199) == 0);
  CHECK(s.run(200) == 1);
  CHECK(s.run(249) == 0);
  CHECK(s.run(250) == 1);             // now every 50
  CHECK(s.restart(id, 1000, 7));      // next run 7 ms from now
  CHECK(s.run(1006) == 0);
  CHECK(s.run(1007) == 1);

  CHECK(s.cancel(id));                // leave only the one-shot below
  mcu::TaskId once = s.add_once(0, 10, count_fn, &c);
  CHECK(!s.set_period(once, 5));      // one-shots have no period
  CHECK(s.restart(once, 2000, 5));
  CHECK(s.run(2004) == 0);
  CHECK(s.run(2005) == 1);
}

static void test_time_until_next() {
  mcu::Scheduler<4> s;
  CHECK(s.time_until_next(0) == mcu::kNever);
  Counter c = {0, 0};
  s.add_periodic(0, 1000, count_fn, &c);
  CHECK(s.time_until_next(0) == 1000);
  CHECK(s.time_until_next(400) == 600);
  CHECK(s.time_until_next(1000) == 0);
  CHECK(s.time_until_next(5000) == 0);  // overdue
  s.add_once(0, 300, count_fn, &c);
  CHECK(s.time_until_next(0) == 300);
  CHECK(s.time_until_next(299) == 1);

  mcu::Scheduler<2> w;  // across the wrap
  w.add_once(0xFFFFFFF0u, 32, count_fn, &c);
  CHECK(w.time_until_next(0xFFFFFFF8u) == 24);
  CHECK(w.time_until_next(0x10u) == 0);
}

// ---------------------------------------------------------------------------
// Scheduler: callbacks that modify the scheduler
// ---------------------------------------------------------------------------
struct SelfCancel {
  mcu::Scheduler<4>* s;
  mcu::TaskId id;
  int n;
};
static void self_cancel_fn(void* ctx, uint32_t) {
  SelfCancel* p = static_cast<SelfCancel*>(ctx);
  ++p->n;
  p->s->cancel(p->id);
}

struct CancelOther {
  mcu::Scheduler<4>* s;
  mcu::TaskId victim;
};
static void cancel_other_fn(void* ctx, uint32_t) {
  CancelOther* p = static_cast<CancelOther*>(ctx);
  p->s->cancel(p->victim);
}

struct Spawner {
  mcu::Scheduler<4>* s;
  Counter* child;
  int spawned;
};
static void spawn_fn(void* ctx, uint32_t now) {
  Spawner* p = static_cast<Spawner*>(ctx);
  ++p->spawned;
  p->s->add_once(now, 0, count_fn, p->child);  // due immediately
}

static void test_callbacks_modify_scheduler() {
  {  // a periodic task cancels itself
    mcu::Scheduler<4> s;
    SelfCancel sc = {&s, mcu::kNoTask, 0};
    sc.id = s.add_periodic(0, 10, self_cancel_fn, &sc);
    for (uint32_t t = 0; t < 200; ++t) s.run(t);
    CHECK(sc.n == 1);
    CHECK(s.count() == 0);
  }
  {  // an earlier task cancels a later one that is due in the same pass
    mcu::Scheduler<4> s;
    Counter victim = {0, 0};
    CancelOther co = {&s, mcu::kNoTask};
    s.add_once(0, 10, cancel_other_fn, &co);           // slot 0
    co.victim = s.add_once(0, 10, count_fn, &victim);  // slot 1
    CHECK(s.run(10) == 1);
    CHECK(victim.n == 0);
  }
  {  // a task added from a callback waits for the next pass, even if due at once
    mcu::Scheduler<4> s;
    Counter child = {0, 0};
    Spawner sp = {&s, &child, 0};
    s.add_once(0, 10, spawn_fn, &sp);
    CHECK(s.run(10) == 1);   // only the spawner ran
    CHECK(child.n == 0);
    CHECK(s.count() == 1);
    CHECK(s.run(10) == 1);   // next pass: the child
    CHECK(child.n == 1);
  }
  {  // run() from inside a callback is ignored
    mcu::Scheduler<4> s;
    static mcu::Scheduler<4>* sp;
    sp = &s;
    static size_t inner;
    inner = 99;
    s.add_once(0, 0, [](void*, uint32_t now) { inner = sp->run(now); });
    CHECK(s.run(0) == 1);
    CHECK(inner == 0);
  }
  {  // a callback that reuses its own just-freed slot
    mcu::Scheduler<1> s;
    static mcu::Scheduler<1>* sp;
    sp = &s;
    static Counter again;
    again.n = 0;
    s.add_once(0, 0, [](void*, uint32_t now) { sp->add_once(now, 0, count_fn, &again); });
    CHECK(s.run(0) == 1);
    CHECK(again.n == 0);
    CHECK(s.run(0) == 1);
    CHECK(again.n == 1);
  }
}

// ---------------------------------------------------------------------------
// Scheduler: Clock overload and member functions
// ---------------------------------------------------------------------------
struct Blinker {
  int ticks;
  uint32_t last;
  void tick(uint32_t now) {
    ++ticks;
    last = now;
  }
};

static void test_clock_and_members() {
  mcu::Scheduler<4> s;
  mcu::ManualClock clk(0);
  Blinker b = {0, 0};
  s.add_periodic(clk.millis(), 100, &mcu::member_task<Blinker, &Blinker::tick>, &b);
  for (int i = 0; i < 1000; ++i) {
    clk.advance(1);
    s.run(clk);
  }
  CHECK(b.ticks == 10);
  CHECK(b.last == 1000);
}

// ---------------------------------------------------------------------------
// Scheduler: randomized consistency (run under a sanitizer)
// ---------------------------------------------------------------------------
static void test_fuzz() {
  mcu::Scheduler<8> s;
  static mcu::TaskId ids[60000];
  size_t nids = 0;
  Counter c = {0, 0};
  uint32_t now = 0xFFFFF000u;  // the counter wraps during the run
  for (int step = 0; step < 50000; ++step) {
    switch (rnd32() % 6) {
      case 0:
        if (nids < sizeof ids / sizeof ids[0])
          ids[nids++] = s.add_periodic(now, rnd32() % 200, count_fn, &c,
                                       (rnd32() & 1) ? mcu::Rate::kFixedRate : mcu::Rate::kFixedDelay,
                                       rnd32() % 50);
        break;
      case 1:
        if (nids < sizeof ids / sizeof ids[0]) ids[nids++] = s.add_once(now, rnd32() % 300, count_fn, &c);
        break;
      case 2:
        if (nids) s.cancel(ids[rnd32() % nids]);
        break;
      case 3:
        if (nids) {
          mcu::TaskId id = ids[rnd32() % nids];
          if (rnd32() & 1) s.pause(id, now); else s.resume(id, now);
        }
        break;
      default:
        now = (uint32_t)(now + rnd32() % 40);
        s.run(now);
        break;
    }
    // the number of live tasks must equal the number of live handles
    if ((step & 255) == 0) {
      size_t live = 0;
      for (size_t i = 0; i < nids; ++i)
        if (ids[i] != mcu::kNoTask && s.active(ids[i])) ++live;
      CHECK(live == s.count());
      CHECK(s.count() <= s.capacity());
    }
  }
  CHECK(c.n > 0);
}

// ---------------------------------------------------------------------------
// timers.hpp
// ---------------------------------------------------------------------------
static void test_stopwatch() {
  mcu::Stopwatch w;
  CHECK(!w.running());
  CHECK(w.elapsed(500) == 0);
  w.start(100);
  CHECK(w.running());
  CHECK(w.elapsed(100) == 0);
  CHECK(w.elapsed(350) == 250);
  CHECK(w.lap(350) == 250);
  CHECK(w.elapsed(400) == 50);
  CHECK(w.stop(450) == 100);
  CHECK(!w.running());
  CHECK(w.elapsed(99999) == 100);  // frozen
  CHECK(w.stop(99999) == 100);
  CHECK(w.lap(99999) == 100);
  w.reset();
  CHECK(w.elapsed(5) == 0);

  mcu::Stopwatch x;  // across the wrap
  x.start(0xFFFFFFF0u);
  CHECK(x.elapsed(0x10u) == 32);
}

static void test_throttle() {
  mcu::Throttle t(1000);
  CHECK(t.wait_ms(0) == 0);
  CHECK(t.allow(5000));          // the first request goes through
  CHECK(!t.allow(5001));
  CHECK(!t.allow(5999));
  CHECK(t.wait_ms(5400) == 600);
  CHECK(t.allow(6000));
  CHECK(!t.allow(6500));
  t.reset();
  CHECK(t.allow(6501));          // reset: allowed at once

  int granted = 0;
  mcu::Throttle u(100);
  for (uint32_t i = 0; i <= 1000; ++i)
    if (u.allow(i)) ++granted;
  CHECK(granted == 11);          // at 0, 100, ..., 1000

  mcu::Throttle w(100);          // across the wrap
  CHECK(w.allow(0xFFFFFFF0u));
  CHECK(!w.allow(0x00000010u));
  CHECK(w.allow(0x00000054u));           // next window ends at 0x54 + 100 = 0xB8
  w.set_interval(10);                    // applies from the next allowed action
  CHECK(!w.allow(0x000000B7u));
  CHECK(w.allow(0x000000B8u));
  CHECK(!w.allow(0x000000B8u + 9));
  CHECK(w.allow(0x000000B8u + 10));
}

static void test_jitter() {
  CHECK(mcu::jitter_down(1000, 0, 20) == 1000);        // rnd 0: no reduction
  CHECK(mcu::jitter_down(1000, 0xFFFF, 20) == 801);    // largest reduction stays below 20%
  CHECK(mcu::jitter_down(1000, 12345, 0) == 1000);     // 0 percent: unchanged
  CHECK(mcu::jitter_down(0, 777, 50) == 0);
  CHECK(mcu::jitter_down(1000, 0xFFFF, 200) == 1);     // percent is capped at 100
  CHECK(mcu::jitter_down(1000, 0x10000, 50) == 1000);  // only the low 16 bits count
  for (int i = 0; i < 5000; ++i) {
    uint32_t delay = rnd32();
    uint8_t pct = (uint8_t)(rnd32() % 101);
    uint32_t j = mcu::jitter_down(delay, rnd32(), pct);
    uint32_t span = (uint32_t)(((uint64_t)delay * pct) / 100);
    CHECK(j <= delay && delay - j <= span);
  }
}

static void test_backoff() {
  mcu::Backoff b(100, 1000);
  CHECK(b.peek() == 100);
  const uint32_t want[] = {100, 200, 400, 800, 1000, 1000, 1000};
  for (size_t i = 0; i < sizeof want / sizeof want[0]; ++i) CHECK(b.next() == want[i]);
  CHECK(b.count() == 7);
  b.reset();
  CHECK(b.count() == 0 && b.next() == 100);

  mcu::Backoff f(10, 5000, 3);  // factor 3
  CHECK(f.next() == 10 && f.next() == 30 && f.next() == 90);

  mcu::Backoff big(1, 0xFFFFFFFFu, 2);  // must not overflow
  uint32_t prev = 0;
  for (int i = 0; i < 100; ++i) {
    uint32_t d = big.next();
    CHECK(d >= prev);
    prev = d;
  }
  CHECK(prev == 0xFFFFFFFFu);

  mcu::Backoff odd(500, 100);  // max below initial: max is raised to initial
  CHECK(odd.next() == 500 && odd.next() == 500);
  mcu::Backoff one(10, 100, 1);  // a factor below 2 is treated as 2
  CHECK(one.next() == 10 && one.next() == 20);
}

static void test_retry_timer() {
  mcu::RetryTimer r(100, 400);
  CHECK(!r.active() && !r.due(0));
  CHECK(r.wait_ms(0) == mcu::kNever);

  r.start(1000);
  CHECK(r.active() && r.due(1000));       // the first attempt is due at once
  CHECK(r.wait_ms(1000) == 0);
  r.failed(1000);                          // next after 100
  CHECK(!r.due(1099) && r.due(1100));
  CHECK(r.wait_ms(1040) == 60);
  r.failed(1100);                          // then 200
  CHECK(!r.due(1299) && r.due(1300));
  r.failed(1300);                          // then 400
  CHECK(!r.due(1699) && r.due(1700));
  r.failed(1700);                          // capped at 400
  CHECK(!r.due(2099) && r.due(2100));
  CHECK(r.failures() == 4);

  r.succeeded();
  CHECK(!r.active() && !r.due(99999) && r.failures() == 0);
  r.start(5000);                           // the backoff starts over
  r.failed(5000);
  CHECK(!r.due(5099) && r.due(5100));

  // with a limit
  mcu::RetryTimer lim(10, 100, 2, 3);
  lim.start(0);
  CHECK(lim.due(0));
  lim.failed(0);
  CHECK(lim.due(10));
  lim.failed(10);
  CHECK(!lim.gave_up());
  CHECK(lim.due(30));
  lim.failed(30);                          // the third failure: give up
  CHECK(lim.gave_up() && !lim.active() && !lim.due(99999));
  CHECK(lim.wait_ms(99999) == mcu::kNever);
  lim.failed(40);                          // ignored once given up
  CHECK(lim.failures() == 3);
  lim.start(1000);                         // can be restarted
  CHECK(!lim.gave_up() && lim.due(1000));

  // jitter only ever shortens the wait
  mcu::RetryTimer j(1000, 1000);
  j.set_jitter_percent(50);
  j.start(0);
  j.failed(0, 0xFFFF);
  CHECK(j.wait_ms(0) >= 500 && j.wait_ms(0) <= 1000);
  j.set_jitter_percent(250);               // clamped to 100
  j.failed(0, 0xFFFF);
  CHECK(j.wait_ms(0) >= 1 && j.wait_ms(0) <= 1000);

  // across the wrap
  mcu::RetryTimer w(100, 100);
  w.start(0xFFFFFFC0u);
  w.failed(0xFFFFFFC0u);
  CHECK(!w.due(0x00000023u) && w.due(0x00000024u));  // 0xFFFFFFC0 + 100 wraps to 0x24
}

int main() {
  test_periodic_basic();
  test_first_delay();
  test_once();
  test_late_fixed_rate_keeps_grid();
  test_fixed_delay();
  test_wraparound();
  test_period_zero_runs_every_pass();
  test_several_tasks();
  test_capacity_and_stale_handles();
  test_pause_resume();
  test_set_period_and_restart();
  test_time_until_next();
  test_callbacks_modify_scheduler();
  test_clock_and_members();
  test_fuzz();
  test_stopwatch();
  test_throttle();
  test_jitter();
  test_backoff();
  test_retry_timer();

  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
