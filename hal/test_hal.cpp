// Host-side tests for the hal library. Build and run on a PC from this folder:
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_hal.cpp -o test_hal && ./test_hal
// (-I.. puts the root of the collection on the include path.)

#include <stdio.h>
#include <string.h>

#include "hal/baud.hpp"
#include "hal/clock.hpp"
#include "hal/loopback_stream.hpp"
#include "hal/ports/arduino_stream.hpp"
#include "hal/stream.hpp"

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

// ---------------------------------------------------------------------------
// Clock
// ---------------------------------------------------------------------------
static uint32_t g_fake_ms = 0;
static uint32_t fake_millis() { return g_fake_ms; }

static void test_time_helpers() {
  CHECK(mcu::elapsed_ms(150, 100) == 50);
  CHECK(mcu::elapsed_ms(5, 0xFFFFFFF0u) == 21);  // across wrap
  CHECK(mcu::elapsed_ms(7, 7) == 0);

  CHECK(mcu::time_reached(100, 100));
  CHECK(mcu::time_reached(101, 100));
  CHECK(!mcu::time_reached(99, 100));
  CHECK(mcu::time_reached(5, 0xFFFFFFF0u));    // deadline just before wrap, now after
  CHECK(!mcu::time_reached(0xFFFFFFE0u, 5));   // now just before wrap, deadline after
  CHECK(mcu::time_reached(0x7FFFFFFFu, 0));    // 24.8 days later is still "reached"
  CHECK(!mcu::time_reached(0, 0x7FFFFFFFu));
}

static void test_clocks() {
  mcu::ManualClock m;
  CHECK(m.millis() == 0);
  m.advance(10);
  CHECK(m.millis() == 10);
  m.set(0xFFFFFFFFu);
  m.advance(2);
  CHECK(m.millis() == 1);  // wraps

  g_fake_ms = 1234;
  mcu::FunctionClock f(fake_millis);
  CHECK(f.millis() == 1234);
  g_fake_ms = 99;
  const mcu::Clock& c = f;  // usable through the interface
  CHECK(c.millis() == 99);
}

static void test_timeout_oneshot() {
  mcu::Timeout t;
  CHECK(!t.active());
  CHECK(!t.expired(0));
  CHECK(!t.expired(123456));  // a stopped timer never expires
  CHECK(t.remaining(0) == 0);

  t.start(100, 50);
  CHECK(t.active());
  CHECK(!t.expired(100));
  CHECK(!t.expired(149));
  CHECK(t.expired(150));
  CHECK(t.expired(5000));
  CHECK(t.remaining(100) == 50);
  CHECK(t.remaining(130) == 20);
  CHECK(t.remaining(150) == 0);

  t.restart(200);
  CHECK(!t.expired(249));
  CHECK(t.expired(250));

  t.stop();
  CHECK(!t.active());
  CHECK(!t.expired(100000));
}

static void test_timeout_wrap() {
  mcu::Timeout t;
  t.start(0xFFFFFFF0u, 32);  // deadline lands at 0x10 after the counter wraps
  CHECK(!t.expired(0xFFFFFFF0u));
  CHECK(!t.expired(0xFFFFFFFFu));
  CHECK(!t.expired(0x0000000Fu));
  CHECK(t.expired(0x00000010u));
  CHECK(t.expired(0x00000011u));
  CHECK(t.remaining(0xFFFFFFF8u) == 24);
}

static void test_timeout_periodic() {
  mcu::Timeout t;
  t.start(0, 100);
  CHECK(!t.poll_periodic(99));
  CHECK(t.poll_periodic(100));
  CHECK(!t.poll_periodic(150));
  CHECK(t.poll_periodic(200));
  // Fires late: the schedule stays anchored to the original start.
  CHECK(t.poll_periodic(301));
  CHECK(!t.poll_periodic(399));
  CHECK(t.poll_periodic(400));

  // More than a whole period behind: fires once, then resynchronises
  // instead of firing a burst of catch-up events.
  CHECK(t.poll_periodic(1000));
  CHECK(!t.poll_periodic(1000));
  CHECK(!t.poll_periodic(1099));
  CHECK(t.poll_periodic(1100));

  // Steady 1 ms ticking: exactly one event per period, no drift.
  mcu::Timeout u;
  u.start(0, 100);
  int fired = 0;
  for (uint32_t now = 1; now <= 10000; ++now)
    if (u.poll_periodic(now)) ++fired;
  CHECK(fired == 100);

  // Same across the counter wrap.
  mcu::Timeout w;
  w.start(0xFFFFFF00u, 100);
  fired = 0;
  for (uint32_t i = 1; i <= 1000; ++i)
    if (w.poll_periodic((uint32_t)(0xFFFFFF00u + i))) ++fired;
  CHECK(fired == 10);

  // Zero period does not hang.
  mcu::Timeout z;
  z.start(0, 0);
  CHECK(z.poll_periodic(5));
}

// ---------------------------------------------------------------------------
// LoopbackStream
// ---------------------------------------------------------------------------
static void test_loopback() {
  mcu::LoopbackStream<8, 8> port;
  mcu::Stream& s = port;  // code under test only sees the interface

  CHECK(s.available() == 0);
  CHECK(s.write_space() == 8);
  uint8_t b = 0;
  CHECK(!s.read_byte(b));

  // wire -> device
  const uint8_t in[5] = {1, 2, 3, 4, 5};
  CHECK(port.inject(in, 5) == 5);
  CHECK(s.available() == 5);
  CHECK(port.rx_space() == 3);
  uint8_t got[8];
  CHECK(s.read(got, 3) == 3);
  CHECK(got[0] == 1 && got[1] == 2 && got[2] == 3);
  CHECK(s.read_byte(b) && b == 4);
  CHECK(s.available() == 1);

  // injecting more than fits delivers only what fits
  const uint8_t big[10] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
  CHECK(port.inject(big, 10) == 7);  // 1 already waiting, 7 more fit
  CHECK(s.available() == 8);
  CHECK(s.read(got, 100) == 8);
  CHECK(got[0] == 5 && got[1] == 10 && got[7] == 16);

  // device -> wire
  const uint8_t out[6] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
  CHECK(s.write(out, 6) == 6);
  CHECK(s.write_space() == 2);
  CHECK(port.tx_size() == 6);
  CHECK(s.write(out, 6) == 2);  // only 2 accepted, caller sees the shortfall
  CHECK(s.write_space() == 0);
  CHECK(!s.write_byte(0xFF));
  CHECK(port.take(got, 4) == 4);
  CHECK(got[0] == 0xA0 && got[3] == 0xA3);
  CHECK(s.write_space() == 4);
  CHECK(s.write_byte(0x77));
  CHECK(port.take(got, 100) == 5);
  CHECK(got[0] == 0xA4 && got[1] == 0xA5 && got[2] == 0xA0 && got[3] == 0xA1 && got[4] == 0x77);

  // Checking space before sending a whole frame avoids splitting it.
  uint8_t frame[6] = {1, 2, 3, 4, 5, 6};
  CHECK(s.write_space() >= sizeof frame);
  CHECK(s.write(frame, sizeof frame) == sizeof frame);
  CHECK(s.write_space() < sizeof frame);  // next frame must wait

  port.clear();
  CHECK(s.available() == 0 && port.tx_size() == 0 && s.write_space() == 8);
}

// ---------------------------------------------------------------------------
// ArduinoStream, against a fake with the same method shapes as Arduino's
// HardwareSerial. No Arduino headers involved.
// ---------------------------------------------------------------------------
struct FakeSerial {
  uint8_t rx[16];
  int rx_head, rx_tail;
  uint8_t tx[16];
  int tx_len;
  int tx_capacity;      // what availableForWrite() reports as free
  bool report_space;    // false mimics cores whose availableForWrite() returns 0

  FakeSerial() : rx_head(0), rx_tail(0), tx_len(0), tx_capacity(16), report_space(true) {}

  void feed(const uint8_t* p, int n) {
    for (int i = 0; i < n; ++i) rx[rx_tail++] = p[i];
  }
  int available() { return rx_tail - rx_head; }
  int read() { return rx_head < rx_tail ? rx[rx_head++] : -1; }
  size_t write(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; ++i) tx[tx_len++] = p[i];
    return n;
  }
  int availableForWrite() { return report_space ? tx_capacity - tx_len : 0; }
};

static void test_arduino_port() {
  FakeSerial fake;
  mcu::ArduinoStream<FakeSerial> port(fake);
  mcu::Stream& s = port;

  // receive
  const uint8_t in[4] = {9, 8, 7, 6};
  fake.feed(in, 4);
  CHECK(s.available() == 4);
  uint8_t got[8];
  CHECK(s.read(got, 3) == 3);
  CHECK(got[0] == 9 && got[1] == 8 && got[2] == 7);
  CHECK(s.read(got, 8) == 1);  // only one left, no blocking for more
  CHECK(got[0] == 6);
  CHECK(s.read(got, 8) == 0);
  CHECK(s.available() == 0);

  // transmit is limited to the free space so write() never blocks
  fake.tx_capacity = 5;
  CHECK(s.write_space() == 5);
  const uint8_t out[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  CHECK(s.write(out, 8) == 5);
  CHECK(fake.tx_len == 5);
  CHECK(fake.tx[4] == 5);
  CHECK(s.write_space() == 0);
  CHECK(s.write(out, 3) == 0);  // full: nothing is passed to the device
  CHECK(fake.tx_len == 5);

  // cores where availableForWrite() is useless: pass everything through
  FakeSerial blocking;
  blocking.report_space = false;
  mcu::ArduinoStream<FakeSerial> raw(blocking, false);
  CHECK(raw.write_space() == mcu::kUnboundedSpace);
  CHECK(raw.write(out, 8) == 8);
  CHECK(blocking.tx_len == 8);

  // with the limit on but a device that reports 0, nothing is sent
  FakeSerial stuck;
  stuck.report_space = false;
  mcu::ArduinoStream<FakeSerial> limited(stuck);
  CHECK(limited.write_space() == 0);
  CHECK(limited.write(out, 8) == 0);
  CHECK(stuck.tx_len == 0);
}

// ---------------------------------------------------------------------------
// BaudControl
// ---------------------------------------------------------------------------
static uint32_t g_fn_baud = 0;
static bool fn_baud(uint32_t b) {
  if (b > 230400) return false;
  g_fn_baud = b;
  return true;
}

// Only begin() and end(): proves ArduinoBaudControl needs nothing else, and
// that ArduinoStream does not need begin() (FakeSerial above has none).
struct FakeBaudSerial {
  unsigned long rate;
  int ends, begins;
  bool ended_before_begin;
  FakeBaudSerial() : rate(9600), ends(0), begins(0), ended_before_begin(true) {}
  void end() { ++ends; }
  void begin(unsigned long r) {
    if (ends <= begins) ended_before_begin = false;
    ++begins;
    rate = r;
  }
};

static void test_baud() {
  mcu::FunctionBaudControl f(fn_baud);
  mcu::BaudControl& bf = f;
  CHECK(bf.set_baud(115200));
  CHECK(g_fn_baud == 115200);
  CHECK(!bf.set_baud(921600));
  CHECK(g_fn_baud == 115200);
  mcu::FunctionBaudControl none(0);
  CHECK(!none.set_baud(9600));

  mcu::ManualBaudControl m(9600, 115200);
  CHECK(m.baud() == 9600 && m.changes() == 0);
  CHECK(m.set_baud(57600));
  CHECK(m.baud() == 57600 && m.changes() == 1);
  CHECK(!m.set_baud(230400));
  CHECK(!m.set_baud(0));
  CHECK(m.baud() == 57600 && m.changes() == 1);

  FakeBaudSerial fs;
  mcu::ArduinoBaudControl<FakeBaudSerial> a(fs);
  mcu::BaudControl& ba = a;
  CHECK(ba.set_baud(38400));
  CHECK(fs.rate == 38400 && fs.ends == 1 && fs.begins == 1 && fs.ended_before_begin);
  CHECK(!ba.set_baud(0));
  CHECK(fs.rate == 38400 && fs.begins == 1);
}

int main() {
  test_time_helpers();
  test_clocks();
  test_timeout_oneshot();
  test_timeout_wrap();
  test_timeout_periodic();
  test_loopback();
  test_arduino_port();
  test_baud();

  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
