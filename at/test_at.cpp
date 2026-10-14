// Host-side tests for at/: AtClient against a scripted fake device, plus at_text.hpp.
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_at.cpp -o test_at && ./test_at

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "at/at_client.hpp"
#include "at/at_text.hpp"
#include "hal/loopback_stream.hpp"

static unsigned long g_checks = 0;
static unsigned long g_fails = 0;

#define CHECK(...)                                                                    \
  do {                                                                                \
    ++g_checks;                                                                       \
    if (!(__VA_ARGS__)) {                                                             \
      ++g_fails;                                                                      \
      if (g_fails < 40) printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__); \
    }                                                                                 \
  } while (0)

static uint64_t g_rng = 0x243F6A8885A308D3ull;
static uint32_t rnd32() {
  g_rng ^= g_rng >> 12;
  g_rng ^= g_rng << 25;
  g_rng ^= g_rng >> 27;
  return (uint32_t)((g_rng * 0x2545F4914F6CDD1Dull) >> 32);
}
static uint32_t rnd_below(uint32_t n) { return rnd32() % n; }

using mcu::AtExpect;
using mcu::AtStatus;

// ---------------------------------------------------------------------------
// Simulated wire + device. Each tick (1 ms) the device takes up to
// `take_per_ms` bytes the client sent and injects up to `inject_per_ms` bytes
// of scheduled replies that are due.
// ---------------------------------------------------------------------------
typedef mcu::LoopbackStream<256, 256> Port;
typedef mcu::LoopbackStream<256, 8> SmallTxPort;  // 8-byte transmit buffer

template <typename P>
struct Sim {
  struct Pending {
    uint32_t at;
    std::string bytes;
  };

  P port;
  uint32_t now;
  size_t take_per_ms;
  size_t inject_per_ms;
  std::string wire;                // everything the client sent
  std::vector<uint32_t> wire_time; // when each byte was taken
  std::vector<Pending> pending;

  // Auto-responder: called for every complete CR-terminated line the device
  // receives; returns the reply (may be empty) to send after `reply_delay` ms.
  std::string (*responder)(const std::string& line);
  uint32_t reply_delay;
  std::string partial;

  explicit Sim(uint32_t start = 1000)
      : now(start), take_per_ms(256), inject_per_ms(256), responder(0), reply_delay(5) {}

  void reply(uint32_t after_ms, const std::string& bytes) {
    Pending p = {(uint32_t)(now + after_ms), bytes};
    pending.push_back(p);
  }

  void tick() {
    uint8_t buf[256];
    const size_t n = port.take(buf, take_per_ms < sizeof buf ? take_per_ms : sizeof buf);
    for (size_t i = 0; i < n; ++i) {
      wire.push_back((char)buf[i]);
      wire_time.push_back(now);
      if (responder != 0) {
        if (buf[i] == '\r') {
          const std::string r = responder(partial);
          partial.clear();
          if (!r.empty()) reply(reply_delay, r);
        } else {
          partial.push_back((char)buf[i]);
        }
      }
    }
    size_t budget = inject_per_ms;
    while (budget > 0 && !pending.empty() && mcu::time_reached(now, pending[0].at)) {
      std::string& b = pending[0].bytes;
      size_t k = b.size() < budget ? b.size() : budget;
      k = port.inject(reinterpret_cast<const uint8_t*>(b.data()), k);
      if (k == 0) break;
      b.erase(0, k);
      budget -= k;
      if (b.empty()) pending.erase(pending.begin());
    }
  }

  // Polls the client once per ms until it is no longer busy (or max_ms).
  template <typename C>
  AtStatus run(C& at, uint32_t max_ms = 100000) {
    AtStatus st = at.poll(now);
    for (uint32_t i = 0; i < max_ms && st == AtStatus::kBusy; ++i) {
      now = (uint32_t)(now + 1);
      tick();
      st = at.poll(now);
    }
    return st;
  }

  // Advances time with the client polled every ms, whatever its state.
  template <typename C>
  void advance(C& at, uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
      now = (uint32_t)(now + 1);
      tick();
      at.poll(now);
    }
  }
};

// Collects lines from handlers.
struct Lines {
  std::vector<std::string> v;
  static void on(void* ctx, const char* line) { static_cast<Lines*>(ctx)->v.push_back(line); }
};

// ---------------------------------------------------------------------------
// at_text.hpp
// ---------------------------------------------------------------------------
static void test_text_builder() {
  mcu::TextBuilder<16> t;
  CHECK(t.size() == 0 && strcmp(t.c_str(), "") == 0 && t.ok());
  t.add("ATID").add_hex(0x7FFF);
  CHECK(strcmp(t.c_str(), "ATID7FFF") == 0 && t.ok());
  t.clear();
  t.add("ATCH").add_hex(0xC, 2);
  CHECK(strcmp(t.c_str(), "ATCH0C") == 0);
  t.clear();
  t.add_hex(0);
  CHECK(strcmp(t.c_str(), "0") == 0);
  t.clear();
  t.add_hex(0xABCDEF, 8);
  CHECK(strcmp(t.c_str(), "00ABCDEF") == 0);
  t.clear();
  t.add_dec((uint32_t)0).add(',').add_dec((uint32_t)4294967295u).add(',').add_dec((int32_t)-1);
  CHECK(strcmp(t.c_str(), "0,4294967295,-1") == 0);  // 15 chars: fits exactly
  CHECK(t.ok() && t.size() == 15);
  t.add('x');
  CHECK(!t.ok() && t.size() == 15);  // dropped, flagged
  CHECK(mcu::TextBuilder<16>::capacity() == 15);

  mcu::TextBuilder<20> u;
  u.add_hex(0x0013A20041ABCDEFull, 16);
  CHECK(strcmp(u.c_str(), "0013A20041ABCDEF") == 0);
  u.clear();
  u.add_hex(0xFFFFFFFFFFFFFFFFull);
  CHECK(strcmp(u.c_str(), "FFFFFFFFFFFFFFFF") == 0);
  u.clear();
  u.add_dec((int32_t)INT32_MIN);
  CHECK(strcmp(u.c_str(), "-2147483648") == 0);
  u.clear();
  u.add((const char*)0);
  CHECK(u.size() == 0 && u.ok());

  // add_hex round trip through parse_hex_u64
  for (int i = 0; i < 20000; ++i) {
    const uint64_t v = ((uint64_t)rnd32() << 32 | rnd32()) >> rnd_below(64);
    mcu::TextBuilder<20> b;
    b.add_hex(v, rnd_below(17));
    uint64_t back = 0;
    CHECK(b.ok() && mcu::parse_hex_u64(b.c_str(), back) && back == v);
  }
}

static void test_parse() {
  uint32_t v = 123;
  CHECK(mcu::parse_hex_u32("7FFF", v) && v == 0x7FFF);
  CHECK(mcu::parse_hex_u32("3e8", v) && v == 1000);
  CHECK(mcu::parse_hex_u32("0", v) && v == 0);
  CHECK(mcu::parse_hex_u32("FFFFFFFF", v) && v == 0xFFFFFFFFu);
  CHECK(mcu::parse_hex_u32("00000000FFFFFFFF", v) && v == 0xFFFFFFFFu);
  v = 5;
  CHECK(!mcu::parse_hex_u32("100000000", v) && v == 5);
  CHECK(!mcu::parse_hex_u32("", v));
  CHECK(!mcu::parse_hex_u32((const char*)0, v));
  CHECK(!mcu::parse_hex_u32("0x10", v));
  CHECK(!mcu::parse_hex_u32(" 10", v));
  CHECK(!mcu::parse_hex_u32("10 ", v));
  CHECK(!mcu::parse_hex_u32("1G", v));
  CHECK(!mcu::parse_hex_u32("ERROR", v));
  CHECK(v == 5);

  uint64_t w = 0;
  CHECK(mcu::parse_hex_u64("0013A20041ABCDEF", w) && w == 0x0013A20041ABCDEFull);
  CHECK(mcu::parse_hex_u64("ffffffffffffffff", w) && w == 0xFFFFFFFFFFFFFFFFull);
  CHECK(mcu::parse_hex_u64("0000ffffffffffffffff", w) && w == 0xFFFFFFFFFFFFFFFFull);
  CHECK(!mcu::parse_hex_u64("10000000000000000", w));

  CHECK(mcu::parse_dec_u32("0", v) && v == 0);
  CHECK(mcu::parse_dec_u32("4294967295", v) && v == 4294967295u);
  CHECK(!mcu::parse_dec_u32("4294967296", v));
  CHECK(!mcu::parse_dec_u32("-1", v));
  CHECK(!mcu::parse_dec_u32("", v));
  CHECK(!mcu::parse_dec_u32("12a", v));
}

// ---------------------------------------------------------------------------
// Basic command shapes
// ---------------------------------------------------------------------------
static void test_final_ok() {
  Sim<Port> sim;
  mcu::AtClient<> at(sim.port);
  CHECK(at.status() == AtStatus::kIdle && !at.busy());
  CHECK(at.command("AT", sim.now));
  CHECK(at.busy() && at.status() == AtStatus::kBusy);
  sim.advance(at, 3);
  CHECK(sim.wire == "AT\r");
  sim.reply(10, "\r\nOK\r\n");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(!at.busy());
  CHECK(at.response_length() == 0 && strcmp(at.response(), "") == 0);
  CHECK(strcmp(at.final_line(), "OK") == 0);
  CHECK(at.line_count() == 0);
  CHECK(at.poll(sim.now) == AtStatus::kOk);  // result is sticky
}

static void test_info_lines_and_echo() {
  Sim<Port> sim;
  mcu::AtClient<> at(sim.port);
  CHECK(at.command("AT+GMR", sim.now));
  sim.reply(5, "AT+GMR\r\r\nAT version:2.2.0\r\nSDK:v4.4\r\n\r\nOK\r\n");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(strcmp(at.response(), "AT version:2.2.0\nSDK:v4.4") == 0);
  CHECK(at.line_count() == 2);
  CHECK(!at.response_truncated());

  // echo filter off: the echo becomes an info line
  at.set_echo_filter(false);
  CHECK(at.command("ATI", sim.now));
  sim.reply(5, "ATI\r\nBrand X\r\nOK\r\n");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(strcmp(at.response(), "ATI\nBrand X") == 0);
}

static void test_errors() {
  Sim<Port> sim;
  mcu::AtClient<> at(sim.port);
  CHECK(at.command("AT+FOO", sim.now));
  sim.reply(5, "\r\nERROR\r\n");
  CHECK(sim.run(at) == AtStatus::kError);
  CHECK(strcmp(at.final_line(), "ERROR") == 0);

  CHECK(at.command("AT+CPIN?", sim.now));
  sim.reply(5, "\r\n+CME ERROR: 10\r\n");
  CHECK(sim.run(at) == AtStatus::kError);
  CHECK(strcmp(at.final_line(), "+CME ERROR: 10") == 0);

  CHECK(at.command("AT+CMGS", sim.now));
  sim.reply(5, "+CMS ERROR: 500\r");
  CHECK(sim.run(at) == AtStatus::kError);
  CHECK(strcmp(at.final_line(), "+CMS ERROR: 500") == 0);

  // "OK" only counts as a whole line outside raw mode
  CHECK(at.command("AT+X", sim.now));
  sim.reply(5, "NOT OK\rOKAY\rOK\r");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(strcmp(at.response(), "NOT OK\nOKAY") == 0);

  // a long final error line is cut to fit final_line()
  CHECK(at.command("AT+Y", sim.now));
  sim.reply(5, "+CME ERROR: a very long explanation of what went wrong\r");
  CHECK(sim.run(at) == AtStatus::kError);
  CHECK(strlen(at.final_line()) == 23);
  CHECK(strncmp(at.final_line(), "+CME ERROR: a very long", 23) == 0);
}

// XBee style: a query is answered by one line and nothing else.
static void test_lines_mode() {
  Sim<Port> sim;
  mcu::AtClient<> at(sim.port);
  CHECK(at.command("ATID", sim.now, 1000, AtExpect::kLines));
  sim.reply(5, "7FFF\r");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(strcmp(at.response(), "7FFF") == 0);
  CHECK(strcmp(at.final_line(), "") == 0);
  CHECK(at.line_count() == 1);
  uint32_t id;
  CHECK(mcu::parse_hex_u32(at.response(), id) && id == 0x7FFF);

  CHECK(at.command("ATZZ", sim.now, 1000, AtExpect::kLines));
  sim.reply(5, "ERROR\r");
  CHECK(sim.run(at) == AtStatus::kError);

  // several queries in one line, several answers
  CHECK(at.command("ATID,CH,PL", sim.now, 1000, AtExpect::kLines, 3));
  sim.reply(5, "7FFF\rC\r4\r");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(strcmp(at.response(), "7FFF\nC\n4") == 0);
  CHECK(at.line_count() == 3);

  // lines arriving one per poll and an extra line after completion
  Lines urc;
  at.set_unsolicited_handler(&Lines::on, &urc);
  CHECK(at.command("ATSH,SL", sim.now, 1000, AtExpect::kLines, 2));
  sim.reply(5, "13A200\r");
  sim.reply(9, "41ABCDEF\r");
  sim.reply(15, "EXTRA\r");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(strcmp(at.response(), "13A200\n41ABCDEF") == 0);
  sim.advance(at, 20);
  CHECK(urc.v.size() == 1 && urc.v[0] == "EXTRA");
  CHECK(at.status() == AtStatus::kOk);  // unaffected by the extra line

  // lines == 0 is treated as 1
  CHECK(at.command("ATCH", sim.now, 1000, AtExpect::kLines, 0));
  sim.reply(5, "B\r");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(strcmp(at.response(), "B") == 0);
}

static void test_none_mode() {
  Sim<Port> sim;
  mcu::AtClient<> at(sim.port);
  CHECK(at.command("AT+RST", sim.now, 1000, AtExpect::kNone));
  CHECK(at.status() == AtStatus::kOk);  // finished in the call itself (all fitted)
  CHECK(!at.busy());
  sim.advance(at, 2);
  CHECK(sim.wire == "AT+RST\r");
}

static void test_timeout() {
  Sim<Port> sim;
  mcu::AtClient<> at(sim.port);
  const uint32_t t0 = sim.now;
  CHECK(at.command("AT", t0, 250));
  sim.advance(at, 249);
  CHECK(at.status() == AtStatus::kBusy);
  sim.advance(at, 1);
  CHECK(at.status() == AtStatus::kTimeout);
  CHECK(strcmp(at.final_line(), "") == 0);

  // a late answer is just unsolicited
  Lines urc;
  at.set_unsolicited_handler(&Lines::on, &urc);
  sim.reply(1, "OK\r");
  sim.advance(at, 5);
  CHECK(at.status() == AtStatus::kTimeout);
  CHECK(urc.v.size() == 1 && urc.v[0] == "OK");

  // partial answer then silence still times out
  CHECK(at.command("AT+GMR", sim.now, 100));
  sim.reply(5, "version 1\r");
  CHECK(sim.run(at) == AtStatus::kTimeout);
  CHECK(strcmp(at.response(), "version 1") == 0);  // what did arrive is kept
}

static void test_timeout_wrap() {
  Sim<Port> sim(0xFFFFFF80u);
  mcu::AtClient<> at(sim.port);
  CHECK(at.command("AT", sim.now, 300));
  sim.advance(at, 299);
  CHECK(at.status() == AtStatus::kBusy);
  sim.advance(at, 1);
  CHECK(at.status() == AtStatus::kTimeout);
  CHECK(sim.now < 0x1000u);  // crossed zero

  CHECK(at.command("AT", sim.now, 300));
  sim.reply(200, "OK\r");
  CHECK(sim.run(at) == AtStatus::kOk);
}

// ---------------------------------------------------------------------------
// Transmit side: partial writes, rejection rules
// ---------------------------------------------------------------------------
static std::string echo_ok(const std::string&) { return "OK\r"; }

static void test_partial_writes() {
  Sim<SmallTxPort> sim;  // client can queue only 8 bytes at a time
  sim.take_per_ms = 3;   // and the wire drains 3 bytes per ms
  sim.responder = echo_ok;
  mcu::AtClient<> at(sim.port);
  const char* cmd = "AT+CWJAP=\"my network\",\"secret password\"";
  CHECK(at.command(cmd, sim.now, 50));
  CHECK(at.busy());
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(sim.wire == std::string(cmd) + "\r");

  // the timeout restarts on every bit of progress, so a slow drain is fine
  // even though the whole send takes longer than the timeout
  sim.wire.clear();
  sim.take_per_ms = 1;
  CHECK(at.command(cmd, sim.now, 20));
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(sim.wire == std::string(cmd) + "\r");

  // no progress at all: timeout while sending
  sim.wire.clear();
  sim.take_per_ms = 0;
  CHECK(at.command(cmd, sim.now, 20));
  sim.advance(at, 19);
  CHECK(at.status() == AtStatus::kBusy);
  sim.advance(at, 1);
  CHECK(at.status() == AtStatus::kTimeout);
}

static void test_rejections() {
  Sim<Port> sim;
  mcu::AtClient<16, 16, 32> at(sim.port);
  CHECK(at.command("AT+ABCDEFGHIJK", sim.now));   // 14 chars + CR + NUL = 16: fits
  CHECK(!at.command("AT", sim.now));               // busy
  at.cancel();
  CHECK(!at.busy() && at.status() == AtStatus::kIdle);
  CHECK(!at.command("AT+ABCDEFGHIJKL", sim.now));  // 15 + CR + NUL: too long
  CHECK(at.status() == AtStatus::kIdle);
  CHECK(at.raw("123456789012345", sim.now, 0, 100));  // raw: 15 + NUL fits
  at.cancel();
  CHECK(!at.raw("1234567890123456", sim.now, 0, 100));
  CHECK(!at.raw("", sim.now, 0, 100));             // nothing to send
  CHECK(!at.command((const char*)0, sim.now));
  CHECK(at.command("", sim.now));                  // bare CR is allowed (wakes some modems)
  sim.reply(3, "OK\r");
  CHECK(sim.run(at) == AtStatus::kOk);

  // custom terminator
  Sim<Port> sim2;
  mcu::AtClient<> lf(sim2.port, '\n');
  CHECK(lf.command("AT", sim2.now));
  sim2.advance(lf, 2);
  CHECK(sim2.wire == "AT\n");
}

// ---------------------------------------------------------------------------
// raw(): quiet time and escape sequences
// ---------------------------------------------------------------------------
static void test_escape_sequence() {
  Sim<Port> sim;
  mcu::AtClient<> at(sim.port);
  const uint32_t t0 = sim.now;
  CHECK(at.raw("+++", t0, 1000, 1500));
  sim.advance(at, 999);
  CHECK(sim.wire.empty() && sim.port.tx_size() == 0);  // nothing sent during the quiet time
  sim.advance(at, 1);
  CHECK(sim.port.tx_size() == 3);  // written at exactly t0 + 1000
  sim.advance(at, 1);
  CHECK(sim.wire == "+++");  // no terminator
  CHECK(sim.wire_time.size() == 3 && sim.wire_time[0] == t0 + 1001);  // taken on the next tick
  sim.reply(999, "OK\r");  // device answers 1000 ms after the +++ (its own guard time)
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(sim.now == t0 + 2000);

  // data from the far end arrives just before the OK, without a separator
  CHECK(at.raw("+++", sim.now, 1000, 1500));
  sim.advance(at, 1000);
  sim.reply(100, "hello from the other radio");
  sim.reply(1000, "OK\r");
  CHECK(sim.run(at) == AtStatus::kOk);

  // ... including line breaks and a lot of it (longer than LineN)
  mcu::AtClient<64, 16, 64> small(sim.port);
  CHECK(small.raw("+++", sim.now, 10, 1500));
  sim.advance(small, 10);
  sim.reply(10, "line one\r\nline two\r");
  sim.reply(20, "a very long run of data with no line break at all, way over sixteen");
  sim.reply(1000, "OK\r");
  CHECK(sim.run(small) == AtStatus::kOk);
  CHECK(small.line_overflows() >= 1);

  // the far end could also end its data with ERROR: that counts as an answer
  CHECK(at.raw("+++", sim.now, 10, 500));
  sim.advance(at, 10);
  sim.reply(5, "xyzERROR\r");
  CHECK(sim.run(at) == AtStatus::kError);

  // no answer: the timeout counts from when +++ went out, not from the call
  const uint32_t t1 = sim.now;
  CHECK(at.raw("+++", t1, 1000, 1200));
  CHECK(sim.run(at) == AtStatus::kTimeout);
  CHECK(sim.now == t1 + 1000 + 1200);
}

static void test_raw_waits_for_space() {
  Sim<SmallTxPort> sim;
  sim.take_per_ms = 0;  // wire stalled
  mcu::AtClient<> at(sim.port);
  // fill the 8-byte TX buffer with 7 bytes of something else
  const uint8_t junk[7] = {'d', 'a', 't', 'a', '1', '2', '3'};
  CHECK(sim.port.write(junk, 7) == 7);
  CHECK(at.raw("+++", sim.now, 0, 100));
  sim.advance(at, 5);
  CHECK(sim.port.tx_size() == 7);  // only 1 byte free: escape not split
  sim.take_per_ms = 8;
  sim.reply(50, "OK\r");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(sim.wire == "data123+++");
}

// ---------------------------------------------------------------------------
// Receive side: terminators, chunking, overflow, handlers
// ---------------------------------------------------------------------------
static void test_unsolicited() {
  Sim<Port> sim;
  mcu::AtClient<> at(sim.port);
  Lines urc, info;
  at.set_unsolicited_handler(&Lines::on, &urc);
  at.set_info_handler(&Lines::on, &info);

  sim.reply(1, "\r\nRING\r\n\r\n+IPD,5:hello\r\n");
  sim.advance(at, 5);
  CHECK(urc.v.size() == 2 && urc.v[0] == "RING" && urc.v[1] == "+IPD,5:hello");

  CHECK(at.command("AT+LIST", sim.now));
  sim.reply(5, "a\rb\rc\rOK\r");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(info.v.size() == 3 && info.v[2] == "c");
  CHECK(urc.v.size() == 2);  // command answers do not go to the URC handler

  // lines arriving during the quiet time are unsolicited too
  CHECK(at.raw("+++", sim.now, 100, 500));
  sim.reply(10, "late data\r");
  sim.advance(at, 50);
  CHECK(urc.v.size() == 3 && urc.v[2] == "late data");
  at.cancel();

  // without handlers nothing breaks
  at.set_unsolicited_handler(0);
  at.set_info_handler(0);
  sim.reply(1, "RING\r");
  sim.advance(at, 3);
  CHECK(at.command("AT", sim.now));
  sim.reply(1, "x\rOK\r");
  CHECK(sim.run(at) == AtStatus::kOk);
}

static void test_discard_input() {
  Sim<Port> sim;
  mcu::AtClient<> at(sim.port);
  Lines urc;
  at.set_unsolicited_handler(&Lines::on, &urc);
  sim.reply(0, "binary junk without end");
  sim.tick();
  at.discard_input();
  CHECK(sim.port.available() == 0);
  CHECK(at.command("AT", sim.now));
  sim.reply(2, "OK\r");
  CHECK(sim.run(at) == AtStatus::kOk);  // the junk did not glue onto "OK"
  CHECK(urc.v.empty());

  // without discard_input the junk WOULD glue on: documents why it exists
  sim.reply(0, "junk");
  sim.tick();
  CHECK(at.command("AT", sim.now, 50));
  sim.reply(2, "OK\r");
  CHECK(sim.run(at) == AtStatus::kTimeout);
  CHECK(strcmp(at.response(), "junkOK") == 0);
}

static void test_overflow() {
  Sim<Port> sim;
  mcu::AtClient<32, 16, 24> at(sim.port);
  CHECK(at.command("AT+BIG", sim.now));
  sim.reply(5, "0123456789abcdefghijXYZ\rshort\rOK\r");  // first line 23 chars > 15
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(at.line_overflows() == 1);
  // first 15 chars of the long line, '\n', then "short" cut to fit 23 chars
  CHECK(strcmp(at.response(), "0123456789abcde\nshort") == 0);
  CHECK(!at.response_truncated());

  CHECK(at.command("AT+BIGGER", sim.now));
  sim.reply(5, "aaaaaaaaaa\rbbbbbbbbbb\rcccccccccc\rOK\r");
  CHECK(sim.run(at) == AtStatus::kOk);
  CHECK(at.response_truncated());
  CHECK(at.response_length() == 23);
  CHECK(strcmp(at.response(), "aaaaaaaaaa\nbbbbbbbbbb\nc") == 0);
  CHECK(at.line_count() == 3);  // counted even when the text did not fit
}

// ---------------------------------------------------------------------------
// Differential test: random answers, random line endings, random chunking.
// ---------------------------------------------------------------------------
static std::string rnd_word() {
  static const char kChars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+:,=\" ";
  std::string w;
  const uint32_t n = 1 + rnd_below(20);
  for (uint32_t i = 0; i < n; ++i) w.push_back(kChars[rnd_below(sizeof kChars - 1)]);
  // never accidentally a final result
  if (w == "OK" || w == "ERROR" || w.compare(0, 10, "+CME ERROR") == 0 ||
      w.compare(0, 10, "+CMS ERROR") == 0)
    w = "x" + w;
  return w;
}

static const char* rnd_eol() {
  switch (rnd_below(3)) {
    case 0: return "\r";
    case 1: return "\n";
    default: return "\r\n";
  }
}

static void test_differential() {
  for (int trial = 0; trial < 4000; ++trial) {
    Sim<Port> sim(0xFFFFFF00u + rnd_below(512));
    sim.inject_per_ms = 1 + rnd_below(12);
    mcu::AtClient<64, 64, 256> at(sim.port);
    Lines urc;
    at.set_unsolicited_handler(&Lines::on, &urc);

    // some unsolicited lines first
    std::vector<std::string> urcs;
    for (uint32_t k = rnd_below(3); k > 0; --k) {
      urcs.push_back(rnd_word());
      sim.reply(0, urcs.back() + rnd_eol());
    }
    sim.advance(at, 300);
    CHECK(urc.v == urcs);

    const bool lines_mode = rnd_below(3) == 0;
    std::string cmd = "AT+" + rnd_word();
    if (cmd.size() > 40) cmd.resize(40);
    for (size_t i = 0; i < cmd.size(); ++i)
      if (cmd[i] == '\r' || cmd[i] == '\n') cmd[i] = 'x';
    const uint8_t want = (uint8_t)(1 + rnd_below(3));
    CHECK(at.command(cmd.c_str(), sim.now, 5000, lines_mode ? AtExpect::kLines : AtExpect::kFinal,
                     want));

    std::string answer;
    std::vector<std::string> info;
    if (rnd_below(2)) answer += cmd + rnd_eol();  // echo
    const uint32_t ninfo = lines_mode ? want : rnd_below(5);
    for (uint32_t k = 0; k < ninfo; ++k) {
      if (rnd_below(3) == 0) answer += rnd_eol();  // empty lines are skipped
      std::string w = rnd_word();
      if (w == cmd) w += "!";
      info.push_back(w);
      answer += w + rnd_eol();
    }
    AtStatus expect = AtStatus::kOk;
    std::string final_line;
    if (!lines_mode) {
      switch (rnd_below(4)) {
        case 0: final_line = "ERROR"; expect = AtStatus::kError; break;
        case 1: final_line = "+CME ERROR: 3"; expect = AtStatus::kError; break;
        default: final_line = "OK"; break;
      }
      answer += rnd_eol() + final_line + rnd_eol();
    }
    sim.reply(1 + rnd_below(20), answer);
    const AtStatus st = sim.run(at, 20000);
    CHECK(st == expect);
    std::string joined;
    for (size_t k = 0; k < info.size(); ++k) joined += (k ? "\n" : "") + info[k];
    CHECK(joined == at.response());
    CHECK(at.line_count() == info.size());
    CHECK(final_line == at.final_line());
    CHECK(sim.wire == cmd + "\r");
  }
}

// Garbage in, nothing breaks: random bytes, random commands, random timing.
// Run under ASAN/UBSAN this is the memory-safety check.
static void test_fuzz() {
  Sim<SmallTxPort> sim(0xFFFF0000u);
  mcu::AtClient<24, 12, 20> at(sim.port);
  Lines urc;
  at.set_unsolicited_handler(&Lines::on, &urc);
  at.set_info_handler(&Lines::on, &urc);
  uint32_t completions = 0;
  for (int i = 0; i < 300000; ++i) {
    sim.take_per_ms = rnd_below(4);
    if (rnd_below(4) == 0) {
      std::string junk;
      for (uint32_t k = rnd_below(30); k > 0; --k) {
        const uint32_t r = rnd_below(10);
        junk.push_back(r == 0 ? '\r' : r == 1 ? '\n' : (char)rnd_below(256));
      }
      if (rnd_below(5) == 0) junk += rnd_below(2) ? "OK\r" : "ERROR\r";
      sim.reply(0, junk);
    }
    if (!at.busy() && rnd_below(50) == 0) {
      char text[30];
      const uint32_t n = rnd_below(26);
      for (uint32_t k = 0; k < n; ++k) text[k] = (char)('A' + rnd_below(26));
      text[n] = 0;
      const AtExpect e = (AtExpect)rnd_below(3);
      const bool accepted = rnd_below(3) == 0
                                ? at.raw(text, sim.now, rnd_below(20), 1 + rnd_below(100), e,
                                         (uint8_t)rnd_below(4))
                                : at.command(text, sim.now, 1 + rnd_below(100), e,
                                             (uint8_t)rnd_below(4));
      const size_t limit = 24 - 1 - 1;  // command(): text + CR + NUL
      if (n <= limit) CHECK(accepted || n == 0);
      if (n > 23) CHECK(!accepted);
    }
    if (at.busy() && rnd_below(2000) == 0) at.cancel();
    const AtStatus before = at.status();
    sim.now = (uint32_t)(sim.now + 1);
    sim.tick();
    const AtStatus st = at.poll(sim.now);
    CHECK(st == at.status());
    CHECK((st == AtStatus::kBusy) == at.busy());
    if (before == AtStatus::kBusy && st != AtStatus::kBusy) ++completions;
    CHECK(strlen(at.response()) == at.response_length());
    CHECK(at.response_length() <= 19);
    CHECK(strlen(at.final_line()) <= 23);
    if (urc.v.size() > 1000) urc.v.clear();
  }
  CHECK(completions > 1000);
}

int main() {
  test_text_builder();
  test_parse();
  test_final_ok();
  test_info_lines_and_echo();
  test_errors();
  test_lines_mode();
  test_none_mode();
  test_timeout();
  test_timeout_wrap();
  test_partial_writes();
  test_rejections();
  test_escape_sequence();
  test_raw_waits_for_space();
  test_unsolicited();
  test_discard_input();
  test_overflow();
  test_differential();
  test_fuzz();

  printf("%lu checks, %lu failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
