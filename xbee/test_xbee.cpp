// Host-side tests for xbee/: XBeeConfigurator against a simulated XBee3.
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_xbee.cpp -o test_xbee && ./test_xbee
//
// The simulated radio follows the documented XBee3 behaviour: command mode is
// entered with guard-time silence around "+++", it leaves again after CT of
// idle, commands end in CR, queries answer a bare hex value, sets answer OK
// (or ERROR), a new BD only takes effect on AC, and WR saves to "flash" that
// survives a reboot().

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "hal/loopback_stream.hpp"
#include "xbee/xbee_config.hpp"

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

static uint64_t g_rng = 0x1319E5A3C0DE1234ull;
static uint32_t rnd32() {
  g_rng ^= g_rng >> 12;
  g_rng ^= g_rng << 25;
  g_rng ^= g_rng >> 27;
  return (uint32_t)((g_rng * 0x2545F4914F6CDD1Dull) >> 32);
}
static uint32_t rnd_below(uint32_t n) { return rnd32() % n; }

using mcu::XBeeError;
using mcu::XBeeStatus;

static std::string hex(uint64_t v) {
  char b[20];
  snprintf(b, sizeof b, "%llX", (unsigned long long)v);
  return b;
}

static const uint32_t kStdBaud[11] = {1200,  2400,  4800,   9600,   19200, 38400,
                                      57600, 115200, 230400, 460800, 921600};

// ---------------------------------------------------------------------------
// Simulated XBee3 on a serial link
// ---------------------------------------------------------------------------
struct Param {
  char n0, n1;
  bool text;
  bool readonly;
  uint64_t val, lo, hi;
  std::string t;
};

struct FakeXBee {
  mcu::LoopbackStream<256, 256> port;
  mcu::ManualBaudControl host;  // the host's UART setting, changed through BaudControl
  uint32_t dev_baud;            // what the radio's UART is running at
  uint32_t now;

  uint32_t gt, ct;
  bool in_cmd;
  uint32_t ct_deadline;
  uint32_t last_rx;
  int plus_count;
  bool plus_valid;
  bool esc_pending;
  uint32_t esc_deadline;
  std::string line;

  std::vector<Param> params, flash;
  std::vector<std::string> log;  // every command line executed in command mode
  int wr_count, ac_count;

  // quirks
  bool ok_on_baud_change;  // answer ATAC at the old rate before switching
  bool garbage_on_baud_change;
  bool clamp_ch;           // accept any CH but store it clamped (makes verify fail)
  bool absent;             // nothing is connected

  struct Pending {
    uint32_t at;
    std::string bytes;
  };
  std::vector<Pending> out;

  FakeXBee()
      : host(9600),
        dev_baud(9600),
        now(5000),
        gt(1000),
        ct(10000),
        in_cmd(false),
        ct_deadline(0),
        last_rx(0),
        plus_count(0),
        plus_valid(false),
        esc_pending(false),
        esc_deadline(0),
        wr_count(0),
        ac_count(0),
        ok_on_baud_change(true),
        garbage_on_baud_change(false),
        clamp_ch(false),
        absent(false) {
    add_num("ID", 0x3332, 0, 0xFFFFFFFFFFFFFFFFull);
    add_num("CH", 0x0C, 0x0B, 0x1A);
    add_num("PL", 4, 0, 4);
    add_num("BD", 3, 0, 0xEC400);
    add_num("AP", 0, 0, 2);
    add_num("CE", 0, 0, 1);
    add_num("DH", 0, 0, 0xFFFFFFFFull);
    add_num("DL", 0, 0, 0xFFFFFFFFull);
    add_text("NI", " ");
    add_ro("SH", 0x0013A200);
    add_ro("SL", 0x41ABCDEF);
    flash = params;
    last_rx = now - 100000;
  }

  void add_num(const char* n, uint64_t v, uint64_t lo, uint64_t hi) {
    Param p = {n[0], n[1], false, false, v, lo, hi, ""};
    params.push_back(p);
  }
  void add_ro(const char* n, uint64_t v) {
    Param p = {n[0], n[1], false, true, v, 0, 0, ""};
    params.push_back(p);
  }
  void add_text(const char* n, const char* t) {
    Param p = {n[0], n[1], true, false, 0, 0, 0, t};
    params.push_back(p);
  }

  Param* find(char a, char b) {
    for (size_t i = 0; i < params.size(); ++i)
      if (params[i].n0 == a && params[i].n1 == b) return &params[i];
    return 0;
  }
  uint64_t get(const char* n) { return find(n[0], n[1])->val; }
  std::string get_text(const char* n) { return find(n[0], n[1])->t; }
  Param* fl(const char* n) {
    for (size_t i = 0; i < flash.size(); ++i)
      if (flash[i].n0 == n[0] && flash[i].n1 == n[1]) return &flash[i];
    return 0;
  }

  // Power cycle: RAM is lost, flash is reloaded, baud follows the saved BD.
  void reboot() {
    params = flash;
    in_cmd = false;
    line.clear();
    out.clear();
    plus_count = 0;
    esc_pending = false;
    apply_baud_now();
  }

  void apply_baud_now() {
    const uint64_t bd = get("BD");
    dev_baud = bd <= 10 ? kStdBaud[bd] : (uint32_t)bd;
  }

  size_t count_log(const char* prefix) const {
    size_t n = 0;
    for (size_t i = 0; i < log.size(); ++i)
      if (log[i].compare(0, strlen(prefix), prefix) == 0) ++n;
    return n;
  }

  void reply(uint32_t after, const std::string& s) {
    Pending p = {(uint32_t)(now + after), s};
    out.push_back(p);
  }

  void execute(const std::string& cmd) {
    log.push_back(cmd);
    if (cmd.size() < 2 || (cmd[0] != 'A' && cmd[0] != 'a') || (cmd[1] != 'T' && cmd[1] != 't')) {
      reply(5, "ERROR\r");
      return;
    }
    if (cmd.size() == 2) {
      reply(5, "OK\r");
      return;
    }
    if (cmd.size() < 4) {
      reply(5, "ERROR\r");
      return;
    }
    const char a = cmd[2], b = cmd[3];
    const std::string rest = cmd.substr(4);
    if (rest.empty()) {
      if (a == 'A' && b == 'C') return do_ac();
      if (a == 'W' && b == 'R') {
        flash = params;
        ++wr_count;
        reply(40, "OK\r");
        return;
      }
      if (a == 'C' && b == 'N') {
        in_cmd = false;
        reply(5, "OK\r");
        return;
      }
      if (a == 'R' && b == 'E') {
        params = flash;  // good enough for this suite
        reply(5, "OK\r");
        return;
      }
      Param* p = find(a, b);
      if (p == 0) {
        reply(5, "ERROR\r");
        return;
      }
      reply(5, (p->text ? p->t : hex(p->val)) + "\r");
      return;
    }
    Param* p = find(a, b);
    if (p == 0 || p->readonly) {
      reply(5, "ERROR\r");
      return;
    }
    if (p->text) {
      if (rest.size() > 20) {
        reply(5, "ERROR\r");
        return;
      }
      p->t = rest;
      reply(5, "OK\r");
      return;
    }
    if (rest.size() > 16) {
      reply(5, "ERROR\r");
      return;
    }
    uint64_t v = 0;
    for (size_t i = 0; i < rest.size(); ++i) {
      const char c = rest[i];
      int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'A' && c <= 'F') ? c - 'A' + 10
            : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : -1;
      if (d < 0) {
        reply(5, "ERROR\r");
        return;
      }
      v = (v << 4) | (uint64_t)d;
    }
    if (a == 'C' && b == 'H' && clamp_ch) {
      p->val = v < p->lo ? p->lo : (v > p->hi ? p->hi : v);
      reply(5, "OK\r");
      return;
    }
    if (v < p->lo || v > p->hi || (a == 'B' && b == 'D' && v > 10 && v < 0x12C)) {
      reply(5, "ERROR\r");
      return;
    }
    p->val = v;
    reply(5, "OK\r");
  }

  void do_ac() {
    ++ac_count;
    const uint64_t bd = get("BD");
    const uint32_t nb = bd <= 10 ? kStdBaud[bd] : (uint32_t)bd;
    if (nb != dev_baud) {
      if (ok_on_baud_change) reply(1, "OK\r");  // goes out at the old rate
      if (garbage_on_baud_change) reply(3, std::string("\xFF\x00\xFE", 3));
      dev_baud = nb;  // from now on the radio talks at the new rate
    } else {
      reply(20, "OK\r");
    }
  }

  void on_byte(uint8_t b) {
    const uint32_t gap = (uint32_t)(now - last_rx);
    last_rx = now;
    esc_pending = false;  // any byte cancels a pending escape
    if (!in_cmd) {
      if (b == '+') {
        if (plus_count == 0) plus_valid = gap >= gt;
        else if (gap >= gt) plus_valid = false;
        ++plus_count;
        if (plus_count == 3 && plus_valid) {
          esc_pending = true;
          esc_deadline = (uint32_t)(now + gt);
        }
        if (plus_count > 3) plus_valid = false;
      } else {
        plus_count = 0;
        plus_valid = false;
      }
      return;
    }
    ct_deadline = (uint32_t)(now + ct);
    if (b == '\r') {
      const std::string c = line;
      line.clear();
      execute(c);
    } else {
      line.push_back((char)b);
    }
  }

  // One millisecond of radio time.
  void tick() {
    ++now;
    if (in_cmd && mcu::time_reached(now, ct_deadline)) {
      in_cmd = false;
      line.clear();
    }
    uint8_t buf[64];
    const size_t n = port.take(buf, sizeof buf);
    if (!absent && host.baud() == dev_baud)
      for (size_t i = 0; i < n; ++i) on_byte(buf[i]);
    if (!absent && esc_pending && mcu::time_reached(now, esc_deadline)) {
      esc_pending = false;
      plus_count = 0;
      in_cmd = true;
      ct_deadline = (uint32_t)(now + ct);
      line.clear();
      reply(0, "OK\r");
    }
    while (!out.empty() && mcu::time_reached(now, out[0].at)) {
      std::string& s = out[0].bytes;
      // A reply sent while the rates differ is unreadable; sent at the old rate
      // during a baud change it still goes through (dev_baud already moved on).
      const bool readable = host.baud() == dev_baud || s == "OK\r";
      size_t k = port.rx_space() < s.size() ? port.rx_space() : s.size();
      if (readable && !absent) port.inject(reinterpret_cast<const uint8_t*>(s.data()), k);
      out.erase(out.begin());
    }
  }
};

// Steps radio and driver together, 1 ms at a time, until the job ends.
template <typename X>
static XBeeStatus drive(X& xb, FakeXBee& f, uint32_t max_ms = 200000) {
  XBeeStatus st = xb.poll(f.now);
  for (uint32_t i = 0; i < max_ms && st == XBeeStatus::kBusy; ++i) {
    f.tick();
    st = xb.poll(f.now);
  }
  return st;
}

template <typename X>
static void idle(X& xb, FakeXBee& f, uint32_t ms) {
  for (uint32_t i = 0; i < ms; ++i) {
    f.tick();
    xb.poll(f.now);
  }
}

typedef mcu::XBeeConfigurator<> XB;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
static void test_helpers() {
  static const mcu::XBeeSetting a = mcu::xb_num("ID", 0x7FFF);
  CHECK(a.c0 == 'I' && a.c1 == 'D' && !a.is_text && a.num == 0x7FFF);
  static const mcu::XBeeSetting b = mcu::xb_text("NI", "Base");
  CHECK(b.c0 == 'N' && b.c1 == 'I' && b.is_text && strcmp(b.text, "Base") == 0);
  uint32_t baud = 0;
  CHECK(mcu::xbee_bd_to_baud(0, baud) && baud == 1200);
  CHECK(mcu::xbee_bd_to_baud(3, baud) && baud == 9600);
  CHECK(mcu::xbee_bd_to_baud(7, baud) && baud == 115200);
  CHECK(mcu::xbee_bd_to_baud(10, baud) && baud == 921600);
  CHECK(!mcu::xbee_bd_to_baud(11, baud));
  CHECK(!mcu::xbee_bd_to_baud(0x12B, baud));
  CHECK(mcu::xbee_bd_to_baud(0x12C, baud) && baud == 300);
  CHECK(mcu::xbee_bd_to_baud(0xEC400, baud) && baud == 967680);
  CHECK(!mcu::xbee_bd_to_baud(0xEC401, baud));
}

static void test_profile_basic() {
  FakeXBee f;
  XB xb(f.port, &f.host);
  static const mcu::XBeeSetting prof[] = {
      mcu::xb_num("ID", 0x7FFF), mcu::xb_num("CH", 0x0C), mcu::xb_num("PL", 4),
      mcu::xb_text("NI", "Base")};
  const uint32_t t0 = f.now;
  CHECK(xb.status() == XBeeStatus::kIdle);
  CHECK(xb.begin_profile(prof, 4, f.now));
  CHECK(xb.busy());
  CHECK(!xb.begin_profile(prof, 4, f.now));  // busy
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(xb.error() == XBeeError::kNone);
  CHECK(xb.changed_count() == 2);  // ID and NI differ; CH and PL already match
  CHECK(f.get("ID") == 0x7FFF && f.get_text("NI") == "Base");
  CHECK(f.wr_count == 1 && f.ac_count == 1);
  CHECK(f.fl("ID")->val == 0x7FFF && f.fl("NI")->t == "Base");  // saved
  CHECK(!f.in_cmd);                                             // ATCN sent
  CHECK(!xb.in_command_mode(f.now));
  CHECK(f.count_log("ATCN") == 1);
  // only the two differing settings were written
  CHECK(f.count_log("ATID7FFF") == 1 && f.count_log("ATNIBase") == 1);
  CHECK(f.count_log("ATCH") == 1 && f.count_log("ATPL") == 1);  // queries only (+ verify none)
  // guard time before and after +++ dominates the duration
  CHECK(f.now - t0 >= 2000 && f.now - t0 < 4500);
}

static void test_profile_nothing_to_do() {
  FakeXBee f;
  XB xb(f.port, &f.host);
  static const mcu::XBeeSetting prof[] = {mcu::xb_num("ID", 0x7FFF), mcu::xb_text("NI", "Base")};
  CHECK(xb.begin_profile(prof, 2, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  const int wr = f.wr_count, ac = f.ac_count;
  f.log.clear();

  // already configured: radio is read, nothing is written, nothing is saved
  CHECK(xb.begin_profile(prof, 2, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(xb.changed_count() == 0);
  CHECK(f.wr_count == wr && f.ac_count == ac);
  CHECK(f.count_log("ATWR") == 0 && f.count_log("ATAC") == 0);
  CHECK(f.count_log("ATID7FFF") == 0 && f.count_log("ATNIBase") == 0);
  CHECK(f.count_log("ATCN") == 1);  // still leaves command mode

  // survives a power cycle
  f.reboot();
  CHECK(f.get("ID") == 0x7FFF && f.get_text("NI") == "Base");
}

static void test_command_mode_tracking() {
  FakeXBee f;
  XB xb(f.port, &f.host);
  xb.options().exit = false;
  static const mcu::XBeeSetting p1[] = {mcu::xb_num("PL", 2)};
  static const mcu::XBeeSetting p2[] = {mcu::xb_num("PL", 1)};
  CHECK(xb.begin_profile(p1, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(f.in_cmd && xb.in_command_mode(f.now));
  CHECK(f.count_log("ATWR") == 1);
  idle(xb, f, 3000);
  CHECK(xb.in_command_mode(f.now));

  // second run right away: no +++, so no guard time
  const uint32_t t0 = f.now;
  CHECK(xb.begin_profile(p2, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(f.now - t0 < 800);
  CHECK(f.get("PL") == 1);

  // radio's CT passes: the driver's own CT view says "not in command mode"
  idle(xb, f, 10500);
  CHECK(!f.in_cmd);
  CHECK(!xb.in_command_mode(f.now));
  static const mcu::XBeeSetting p3[] = {mcu::xb_num("PL", 3)};
  const uint32_t t1 = f.now;
  CHECK(xb.begin_profile(p3, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(f.now - t1 >= 2000);  // had to enter again
  CHECK(f.get("PL") == 3);

  // driver thinks it is in command mode but the radio timed out sooner
  xb.options().command_mode_ms = 60000;
  idle(xb, f, 12000);
  CHECK(!f.in_cmd && xb.in_command_mode(f.now));
  CHECK(xb.begin_profile(p1, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed);
  CHECK(xb.error() == XBeeError::kTimeout && xb.failed_setting() == 0);
  CHECK(!xb.in_command_mode(f.now));  // forgotten after a timeout
  // and the next run recovers by entering again
  CHECK(xb.begin_profile(p1, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(f.get("PL") == 2);
}

static void test_baud_change() {
  for (int variant = 0; variant < 3; ++variant) {
    FakeXBee f;
    f.ok_on_baud_change = (variant != 1);
    f.garbage_on_baud_change = (variant == 2);
    XB xb(f.port, &f.host);
    static const mcu::XBeeSetting prof[] = {mcu::xb_num("ID", 0x1234), mcu::xb_num("BD", 7)};
    CHECK(f.host.baud() == 9600 && f.dev_baud == 9600);
    CHECK(xb.begin_profile(prof, 2, f.now));
    CHECK(drive(xb, f) == XBeeStatus::kDone);
    CHECK(xb.changed_count() == 2);
    CHECK(f.dev_baud == 115200 && f.host.baud() == 115200);  // host followed the radio
    CHECK(xb.detected_baud() == 115200);
    CHECK(f.get("BD") == 7 && f.get("ID") == 0x1234);
    CHECK(f.fl("BD")->val == 7 && f.wr_count == 1);          // saved only after verifying
    CHECK(f.count_log("ATWR") == 1);
    // the radio comes back up at the saved rate
    f.reboot();
    CHECK(f.dev_baud == 115200);
  }

  // a custom BD reads back as the rate the radio can really do: not an error
  {
    FakeXBee f;
    XB xb(f.port, &f.host);
    static const mcu::XBeeSetting prof[] = {mcu::xb_num("BD", 0x1C200)};  // 115200 as a raw rate
    CHECK(xb.begin_profile(prof, 1, f.now));
    CHECK(drive(xb, f) == XBeeStatus::kDone);
    CHECK(f.host.baud() == 115200);
  }

  // without a BaudControl a BD change is refused up front
  {
    FakeXBee f;
    XB xb(f.port);
    static const mcu::XBeeSetting prof[] = {mcu::xb_num("BD", 7)};
    CHECK(!xb.begin_profile(prof, 1, f.now));
    CHECK(xb.status() == XBeeStatus::kIdle);
    static const mcu::XBeeSetting other[] = {mcu::xb_num("PL", 2)};  // everything else still works
    CHECK(xb.begin_profile(other, 1, f.now));
    CHECK(drive(xb, f) == XBeeStatus::kDone);
  }

  // host UART refuses the new rate
  {
    FakeXBee f;
    mcu::ManualBaudControl picky(9600, 57600);  // cannot do 115200
    XB xb(f.port, &picky);
    static const mcu::XBeeSetting prof[] = {mcu::xb_num("BD", 7)};
    // the host side of the link follows `picky`, so mirror it
    CHECK(xb.begin_profile(prof, 1, f.now));
    XBeeStatus st = xb.poll(f.now);
    for (int i = 0; i < 20000 && st == XBeeStatus::kBusy; ++i) {
      f.tick();
      st = xb.poll(f.now);
    }
    CHECK(st == XBeeStatus::kFailed && xb.error() == XBeeError::kBaudChange);
  }
}

static void test_find_baud() {
  // radio saved at 115200, host starts at 9600; try both
  FakeXBee f;
  f.flash[3].val = 7;  // BD
  f.reboot();
  CHECK(f.dev_baud == 115200 && f.host.baud() == 9600);
  XB xb(f.port, &f.host);
  static const uint32_t cands[] = {9600, 115200, 57600};
  xb.options().baud_candidates = cands;
  xb.options().baud_count = 3;
  static const mcu::XBeeSetting prof[] = {mcu::xb_num("PL", 1)};
  CHECK(xb.begin_profile(prof, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(xb.detected_baud() == 115200 && f.host.baud() == 115200);
  CHECK(f.get("PL") == 1);

  // single enter job reports the same
  FakeXBee g;
  g.flash[3].val = 6;  // 57600
  g.reboot();
  XB xb2(g.port, &g.host);
  xb2.options().baud_candidates = cands;
  xb2.options().baud_count = 3;
  CHECK(xb2.begin_enter(g.now));
  CHECK(drive(xb2, g) == XBeeStatus::kDone);
  CHECK(xb2.detected_baud() == 57600 && g.in_cmd && xb2.in_command_mode(g.now));
}

static void test_no_radio() {
  FakeXBee f;
  f.absent = true;
  XB xb(f.port, &f.host);
  static const mcu::XBeeSetting prof[] = {mcu::xb_num("PL", 1)};
  const uint32_t t0 = f.now;
  CHECK(xb.begin_profile(prof, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed);
  CHECK(xb.error() == XBeeError::kNoResponse && xb.failed_setting() == -1);
  // each attempt: 1 s of silence, then up to 2 s for the OK
  CHECK(f.now - t0 >= 2 * 3000 && f.now - t0 <= 2 * 3000 + 50);
  CHECK(!xb.in_command_mode(f.now));

  // with three candidate rates: 3 rates x 2 attempts
  static const uint32_t cands[] = {9600, 19200, 115200};
  xb.options().baud_candidates = cands;
  xb.options().baud_count = 3;
  const uint32_t t1 = f.now;
  CHECK(xb.begin_enter(f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed);
  CHECK(f.now - t1 >= 3 * 2 * 3000 && f.now - t1 <= 3 * 2 * 3000 + 100);

  // not waiting long enough for the radio's reply (it answers after its own
  // 1 s guard time) is also just "no response"
  FakeXBee g;
  XB xb2(g.port, &g.host);
  xb2.options().guard_ms = 200;
  xb2.options().reply_ms = 500;
  xb2.options().enter_attempts = 1;
  CHECK(xb2.begin_enter(g.now));
  CHECK(drive(xb2, g) == XBeeStatus::kFailed && xb2.error() == XBeeError::kNoResponse);
}

static void test_command_errors() {
  FakeXBee f;
  XB xb(f.port, &f.host);

  // value out of range: the radio says ERROR, nothing is applied or saved
  static const mcu::XBeeSetting p1[] = {mcu::xb_num("PL", 2), mcu::xb_num("CH", 0x99),
                                        mcu::xb_text("NI", "X")};
  CHECK(xb.begin_profile(p1, 3, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed);
  CHECK(xb.error() == XBeeError::kCommandError && xb.failed_setting() == 1);
  CHECK(f.count_log("ATWR") == 0 && f.count_log("ATAC") == 0);
  CHECK(f.fl("PL")->val == 4);  // flash untouched
  CHECK(strcmp(xb.last_final_line(), "ERROR") == 0);

  // read-only setting
  static const mcu::XBeeSetting p2[] = {mcu::xb_num("SL", 5)};
  CHECK(xb.begin_profile(p2, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed);
  CHECK(xb.error() == XBeeError::kCommandError && xb.failed_setting() == 0);

  // unknown setting fails on the read
  static const mcu::XBeeSetting p3[] = {mcu::xb_num("ZZ", 5)};
  CHECK(xb.begin_profile(p3, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed);
  CHECK(xb.error() == XBeeError::kCommandError && xb.failed_setting() == 0);

  // a radio that stores a different value than it was given
  FakeXBee g;
  g.clamp_ch = true;
  XB xb2(g.port, &g.host);
  static const mcu::XBeeSetting p4[] = {mcu::xb_num("CH", 0x40)};
  CHECK(xb2.begin_profile(p4, 1, g.now));
  CHECK(drive(xb2, g) == XBeeStatus::kFailed);
  CHECK(xb2.error() == XBeeError::kVerifyFailed && xb2.failed_setting() == 0);
  CHECK(g.count_log("ATWR") == 0);  // never saved

  // verify off: the same radio "succeeds"
  FakeXBee h;
  h.clamp_ch = true;
  XB xb3(h.port, &h.host);
  xb3.options().verify = false;
  CHECK(xb3.begin_profile(p4, 1, h.now));
  CHECK(drive(xb3, h) == XBeeStatus::kDone);
}

static void test_bad_arguments() {
  FakeXBee f;
  XB xb(f.port, &f.host);
  static const mcu::XBeeSetting ok[] = {mcu::xb_num("PL", 1)};
  CHECK(!xb.begin_profile(0, 1, f.now));
  CHECK(!xb.begin_profile(ok, 0, f.now));
  CHECK(!xb.begin_profile(ok, 25, f.now));  // more than MaxSettings
  static const mcu::XBeeSetting bad_name[] = {{'P', '?', false, 1, 0}};
  CHECK(!xb.begin_profile(bad_name, 1, f.now));
  static const mcu::XBeeSetting comma[] = {mcu::xb_text("NI", "a,b")};
  CHECK(!xb.begin_profile(comma, 1, f.now));
  static const mcu::XBeeSetting cr[] = {mcu::xb_text("NI", "a\rb")};
  CHECK(!xb.begin_profile(cr, 1, f.now));
  static const mcu::XBeeSetting too_long[] = {
      mcu::xb_text("NI", "0123456789012345678901234567890123456")};
  CHECK(!xb.begin_profile(too_long, 1, f.now));
  static const mcu::XBeeSetting null_text[] = {mcu::xb_text("NI", 0)};
  CHECK(!xb.begin_profile(null_text, 1, f.now));
  static const mcu::XBeeSetting bad_bd[] = {mcu::xb_num("BD", 11)};
  CHECK(!xb.begin_profile(bad_bd, 1, f.now));
  static const mcu::XBeeSetting big_bd[] = {mcu::xb_num("BD", 0xEC401)};
  CHECK(!xb.begin_profile(big_bd, 1, f.now));
  CHECK(xb.status() == XBeeStatus::kIdle);

  CHECK(!xb.begin_get("I", f.now));
  CHECK(!xb.begin_get("IDX", f.now));
  CHECK(!xb.begin_get("I?", f.now));
  CHECK(!xb.begin_get(0, f.now));
  CHECK(!xb.begin_set_text("NI", 0, f.now));
  CHECK(!xb.begin_set_text("NI", "a,b", f.now));
  CHECK(!xb.begin_command("A", f.now));
  CHECK(xb.status() == XBeeStatus::kIdle);

  // exactly MaxSettings is fine
  mcu::XBeeSetting many[24];
  for (int i = 0; i < 24; ++i) many[i] = mcu::xb_num("PL", 4);
  CHECK(xb.begin_profile(many, 24, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(xb.changed_count() == 0);
}

static void test_singles() {
  FakeXBee f;
  XB xb(f.port, &f.host);
  CHECK(xb.begin_enter(f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(xb.in_command_mode(f.now));

  CHECK(xb.begin_get("ID", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(xb.value() == 0x3332);

  CHECK(xb.begin_get("SH", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone && xb.value() == 0x0013A200);
  CHECK(xb.begin_get("SL", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone && xb.value() == 0x41ABCDEF);

  CHECK(xb.begin_get_text("NI", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(strcmp(xb.text(), " ") == 0);

  CHECK(xb.begin_set("CH", 0x10, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(f.get("CH") == 0x10 && f.wr_count == 0 && f.ac_count == 0);  // set only
  CHECK(xb.begin_set_text("NI", "Rover 2", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone && f.get_text("NI") == "Rover 2");
  CHECK(xb.begin_set("CH", 0x99, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed && xb.error() == XBeeError::kCommandError);

  CHECK(xb.begin_command("AC", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone && f.ac_count == 1);
  CHECK(xb.begin_command("WR", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone && f.wr_count == 1 && f.fl("CH")->val == 0x10);
  CHECK(xb.begin_command("XX", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed && xb.error() == XBeeError::kCommandError);
  CHECK(xb.begin_get("ZZ", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed && xb.error() == XBeeError::kCommandError);

  CHECK(xb.begin_command("CN", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(!f.in_cmd && !xb.in_command_mode(f.now));

  // a query that is not a number
  CHECK(xb.begin_enter(f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(xb.begin_set_text("NI", "hello", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  // reading NI as a number: "hello" is not hex
  CHECK(xb.begin_get("NI", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kFailed && xb.error() == XBeeError::kBadReply);

  // a long text result is cut to fit
  f.find('N', 'I')->t = "12345678901234567890";
  CHECK(xb.begin_get_text("NI", f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(strlen(xb.text()) == 20);
}

static void test_wide_values() {
  FakeXBee f;
  XB xb(f.port, &f.host);
  static const mcu::XBeeSetting prof[] = {
      mcu::xb_num("ID", 0x0123456789ABCDEFull), mcu::xb_num("DH", 0xFFFFFFFFull),
      mcu::xb_num("DL", 0)};
  CHECK(xb.begin_profile(prof, 3, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(f.get("ID") == 0x0123456789ABCDEFull && f.get("DH") == 0xFFFFFFFFull);
  CHECK(f.count_log("ATID0123456789ABCDEF") == 0);  // no leading zeros are sent
  CHECK(f.count_log("ATID123456789ABCDEF") == 1);
  CHECK(xb.changed_count() == 2);

  // full 64-bit value and zero round trip
  static const mcu::XBeeSetting p2[] = {mcu::xb_num("ID", 0xFFFFFFFFFFFFFFFFull)};
  CHECK(xb.begin_profile(p2, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone && f.get("ID") == 0xFFFFFFFFFFFFFFFFull);
  static const mcu::XBeeSetting p3[] = {mcu::xb_num("ID", 0)};
  CHECK(xb.begin_profile(p3, 1, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone && f.get("ID") == 0);
  CHECK(f.count_log("ATID0") == 1);
}

static void test_time_wrap() {
  FakeXBee f;
  f.now = 0xFFFFFC00u;
  f.last_rx = f.now - 100000;
  XB xb(f.port, &f.host);
  static const mcu::XBeeSetting prof[] = {mcu::xb_num("ID", 0x7FFF), mcu::xb_num("BD", 5)};
  CHECK(xb.begin_profile(prof, 2, f.now));
  CHECK(drive(xb, f) == XBeeStatus::kDone);
  CHECK(f.now < 0x10000u);  // crossed zero during the job
  CHECK(f.get("ID") == 0x7FFF && f.host.baud() == 38400 && f.wr_count == 1);
}

static void test_unsolicited_passthrough() {
  // data from the other radio around the guard time does not break entry
  FakeXBee f;
  XB xb(f.port, &f.host);
  CHECK(xb.begin_enter(f.now));
  // the radio is quiet; a late reply of old data arrives while we wait
  f.reply(300, "stale\r");
  CHECK(drive(xb, f) == XBeeStatus::kDone);
}

// Random profiles against random radio states. Invariants: on Done the radio's
// RAM and flash hold the profile; flash is written only if something differed;
// the radio is left out of command mode.
static void test_fuzz() {
  for (int trial = 0; trial < 3000; ++trial) {
    FakeXBee f;
    f.ok_on_baud_change = (rnd32() & 1) != 0;
    f.garbage_on_baud_change = (rnd32() & 1) != 0;
    // random starting state, saved to flash
    f.find('I', 'D')->val = rnd32();
    f.find('C', 'H')->val = 0x0B + rnd_below(16);
    f.find('P', 'L')->val = rnd_below(5);
    f.find('A', 'P')->val = rnd_below(3);
    f.find('N', 'I')->t = rnd_below(2) ? "node" : " ";
    const bool change_baud = rnd_below(3) == 0;
    f.flash = f.params;
    f.reboot();

    mcu::XBeeSetting prof[8];
    size_t n = 0;
    uint64_t want[4] = {0, 0, 0, 0};
    bool have[4] = {false, false, false, false};
    bool differs = false;
    // distinct names in random order: ID CH PL AP
    unsigned order[4] = {0, 1, 2, 3};
    for (unsigned i = 3; i > 0; --i) {
      const unsigned j = rnd_below(i + 1);
      const unsigned t = order[i];
      order[i] = order[j];
      order[j] = t;
    }
    const size_t pick = 1 + rnd_below(4);
    for (size_t i = 0; i < pick; ++i) {
      const unsigned w = order[i];
      uint64_t v = 0;
      uint64_t cur = 0;
      switch (w) {
        case 0: cur = f.get("ID"); v = rnd_below(2) ? cur : (uint64_t)rnd32(); prof[n] = mcu::xb_num("ID", v); break;
        case 1: cur = f.get("CH"); v = rnd_below(2) ? cur : 0x0B + rnd_below(16); prof[n] = mcu::xb_num("CH", v); break;
        case 2: cur = f.get("PL"); v = rnd_below(2) ? cur : rnd_below(5); prof[n] = mcu::xb_num("PL", v); break;
        default: cur = f.get("AP"); v = rnd_below(2) ? cur : rnd_below(3); prof[n] = mcu::xb_num("AP", v); break;
      }
      ++n;
      have[w] = true;
      want[w] = v;
      if (v != cur) differs = true;
    }
    uint64_t new_bd = 0;
    if (change_baud) {
      new_bd = 4 + rnd_below(6);  // 19200 .. 921600
      prof[n++] = mcu::xb_num("BD", new_bd);
      if (new_bd != f.get("BD")) differs = true;
    }

    XB xb(f.port, &f.host);
    // start the host at the radio's rate
    f.host.set_baud(f.dev_baud);
    if (rnd_below(3) == 0) idle(xb, f, rnd_below(3000));
    const int wr0 = f.wr_count;
    if (!xb.begin_profile(prof, n, f.now)) {
      CHECK(false);
      continue;
    }
    const XBeeStatus st = drive(xb, f);
    CHECK(st == XBeeStatus::kDone);
    if (st != XBeeStatus::kDone) continue;
    static const char* const kN[4] = {"ID", "CH", "PL", "AP"};
    for (unsigned w = 0; w < 4; ++w)
      if (have[w]) CHECK(f.get(kN[w]) == want[w] && f.fl(kN[w])->val == want[w]);
    if (change_baud) {
      CHECK(f.get("BD") == new_bd);
      CHECK(f.host.baud() == f.dev_baud);
    }
    CHECK((f.wr_count - wr0) == (differs ? 1 : 0));
    CHECK(f.ac_count == (differs ? 1 : 0));
    CHECK(!f.in_cmd);
    f.reboot();
    for (unsigned w = 0; w < 4; ++w)
      if (have[w]) CHECK(f.get(kN[w]) == want[w]);  // survives a power cycle
  }
}

int main() {
  test_helpers();
  test_profile_basic();
  test_profile_nothing_to_do();
  test_command_mode_tracking();
  test_baud_change();
  test_find_baud();
  test_no_radio();
  test_command_errors();
  test_bad_arguments();
  test_singles();
  test_wide_values();
  test_time_wrap();
  test_unsolicited_passthrough();
  test_fuzz();

  printf("%lu checks, %lu failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
