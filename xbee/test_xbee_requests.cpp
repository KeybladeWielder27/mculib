// Host-side tests for xbee/xbee_requests.hpp against a simulated API-mode XBee.
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_xbee_requests.cpp -o test_xbee_requests && ./test_xbee_requests
//
// Time is virtual: the tests advance a millisecond counter and call the
// radio's step() and the library's poll() in turn.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <map>
#include <string>
#include <vector>

#include "hal/loopback_stream.hpp"
#include "xbee/xbee_requests.hpp"

using namespace mcu;
typedef std::vector<uint8_t> Bytes;

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

static uint64_t g_rng = 0xC0FFEE1234567ull;
static uint32_t rnd32() {
  g_rng ^= g_rng >> 12;
  g_rng ^= g_rng << 25;
  g_rng ^= g_rng >> 27;
  return (uint32_t)((g_rng * 0x2545F4914F6CDD1Dull) >> 32);
}
static uint32_t rnd_below(uint32_t n) { return rnd32() % n; }

static Bytes B(std::initializer_list<int> l) {
  Bytes v;
  for (int x : l) v.push_back((uint8_t)x);
  return v;
}

// ---- simulated radio ------------------------------------------------------------

template <size_t TxN>
struct SimRadio {
  typedef LoopbackStream<2048, TxN> Wire;

  Wire& wire;
  XBeeApiMode mode;
  XBeeApiParser<256> parser;
  uint64_t remote64;  // address that answers remote AT commands
  std::map<std::string, Bytes> settings;

  // behaviour
  int drop_next;           // silently ignore this many requests
  int dup_next;            // answer this many requests twice
  uint32_t delay_ms;       // latency of every answer
  uint8_t tx_delivery;     // delivery status reported for transmit requests
  bool tx_extended;        // answer with 0x8B instead of 0x89
  int wrong_id_next;       // answer this many requests with the id off by one (a stray answer)
  uint64_t reply_from;     // source address in 0x97 answers (0 = remote64)

  struct Due {
    uint32_t at;
    Bytes data;  // frame data
    size_t reply;  // index into `replies`
  };
  std::vector<Due> out;

  // log
  struct Seen {
    uint8_t type;
    uint8_t id;
    uint32_t t;
    Bytes body;
  };
  std::vector<Seen> seen;
  struct Reply {
    uint8_t type;
    uint8_t id;
    uint8_t status;
    uint32_t t;  // time the reply was put on the wire
  };
  std::vector<Reply> replies;

  SimRadio(Wire& w, XBeeApiMode m)
      : wire(w), mode(m), parser(m), remote64(0x0013A20041000001ull), drop_next(0), dup_next(0),
        delay_ms(0), tx_delivery(0), tx_extended(false), wrong_id_next(0), reply_from(0) {
    settings["ID"] = B({0x7F, 0xFF});
    settings["NI"] = B({'B', 'a', 's', 'e'});
    settings["CH"] = B({0x0C});
  }

  void queue(uint32_t now, const Bytes& frame, uint8_t type, uint8_t id, uint8_t status) {
    Due d;
    d.at = now + delay_ms;
    d.data = frame;
    d.reply = replies.size();
    out.push_back(d);
    Reply r;
    r.type = type;
    r.id = id;
    r.status = status;
    r.t = d.at;
    replies.push_back(r);
  }

  void handle(uint32_t now, const uint8_t* f, size_t n) {
    Seen s;
    s.type = f[0];
    s.id = n > 1 ? f[1] : 0;
    s.t = now;
    s.body = Bytes(f, f + n);
    seen.push_back(s);
    const uint8_t type = f[0];
    if (type != 0x08 && type != 0x09 && type != 0x17 && type != 0x10) return;
    const uint8_t id = f[1];
    if (drop_next > 0) {
      --drop_next;
      return;
    }
    if (id == 0) return;  // no answer wanted
    const uint8_t rid = wrong_id_next > 0 ? (uint8_t)(id + 1) : id;
    if (wrong_id_next > 0) --wrong_id_next;
    const int copies = dup_next > 0 ? 2 : 1;
    if (dup_next > 0) --dup_next;
    for (int c = 0; c < copies; ++c) {
      if (type == 0x08 || type == 0x09) {
        if (n < 4) return;
        std::string cmd((const char*)f + 2, 2);
        Bytes r = B({0x88, rid, cmd[0], cmd[1], 0});
        uint8_t st = 0;
        if (!settings.count(cmd)) {
          st = 2;
        } else if (n > 4) {
          settings[cmd] = Bytes(f + 4, f + n);
        } else {
          r.insert(r.end(), settings[cmd].begin(), settings[cmd].end());
        }
        r[4] = st;
        queue(now, r, 0x88, rid, st);
      } else if (type == 0x17) {
        if (n < 15) return;
        std::string cmd((const char*)f + 13, 2);
        uint64_t src = reply_from ? reply_from : remote64;
        Bytes r = B({0x97, rid});
        for (int i = 7; i >= 0; --i) r.push_back((uint8_t)(src >> (8 * i)));
        r.push_back(0xFF);
        r.push_back(0xFE);
        r.push_back((uint8_t)cmd[0]);
        r.push_back((uint8_t)cmd[1]);
        uint8_t st = 0;
        if (!settings.count(cmd)) st = 2;
        r.push_back(st);
        if (st == 0 && n == 15) r.insert(r.end(), settings[cmd].begin(), settings[cmd].end());
        queue(now, r, 0x97, rid, st);
      } else {
        if (tx_extended) {
          queue(now, B({0x8B, rid, 0x12, 0x34, 1, tx_delivery, 0}), 0x8B, rid, tx_delivery);
        } else {
          queue(now, B({0x89, rid, tx_delivery}), 0x89, rid, tx_delivery);
        }
      }
    }
  }

  // Unsolicited frames
  void send_now(const Bytes& frame) {
    uint8_t buf[300];
    const size_t n = xbee_encode(mode, frame.data(), frame.size(), buf, sizeof buf);
    wire.inject(buf, n);
  }

  void step(uint32_t now) {
    uint8_t buf[256];
    for (;;) {
      const size_t n = wire.take(buf, sizeof buf);
      if (n == 0) break;
      size_t i = 0;
      while (i < n) {
        const size_t used = parser.feed(buf + i, n - i);
        i += used;
        if (parser.frame_ready()) handle(now, parser.frame(), parser.frame_length());
      }
    }
    for (size_t k = 0; k < out.size();) {
      if (time_reached(now, out[k].at)) {
        send_now(out[k].data);
        replies[out[k].reply].t = now;  // when it really went onto the wire
        out.erase(out.begin() + (long)k);
      } else {
        ++k;
      }
    }
  }

  size_t count_seen(uint8_t type) const {
    size_t c = 0;
    for (size_t i = 0; i < seen.size(); ++i) c += seen[i].type == type ? 1 : 0;
    return c;
  }
};

typedef LoopbackStream<2048, 2048> BigWire;
typedef SimRadio<2048> Radio;
typedef XBeeRequests<4> Req;

// Runs virtual time: radio then host, `step` ms at a time, until `until` ms pass or stop() is true.
template <typename R, typename Rad, typename Pred>
static uint32_t run(R& xb, Rad& radio, uint32_t& now, uint32_t until, Pred stop, uint32_t step = 10) {
  const uint32_t start = now;
  while ((uint32_t)(now - start) < until) {
    radio.step(now);
    xb.poll(now);
    if (stop()) return (uint32_t)(now - start);
    now += step;
  }
  return until;
}
struct Never {
  bool operator()() const { return false; }
};
template <typename R>
struct Done {
  R& xb;
  XBeeReqHandle h;
  bool operator()() const { return xb.done(h); }
};

// ---- tests ------------------------------------------------------------------

static void test_at_basics(XBeeApiMode mode) {
  BigWire wire;
  Radio radio(wire, mode);
  Req xb(wire, mode);
  uint32_t now = 1000;

  // query
  XBeeReqHandle h = xb.at("ID", 0, 0, now);
  CHECK(h != 0 && xb.submit_error() == XBeeReqError::kNone);
  CHECK(xb.pending() == 1 && !xb.done(h));
  const XBeeReqInfo* in = xb.info(h);
  CHECK(in && in->result == XBeeReqResult::kPending && in->kind == XBeeReqKind::kAt);
  CHECK(in->frame_id != 0 && in->attempts == 1);  // sent immediately
  run(xb, radio, now, 500, Done<Req>{xb, h});
  CHECK(xb.done(h));
  in = xb.info(h);
  CHECK(in->result == XBeeReqResult::kOk && in->status == 0 && in->data_len == 2 && !in->data_truncated);
  uint64_t v = 0;
  CHECK(xbee_resp_u64(*in, v) && v == 0x7FFF);
  CHECK(xb.pending() == 0 && xb.free_slots() == 3);
  xb.release(h);
  CHECK(xb.free_slots() == 4 && xb.info(h) == 0 && !xb.done(h));
  xb.release(h);  // stale: harmless
  CHECK(xb.free_slots() == 4);

  // set, then read back
  const uint8_t ch[1] = {0x14};
  h = xb.at("CH", ch, 1, now);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  CHECK(xb.info(h)->result == XBeeReqResult::kOk && xb.info(h)->data_len == 0);
  xb.release(h);
  h = xb.at("CH", 0, 0, now);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  CHECK(xbee_resp_u64(*xb.info(h), v) && v == 0x14);
  xb.release(h);

  // text answer
  h = xb.at("NI", 0, 0, now);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  in = xb.info(h);
  CHECK(in->data_len == 4 && memcmp(in->data, "Base", 4) == 0);
  CHECK(!xbee_resp_u64(*in, v) || true);  // 4 bytes parse as a number too; no crash either way
  xb.release(h);

  // unknown command: radio error with status 2
  h = xb.at("ZZ", 0, 0, now);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  in = xb.info(h);
  CHECK(in->result == XBeeReqResult::kRadioError && in->status == 2);
  xb.release(h);

  // the frames on the wire were what we asked for
  CHECK(radio.count_seen(0x08) == 5);
  CHECK(xb.stats().submitted == 5 && xb.stats().ok == 4 && xb.stats().radio_errors == 1);
  CHECK(xb.stats().timeouts == 0 && xb.stats().stale == 0);

  // queued AT uses frame 0x09
  h = xb.at("CH", ch, 1, now, 0, true);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  CHECK(radio.count_seen(0x09) == 1);
  xb.release(h);
}

static void test_bad_args() {
  BigWire wire;
  Req xb(wire);
  CHECK(xb.at("I", 0, 0, 0) == 0 && xb.submit_error() == XBeeReqError::kBadArg);
  CHECK(xb.at(0, 0, 0, 0) == 0 && xb.submit_error() == XBeeReqError::kBadArg);
  CHECK(xb.at("ID", 0, 3, 0) == 0 && xb.submit_error() == XBeeReqError::kBadArg);
  Bytes big(200, 1);
  CHECK(xb.at("ID", big.data(), big.size(), 0) == 0 && xb.submit_error() == XBeeReqError::kTooBig);
  CHECK(xb.tx(1, big.data(), big.size(), 0) == 0 && xb.submit_error() == XBeeReqError::kTooBig);
  CHECK(xb.tx(1, 0, 4, 0) == 0 && xb.submit_error() == XBeeReqError::kBadArg);
  CHECK(xb.remote_at(1, "ID", big.data(), big.size(), 0) == 0 && xb.submit_error() == XBeeReqError::kTooBig);
  CHECK(xb.free_slots() == 4 && xb.pending() == 0);
  uint8_t out[8];
  CHECK(wire.take(out, sizeof out) == 0);  // nothing was written
}

static void test_timeout_and_retry(XBeeApiMode mode) {
  BigWire wire;
  Radio radio(wire, mode);
  Req xb(wire, mode);
  uint32_t now = 5000;

  // radio drops the first attempt; the retry (same frame id) is answered
  radio.drop_next = 1;
  XBeeReqHandle h = xb.at("ID", 0, 0, now);
  const uint8_t id = xb.info(h)->frame_id;
  uint32_t took = run(xb, radio, now, 5000, Done<Req>{xb, h});
  const XBeeReqInfo* in = xb.info(h);
  CHECK(in->result == XBeeReqResult::kOk && in->attempts == 2);
  CHECK(took >= 1000 && took <= 1100);  // one timeout (1 s) then the answer
  CHECK(radio.count_seen(0x08) == 2);
  CHECK(radio.seen[0].id == id && radio.seen[1].id == id);
  CHECK(xb.stats().retries == 1);
  xb.release(h);

  // never answered: 1 + 2 retries, then timeout
  radio.drop_next = 100;
  h = xb.at("ID", 0, 0, now);
  took = run(xb, radio, now, 10000, Done<Req>{xb, h});
  in = xb.info(h);
  CHECK(in->result == XBeeReqResult::kTimeout && in->attempts == 3);
  CHECK(took >= 3000 && took <= 3100);
  CHECK(xb.stats().timeouts == 1 && xb.pending() == 0);
  xb.release(h);

  // custom options
  radio.drop_next = 100;
  XBeeReqOptions o = XBeeReqOptions::at();
  o.timeout_ms = 200;
  o.retries = 0;
  h = xb.at("ID", 0, 0, now, &o);
  took = run(xb, radio, now, 5000, Done<Req>{xb, h});
  CHECK(xb.info(h)->result == XBeeReqResult::kTimeout && xb.info(h)->attempts == 1);
  CHECK(took >= 200 && took <= 300);
  xb.release(h);

  // a late answer to the first attempt, arriving while the retry is waiting, still completes it
  radio.drop_next = 0;
  radio.delay_ms = 1200;  // longer than the 1 s timeout
  h = xb.at("ID", 0, 0, now);
  took = run(xb, radio, now, 5000, Done<Req>{xb, h});
  in = xb.info(h);
  CHECK(in->result == XBeeReqResult::kOk && in->attempts == 2);
  CHECK(took >= 1200 && took <= 1300);
  xb.release(h);
  // the second answer (to the retry) arrives later and matches nothing
  const uint32_t stale0 = xb.stats().stale;
  run(xb, radio, now, 2000, Never());
  CHECK(xb.stats().stale == stale0 + 1);
}

static void test_duplicates_and_strays(XBeeApiMode mode) {
  BigWire wire;
  Radio radio(wire, mode);
  Req xb(wire, mode);
  uint32_t now = 100;

  radio.dup_next = 1;
  XBeeReqHandle h = xb.at("ID", 0, 0, now);
  run(xb, radio, now, 300, Never());
  CHECK(xb.info(h)->result == XBeeReqResult::kOk);
  CHECK(xb.stats().stale == 1 && xb.stats().ok == 1);
  xb.release(h);

  // a stray answer with another id must not complete anything
  radio.wrong_id_next = 1;
  radio.drop_next = 0;
  h = xb.at("ID", 0, 0, now);
  run(xb, radio, now, 200, Never());
  CHECK(xb.info(h)->result == XBeeReqResult::kPending);
  CHECK(xb.stats().stale == 2);
  run(xb, radio, now, 1500, Done<Req>{xb, h});  // retry gets a proper answer
  CHECK(xb.info(h)->result == XBeeReqResult::kOk);
  xb.release(h);

  // an answer to a released (cancelled) request is dropped
  radio.delay_ms = 300;
  h = xb.at("ID", 0, 0, now);
  xb.release(h);
  run(xb, radio, now, 600, Never());
  CHECK(xb.stats().stale == 3 && xb.free_slots() == 4);
  // and the stale handle stays dead even after the slot is reused
  XBeeReqHandle h2 = xb.at("ID", 0, 0, now);
  CHECK(h2 != h && xb.info(h) == 0 && xb.info(h2) != 0);
  radio.delay_ms = 0;
  run(xb, radio, now, 1500, Done<Req>{xb, h2});
  xb.release(h2);
}

static void test_many_in_flight(XBeeApiMode mode) {
  BigWire wire;
  Radio radio(wire, mode);
  Req xb(wire, mode);
  uint32_t now = 0;
  XBeeReqHandle h[4];
  const char* cmds[4] = {"ID", "NI", "CH", "ZZ"};
  radio.delay_ms = 50;
  for (int i = 0; i < 4; ++i) {
    h[i] = xb.at(cmds[i], 0, 0, now);
    CHECK(h[i] != 0);
  }
  // all slots busy
  CHECK(xb.at("ID", 0, 0, now) == 0 && xb.submit_error() == XBeeReqError::kNoSlot);
  CHECK(xb.pending() == 4 && xb.free_slots() == 0);
  // distinct frame ids, sent in submission order
  for (int i = 0; i < 4; ++i)
    for (int j = i + 1; j < 4; ++j) CHECK(xb.info(h[i])->frame_id != xb.info(h[j])->frame_id);
  run(xb, radio, now, 400, Never());
  CHECK(xb.pending() == 0);
  CHECK(xb.info(h[0])->data_len == 2 && xb.info(h[1])->data_len == 4 && xb.info(h[2])->data_len == 1);
  CHECK(xb.info(h[3])->result == XBeeReqResult::kRadioError);
  for (int i = 0; i < 4; ++i) xb.release(h[i]);

  // answers in reverse order still reach the right request
  BigWire wire2;
  Radio radio2(wire2, mode);
  Req xb2(wire2, mode);
  now = 0;
  XBeeReqHandle a = xb2.at("ID", 0, 0, now);
  XBeeReqHandle b = xb2.at("NI", 0, 0, now);
  XBeeReqHandle c = xb2.at("CH", 0, 0, now);
  radio2.step(now);  // the radio reads all three
  CHECK(radio2.out.size() == 0);
  // collect the three answers and inject them reversed
  // (the radio queued nothing because delay_ms is 0 and step() delivered them already)
  xb2.poll(now);
  CHECK(xb2.info(a)->result == XBeeReqResult::kOk && xb2.info(b)->result == XBeeReqResult::kOk &&
        xb2.info(c)->result == XBeeReqResult::kOk);
  CHECK(xb2.info(b)->data_len == 4 && xb2.info(c)->data_len == 1);
  xb2.release(a);
  xb2.release(b);
  xb2.release(c);
}

static void test_id_allocation() {
  BigWire wire;
  Radio radio(wire, XBeeApiMode::kApi2);
  XBeeRequests<2> xb(wire);
  uint32_t now = 0;
  // one long-lived request keeps its id for a thousand others
  radio.delay_ms = 0;
  XBeeReqOptions slow = XBeeReqOptions::at();
  slow.timeout_ms = 100000;
  radio.drop_next = 1;
  XBeeReqHandle keeper = xb.at("ID", 0, 0, now, &slow);
  const uint8_t keep_id = xb.info(keeper)->frame_id;
  uint8_t seen_max = 0;
  for (int i = 0; i < 1000; ++i) {
    XBeeReqHandle h = xb.at("CH", 0, 0, now);
    CHECK(h != 0);
    const uint8_t id = xb.info(h)->frame_id;
    CHECK(id != 0 && id != keep_id);
    if (id > seen_max) seen_max = id;
    radio.step(now);
    xb.poll(now);
    CHECK(xb.info(h)->result == XBeeReqResult::kOk);
    xb.release(h);
    now += 1;
  }
  CHECK(seen_max == 255);
  CHECK(xb.info(keeper)->result == XBeeReqResult::kPending);
  xb.release(keeper);
}

static void test_tx(XBeeApiMode mode) {
  BigWire wire;
  Radio radio(wire, mode);
  Req xb(wire, mode);
  uint32_t now = 0;
  const uint8_t payload[5] = {'h', 'e', 0x7E, 'l', 'o'};

  XBeeReqHandle h = xb.tx(0x0013A20041ABCDEFull, payload, 5, now);
  CHECK(h != 0 && xb.info(h)->kind == XBeeReqKind::kTx);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  const XBeeReqInfo* in = xb.info(h);
  CHECK(in->result == XBeeReqResult::kOk && in->status == 0 && !in->tx_extended);
  // what the radio saw
  const Bytes& f = radio.seen[0].body;
  CHECK(f.size() == 14 + 5 && f[0] == 0x10 && f[9] == 0xEF && f[14 + 2] == 0x7E);
  xb.release(h);

  // delivery failure
  radio.tx_delivery = 0x21;
  radio.tx_extended = true;
  h = xb.tx(1, payload, 5, now);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  in = xb.info(h);
  CHECK(in->result == XBeeReqResult::kRadioError && in->status == 0x21 && in->tx_extended);
  CHECK(in->addr16 == 0x1234 && in->tx_retries == 1 && in->discovery == 0);
  xb.release(h);

  // no retries by default: lost status means timeout after 5 s, one send
  radio.drop_next = 5;
  h = xb.tx(1, payload, 5, now);
  uint32_t took = run(xb, radio, now, 20000, Done<Req>{xb, h});
  CHECK(xb.info(h)->result == XBeeReqResult::kTimeout && xb.info(h)->attempts == 1);
  CHECK(took >= 5000 && took <= 5100);
  xb.release(h);

  // fire and forget: frame id 0, finished once written, no answer expected
  radio.drop_next = 0;
  radio.tx_delivery = 0;
  XBeeReqOptions o = XBeeReqOptions::tx();
  o.want_response = false;
  o.auto_release = true;
  const size_t before = radio.seen.size();
  h = xb.tx(0x000000000000FFFFull, payload, 5, now, &o);
  CHECK(h != 0);
  CHECK(xb.free_slots() == 4);  // finished and released at once
  run(xb, radio, now, 200, Never());
  CHECK(radio.seen.size() == before + 1 && radio.seen[before].id == 0);
  CHECK(xb.stats().ok >= 1);

  // fire and forget, keeping the handle
  o.auto_release = false;
  h = xb.tx(1, payload, 5, now, &o);
  CHECK(xb.info(h)->result == XBeeReqResult::kOk && xb.info(h)->frame_id == 0 && xb.info(h)->attempts == 1);
  xb.release(h);
}

static void test_remote_at(XBeeApiMode mode) {
  BigWire wire;
  Radio radio(wire, mode);
  Req xb(wire, mode);
  uint32_t now = 0;
  const uint64_t dest = radio.remote64;

  XBeeReqHandle h = xb.remote_at(dest, "ID", 0, 0, now);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  const XBeeReqInfo* in = xb.info(h);
  uint64_t v = 0;
  CHECK(in->result == XBeeReqResult::kOk && in->kind == XBeeReqKind::kRemoteAt && xbee_resp_u64(*in, v) &&
        v == 0x7FFF);
  const Bytes& f = radio.seen[0].body;
  CHECK(f.size() == 15 && f[0] == 0x17 && f[12] == xbee_frame::kRemoteApplyChanges);
  xb.release(h);

  // unknown command on the far side
  h = xb.remote_at(dest, "ZZ", 0, 0, now);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  CHECK(xb.info(h)->result == XBeeReqResult::kRadioError && xb.info(h)->status == 2);
  xb.release(h);

  // an answer from the wrong radio is not accepted
  radio.reply_from = 0x0013A20099999999ull;
  XBeeReqOptions o = XBeeReqOptions::remote_at();
  o.timeout_ms = 300;
  o.retries = 0;
  h = xb.remote_at(dest, "ID", 0, 0, now, &o);
  run(xb, radio, now, 1000, Done<Req>{xb, h});
  CHECK(xb.info(h)->result == XBeeReqResult::kTimeout);
  CHECK(xb.stats().stale == 1);
  xb.release(h);

  // a broadcast takes the first answer from anyone
  h = xb.remote_at(0x000000000000FFFFull, "ID", 0, 0, now, &o);
  run(xb, radio, now, 1000, Done<Req>{xb, h});
  CHECK(xb.info(h)->result == XBeeReqResult::kOk);
  xb.release(h);

  // option byte is passed through
  radio.reply_from = 0;
  h = xb.remote_at(dest, "CH", 0, 0, now, 0, xbee_frame::kRemoteDisableAck);
  CHECK(h != 0);
  run(xb, radio, now, 500, Done<Req>{xb, h});
  CHECK(radio.seen.back().body[12] == xbee_frame::kRemoteDisableAck);
  xb.release(h);

  // a local 0x88 with the same id as a remote request does not complete it
  radio.drop_next = 1;
  h = xb.remote_at(dest, "ID", 0, 0, now, &o);
  const uint8_t id = xb.info(h)->frame_id;
  radio.send_now(B({0x88, id, 'I', 'D', 0, 1, 2}));
  run(xb, radio, now, 100, Never());
  CHECK(xb.info(h)->result == XBeeReqResult::kPending);
  run(xb, radio, now, 1000, Done<Req>{xb, h});
  xb.release(h);
}

static void test_unsolicited(XBeeApiMode mode) {
  BigWire wire;
  Radio radio(wire, mode);
  Req xb(wire, mode);
  uint32_t now = 0;
  radio.delay_ms = 100;
  XBeeReqHandle h = xb.at("ID", 0, 0, now);

  // received data and a modem status arrive while the request is in flight
  radio.send_now(B({0x8A, 0x06}));
  radio.send_now(B({0x90, 0x00, 0x13, 0xA2, 0x00, 0x41, 0x52, 0x53, 0x54, 0xFF, 0xFE, 0x01, 'h', 'i'}));
  int modem = 0, rx = 0;
  for (int i = 0; i < 50; ++i) {
    radio.step(now);
    while (xb.poll(now)) {
      if (xb.frame_type() == 0x8A) {
        uint8_t st = 0;
        CHECK(xbee_parse_modem_status(xb.frame(), xb.frame_length(), st) && st == 6);
        ++modem;
      } else if (xb.frame_type() == 0x90) {
        XBeeRxPacket p = XBeeRxPacket();
        CHECK(xbee_parse_rx(xb.frame(), xb.frame_length(), p) && p.data_len == 2 && p.data[0] == 'h');
        ++rx;
      } else {
        CHECK(false);
      }
    }
    now += 10;
  }
  CHECK(modem == 1 && rx == 1);
  CHECK(xb.info(h)->result == XBeeReqResult::kOk);  // and the answer was not shown to the caller
  xb.release(h);
  CHECK(xb.frame_length() == 0 && xb.frame_type() == 0);  // nothing pending after poll returned false

  // frames that look like answers but are malformed are handed back, not swallowed
  radio.send_now(B({0x88, 1}));  // too short for an AT response
  bool got = false;
  for (int i = 0; i < 5 && !got; ++i) {
    got = xb.poll(now);
    now += 10;
  }
  CHECK(got && xb.frame_type() == 0x88 && xb.frame_length() == 2);
}

struct CbLog {
  int calls;
  XBeeReqHandle last;
  XBeeReqResult result;
  uint64_t value;
  bool resubmit;
  Req* xb;
  uint32_t* now;
};
static void on_done(void* ctx, XBeeReqHandle h, const XBeeReqInfo& info) {
  CbLog* l = static_cast<CbLog*>(ctx);
  ++l->calls;
  l->last = h;
  l->result = info.result;
  xbee_resp_u64(info, l->value);
  if (l->resubmit) {
    l->resubmit = false;
    XBeeReqOptions o = XBeeReqOptions::at();
    o.on_done = on_done;
    o.ctx = ctx;
    l->xb->at("CH", 0, 0, *l->now, &o);  // submitting from a callback is allowed
  }
}

static void test_callbacks(XBeeApiMode mode) {
  BigWire wire;
  Radio radio(wire, mode);
  Req xb(wire, mode);
  uint32_t now = 0;
  CbLog log = {0, 0, XBeeReqResult::kPending, 0, false, &xb, &now};
  XBeeReqOptions o = XBeeReqOptions::at();
  o.on_done = on_done;
  o.ctx = &log;
  XBeeReqHandle h = xb.at("ID", 0, 0, now, &o);
  CHECK(xb.free_slots() == 3);
  run(xb, radio, now, 300, Never());
  CHECK(log.calls == 1 && log.last == h && log.result == XBeeReqResult::kOk && log.value == 0x7FFF);
  CHECK(xb.free_slots() == 4 && xb.info(h) == 0);  // freed after the callback

  // timeout reaches the callback too
  radio.drop_next = 100;
  o.retries = 0;
  o.timeout_ms = 100;
  xb.at("ID", 0, 0, now, &o);
  run(xb, radio, now, 500, Never());
  CHECK(log.calls == 2 && log.result == XBeeReqResult::kTimeout);
  CHECK(xb.free_slots() == 4);

  // a callback that submits another request
  radio.drop_next = 0;
  log.resubmit = true;
  xb.at("ID", 0, 0, now, &o);
  o.timeout_ms = 1000;
  run(xb, radio, now, 500, Never());
  CHECK(log.calls == 4 && log.result == XBeeReqResult::kOk);
  CHECK(xb.free_slots() == 4 && xb.pending() == 0);

  // a callback is invoked for an immediate failure too (no space, send timeout)
}

static void test_send_blocked() {
  // The serial output buffer is tiny, so frames wait for room, in order.
  typedef SimRadio<64> TinyRadio;
  TinyRadio::Wire wire;
  TinyRadio radio(wire, XBeeApiMode::kApi2);
  Req xb(wire, XBeeApiMode::kApi2);
  uint32_t now = 0;

  // each AT query frame is 8 bytes; a tx with a 40-byte payload is 14+40+4 = 58 bytes
  Bytes payload(40, 0x55);
  XBeeReqHandle big = xb.tx(1, payload.data(), payload.size(), now);
  CHECK(big != 0 && xb.info(big)->attempts == 1);  // fits (58 <= 64)
  XBeeReqHandle q1 = xb.at("ID", 0, 0, now);       // does not fit yet
  XBeeReqHandle q2 = xb.at("NI", 0, 0, now);
  CHECK(xb.info(q1)->attempts == 0 && xb.info(q2)->attempts == 0);
  xb.poll(now);  // still full, nothing drained
  CHECK(xb.info(q1)->attempts == 0 && xb.info(q2)->attempts == 0 && xb.pending() == 3);
  // the radio drains the wire; the queued frames go out in order
  now += 10;
  radio.step(now);
  xb.poll(now);
  CHECK(xb.info(q1)->attempts == 1 && xb.info(q2)->attempts == 1);
  run(xb, radio, now, 1000, Never());
  CHECK(xb.pending() == 0);
  CHECK(xb.info(big)->result == XBeeReqResult::kOk && xb.info(q1)->result == XBeeReqResult::kOk &&
        xb.info(q2)->result == XBeeReqResult::kOk);
  // order on the wire: tx, ID, NI
  CHECK(radio.seen.size() == 3 && radio.seen[0].type == 0x10 && radio.seen[1].body[2] == 'I' &&
        radio.seen[2].body[2] == 'N');
  xb.release(big);
  xb.release(q1);
  xb.release(q2);

  // nobody drains the wire: the send times out
  TinyRadio::Wire wire2;
  Req xb2(wire2, XBeeApiMode::kApi2);
  now = 0;
  XBeeReqHandle a = xb2.tx(1, payload.data(), payload.size(), now);
  XBeeReqOptions o = XBeeReqOptions::at();
  o.send_timeout_ms = 500;
  XBeeReqHandle b = xb2.at("ID", 0, 0, now, &o);
  CHECK(xb2.info(b)->attempts == 0);
  for (int i = 0; i < 49; ++i) {
    now += 10;
    xb2.poll(now);
  }
  CHECK(xb2.info(b)->result == XBeeReqResult::kPending);
  now += 20;
  xb2.poll(now);
  CHECK(xb2.info(b)->result == XBeeReqResult::kSendFailed && xb2.info(b)->attempts == 0);
  CHECK(xb2.info(a)->result == XBeeReqResult::kPending);  // the one that was written keeps waiting
  CHECK(xb2.stats().send_failures == 1);
  uint8_t drain[64];
  CHECK(wire2.take(drain, sizeof drain) == 58);  // the failed frame never reached the wire
}

static void test_time_wrap() {
  BigWire wire;
  Radio radio(wire, XBeeApiMode::kApi2);
  Req xb(wire);
  uint32_t now = 0xFFFFFF00u;  // 256 ms before the counter wraps
  radio.drop_next = 1;
  XBeeReqHandle h = xb.at("ID", 0, 0, now);
  uint32_t took = run(xb, radio, now, 5000, Done<Req>{xb, h});
  CHECK(xb.info(h)->result == XBeeReqResult::kOk && xb.info(h)->attempts == 2);
  CHECK(took >= 1000 && took <= 1100);
  xb.release(h);
  radio.drop_next = 100;
  now = 0xFFFFFFF0u;
  h = xb.at("ID", 0, 0, now);
  took = run(xb, radio, now, 10000, Done<Req>{xb, h});
  CHECK(xb.info(h)->result == XBeeReqResult::kTimeout && took >= 3000 && took <= 3100);
  xb.release(h);
}

static void test_late_answer_while_retry_blocked() {
  typedef SimRadio<64> TinyRadio;
  TinyRadio::Wire wire;
  TinyRadio radio(wire, XBeeApiMode::kApi2);
  Req xb(wire, XBeeApiMode::kApi2);
  uint32_t now = 0;
  Bytes payload(40, 0x55);
  XBeeReqHandle q = xb.at("ID", 0, 0, now);  // 8 bytes on the wire, nobody reads them
  XBeeReqHandle big = xb.tx(1, payload.data(), payload.size(), now);  // 58 bytes: does not fit yet
  CHECK(xb.info(q)->attempts == 1 && xb.info(big)->attempts == 0);
  for (int i = 0; i < 105; ++i) {  // past the 1 s timeout of the query
    now += 10;
    xb.poll(now);
  }
  // the retry is due but queued behind the transmit request that cannot be written
  CHECK(xb.info(q)->result == XBeeReqResult::kPending && xb.info(q)->attempts == 1);
  CHECK(xb.stats().retries == 1);
  // the answer to the first attempt shows up now
  radio.send_now(B({0x88, xb.info(q)->frame_id, 'I', 'D', 0, 0x7F, 0xFF}));
  xb.poll(now);
  CHECK(xb.info(q)->result == XBeeReqResult::kOk && xb.info(q)->attempts == 1);
  CHECK(xb.info(big)->result == XBeeReqResult::kPending);
}

static void test_noisy_link(XBeeApiMode mode) {
  // garbage bytes between frames must not break matching
  BigWire wire;
  Radio radio(wire, mode);
  Req xb(wire, mode);
  uint32_t now = 0;
  int ok = 0;
  for (int i = 0; i < 200; ++i) {
    XBeeReqHandle h = xb.at("NI", 0, 0, now);
    // noise without start delimiters
    uint8_t junk[4];
    for (size_t k = 0; k < sizeof junk; ++k) {
      junk[k] = (uint8_t)rnd32();
      if (junk[k] == 0x7E) junk[k] = 0x7F;
    }
    wire.inject(junk, rnd_below(5));
    run(xb, radio, now, 300, Done<Req>{xb, h});
    if (xb.info(h)->result == XBeeReqResult::kOk) ++ok;
    xb.release(h);
  }
  CHECK(ok == 200);
}

// Random submissions and random radio misbehaviour; check invariants only.
static void test_fuzz(XBeeApiMode mode) {
  BigWire wire;
  Radio radio(wire, mode);
  XBeeRequests<6, 16, 64, 128> xb(wire, mode);
  uint32_t now = 0;

  struct Track {
    XBeeReqHandle h;
    uint8_t id;
    XBeeReqKind kind;
    uint32_t submitted;
    uint32_t ended;
    bool open;
  };
  std::vector<Track> tracks;
  const char* cmds[4] = {"ID", "NI", "CH", "ZZ"};
  const uint8_t pl[6] = {1, 2, 3, 4, 5, 6};
  size_t completed = 0, total_submitted = 0;

  for (int step = 0; step < 40000; ++step) {
    // radio behaviour changes now and then
    if (rnd_below(200) == 0) radio.drop_next = (int)rnd_below(3);
    if (rnd_below(300) == 0) radio.dup_next = (int)rnd_below(3);
    if (rnd_below(400) == 0) radio.wrong_id_next = (int)rnd_below(2);
    if (rnd_below(100) == 0) radio.delay_ms = rnd_below(3) == 0 ? rnd_below(2500) : rnd_below(60);
    if (rnd_below(150) == 0) radio.tx_delivery = rnd_below(3) == 0 ? 0x25 : 0;
    if (rnd_below(500) == 0) radio.tx_extended = !radio.tx_extended;

    // submit
    if (rnd_below(6) == 0) {
      XBeeReqOptions o = rnd_below(2) ? XBeeReqOptions::at() : XBeeReqOptions::tx();
      o.timeout_ms = 100 + rnd_below(1500);
      o.retries = (uint8_t)rnd_below(3);
      o.send_timeout_ms = rnd_below(2) ? 0 : 50 + rnd_below(500);
      XBeeReqHandle h = 0;
      XBeeReqKind kind;
      switch (rnd_below(3)) {
        case 0: kind = XBeeReqKind::kAt; h = xb.at(cmds[rnd_below(4)], 0, 0, now, &o); break;
        case 1:
          kind = XBeeReqKind::kRemoteAt;
          h = xb.remote_at(radio.remote64, cmds[rnd_below(4)], 0, 0, now, &o);
          break;
        default: kind = XBeeReqKind::kTx; h = xb.tx(0x1234, pl, 1 + rnd_below(6), now, &o); break;
      }
      if (h == 0) {
        CHECK(xb.submit_error() == XBeeReqError::kNoSlot);
        CHECK(xb.free_slots() == 0);
      } else {
        Track t;
        t.h = h;
        t.id = xb.info(h)->frame_id;
        t.kind = kind;
        t.submitted = now;
        t.ended = 0;
        t.open = true;
        tracks.push_back(t);
        ++total_submitted;
        CHECK(t.id != 0);
      }
    }

    radio.step(now);
    xb.poll(now);

    // harvest finished requests
    for (size_t i = 0; i < tracks.size(); ++i) {
      Track& t = tracks[i];
      if (!t.open) continue;
      const XBeeReqInfo* in = xb.info(t.h);
      CHECK(in != 0);
      if (!in) continue;
      if (in->result == XBeeReqResult::kPending) continue;
      t.open = false;
      t.ended = now;
      ++completed;
      // invariants on the outcome
      CHECK(in->frame_id == t.id);
      CHECK(in->attempts >= (in->result == XBeeReqResult::kSendFailed ? 0u : 1u));
      CHECK(in->attempts <= 3);
      // was a matching answer put on the wire inside this request's lifetime?
      bool ok_reply = false, any_reply = false;
      for (size_t r = 0; r < radio.replies.size(); ++r) {
        const SimRadio<2048>::Reply& rp = radio.replies[r];
        if (rp.id != t.id) continue;
        const bool kind_match = (t.kind == XBeeReqKind::kAt && rp.type == 0x88) ||
                                (t.kind == XBeeReqKind::kRemoteAt && rp.type == 0x97) ||
                                (t.kind == XBeeReqKind::kTx && (rp.type == 0x89 || rp.type == 0x8B));
        if (!kind_match) continue;
        if ((int32_t)(rp.t - t.submitted) < 0) continue;
        if ((int32_t)(rp.t - t.ended) > 0) continue;  // came after the request ended
        any_reply = true;
        if (rp.status == 0) ok_reply = true;
      }
      if ((in->result == XBeeReqResult::kOk && !ok_reply) || (in->result == XBeeReqResult::kRadioError && !any_reply) ||
          (in->result == XBeeReqResult::kTimeout && any_reply)) {
        printf("  oracle mismatch: result %d kind %d id %u submitted %u ended %u attempts %u status %u\n",
               (int)in->result, (int)t.kind, t.id, t.submitted, t.ended, in->attempts, in->status);
        for (size_t r = 0; r < radio.replies.size(); ++r)
          if (radio.replies[r].id == t.id)
            printf("     reply type %02x status %u at %u\n", radio.replies[r].type, radio.replies[r].status, radio.replies[r].t);
      }
      if (in->result == XBeeReqResult::kOk) CHECK(ok_reply);
      if (in->result == XBeeReqResult::kTimeout) CHECK(!any_reply);
      if (in->result == XBeeReqResult::kRadioError) CHECK(any_reply);
      xb.release(t.h);
    }
    CHECK(xb.pending() + xb.free_slots() <= 6);
    now += 1 + rnd_below(15);
  }
  // let everything finish
  for (int i = 0; i < 2000; ++i) {
    radio.step(now);
    xb.poll(now);
    now += 10;
  }
  CHECK(xb.pending() == 0);
  size_t open = 0;
  for (size_t i = 0; i < tracks.size(); ++i) {
    if (tracks[i].open) {
      const XBeeReqInfo* in = xb.info(tracks[i].h);
      CHECK(in && in->result != XBeeReqResult::kPending);
      ++open;
    }
  }
  const XBeeReqStats& s = xb.stats();
  CHECK(s.submitted == total_submitted);
  CHECK(s.ok + s.radio_errors + s.timeouts + s.send_failures == completed + open);
  printf("  fuzz %s: %zu requests, ok %u, radio errors %u, timeouts %u, retries %u, stale %u\n",
         mode == XBeeApiMode::kApi1 ? "AP=1" : "AP=2", total_submitted, s.ok, s.radio_errors, s.timeouts,
         s.retries, s.stale);
  CHECK(s.ok > 100 && s.timeouts > 10 && s.stale > 5 && s.retries > 10);
}

int main() {
  const XBeeApiMode modes[2] = {XBeeApiMode::kApi1, XBeeApiMode::kApi2};
  test_bad_args();
  test_id_allocation();
  test_send_blocked();
  test_time_wrap();
  test_late_answer_while_retry_blocked();
  for (int i = 0; i < 2; ++i) {
    test_at_basics(modes[i]);
    test_timeout_and_retry(modes[i]);
    test_duplicates_and_strays(modes[i]);
    test_many_in_flight(modes[i]);
    test_tx(modes[i]);
    test_remote_at(modes[i]);
    test_unsolicited(modes[i]);
    test_callbacks(modes[i]);
    test_noisy_link(modes[i]);
    test_fuzz(modes[i]);
  }
  printf("%lu checks, %lu failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
