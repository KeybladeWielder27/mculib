// Host-side tests for xbee/xbee_api.hpp: frame codec, parser, typed frames, XBeeApi.
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_xbee_api.cpp -o test_xbee_api && ./test_xbee_api

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <vector>

#include "hal/loopback_stream.hpp"
#include "xbee/xbee_api.hpp"

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

static uint64_t g_rng = 0x5EED0A91C0DEBEEFull;
static uint32_t rnd32() {
  g_rng ^= g_rng >> 12;
  g_rng ^= g_rng << 25;
  g_rng ^= g_rng >> 27;
  return (uint32_t)((g_rng * 0x2545F4914F6CDD1Dull) >> 32);
}
static uint32_t rnd_below(uint32_t n) { return rnd32() % n; }

static const XBeeApiMode kModes[2] = {XBeeApiMode::kApi1, XBeeApiMode::kApi2};

// ---- helpers ----------------------------------------------------------------

static Bytes B(std::initializer_list<int> l) {
  Bytes v;
  for (int x : l) v.push_back((uint8_t)x);
  return v;
}

static Bytes encode(XBeeApiMode m, const Bytes& d) {
  Bytes out(xbee_max_encoded_size(m, d.size()));
  size_t n = xbee_encode(m, d.data(), d.size(), out.data(), out.size());
  out.resize(n);
  return out;
}

static Bytes random_frame(size_t maxlen, bool escapey) {
  size_t len = 1 + rnd_below((uint32_t)maxlen);
  Bytes d(len);
  for (size_t i = 0; i < len; ++i) {
    if (escapey && rnd_below(3) == 0) {
      static const uint8_t sp[4] = {0x7E, 0x7D, 0x11, 0x13};
      d[i] = sp[rnd_below(4)];
    } else {
      d[i] = (uint8_t)rnd32();
    }
  }
  return d;
}

// Feeds `in` into a parser with random chunking via feed(); collects frames.
template <size_t N>
static void run_feed(XBeeApiParser<N>& p, const Bytes& in, size_t max_chunk,
                     std::vector<Bytes>& frames) {
  size_t i = 0;
  while (i < in.size()) {
    size_t chunk = 1 + rnd_below((uint32_t)max_chunk);
    if (chunk > in.size() - i) chunk = in.size() - i;
    size_t end = i + chunk;
    while (i < end) {
      size_t used = p.feed(in.data() + i, end - i);
      i += used;
      if (p.frame_ready()) frames.push_back(Bytes(p.frame(), p.frame() + p.frame_length()));
    }
  }
  // a frame can still be pending in the replay buffer
  for (;;) {
    p.feed(0, 0);
    if (!p.frame_ready()) break;
    frames.push_back(Bytes(p.frame(), p.frame() + p.frame_length()));
  }
}

typedef LoopbackStream<4096, 4096> Wire;

template <size_t N>
static void run_poll(XBeeApiParser<N>& p, const Bytes& in, size_t max_chunk,
                     std::vector<Bytes>& frames) {
  Wire w;
  size_t i = 0;
  while (i < in.size()) {
    size_t chunk = 1 + rnd_below((uint32_t)max_chunk);
    if (chunk > in.size() - i) chunk = in.size() - i;
    CHECK(w.inject(in.data() + i, chunk) == chunk);
    i += chunk;
    while (p.poll(w)) frames.push_back(Bytes(p.frame(), p.frame() + p.frame_length()));
  }
}

// ---- tests ------------------------------------------------------------------

static void test_vectors() {
  // Digi documentation examples.
  const uint8_t d1[] = {0x23, 0x11};
  CHECK(xbee_checksum(d1, 2) == 0xCB);
  Bytes e2 = encode(XBeeApiMode::kApi2, B({0x23, 0x11}));
  CHECK(e2 == B({0x7E, 0x00, 0x02, 0x23, 0x7D, 0x31, 0xCB}));
  Bytes e1 = encode(XBeeApiMode::kApi1, B({0x23, 0x11}));
  CHECK(e1 == B({0x7E, 0x00, 0x02, 0x23, 0x11, 0xCB}));
  CHECK(encode(XBeeApiMode::kApi1, B({0x8A, 0x00})) == B({0x7E, 0x00, 0x02, 0x8A, 0x00, 0x75}));
  CHECK(encode(XBeeApiMode::kApi2, B({0x8A, 0x00})) == B({0x7E, 0x00, 0x02, 0x8A, 0x00, 0x75}));
  CHECK(encode(XBeeApiMode::kApi1, B({0x89, 0x52, 0x00})) ==
        B({0x7E, 0x00, 0x03, 0x89, 0x52, 0x00, 0x24}));
  // AT command "NJ" frame id 1 (documented as 7E 00 04 08 01 4E 4A 5F)
  uint8_t f[16];
  size_t n = xbee_build_at(f, sizeof f, 1, "NJ");
  CHECK(n == 4);
  CHECK(encode(XBeeApiMode::kApi1, Bytes(f, f + n)) ==
        B({0x7E, 0x00, 0x04, 0x08, 0x01, 0x4E, 0x4A, 0x5E}));
  // empty and oversize data
  uint8_t o[8];
  CHECK(xbee_encode(XBeeApiMode::kApi1, d1, 0, o, sizeof o) == 0);
  CHECK(xbee_encoded_size(XBeeApiMode::kApi2, d1, 0) == 0);
  CHECK(xbee_encode(XBeeApiMode::kApi1, d1, 2, o, 5) == 0);  // needs 6
  CHECK(xbee_encode(XBeeApiMode::kApi1, d1, 2, o, 6) == 6);
  CHECK(xbee_encode(XBeeApiMode::kApi2, d1, 2, o, 6) == 0);  // needs 7
  CHECK(xbee_encode(XBeeApiMode::kApi2, d1, 2, o, 7) == 7);
  Bytes huge(65536, 0x55);
  CHECK(xbee_encoded_size(XBeeApiMode::kApi1, huge.data(), huge.size()) == 0);
  CHECK(xbee_encoded_size(XBeeApiMode::kApi1, huge.data(), 65535) == 65539);
  // escaped length: 0x7E bytes of data -> length byte 0x7E
  Bytes d126(0x7E, 0x01);
  Bytes e = encode(XBeeApiMode::kApi2, d126);
  CHECK(e[0] == 0x7E && e[1] == 0x00 && e[2] == 0x7D && e[3] == 0x5E);
  CHECK(e.size() == 0x7E + 4 + 1 || e.size() == 0x7E + 4 + 2);
  // escaped length high byte: 0x1100 = 4352 bytes
  Bytes d4352(0x1100, 0x01);
  Bytes e4 = encode(XBeeApiMode::kApi2, d4352);
  CHECK(e4[1] == 0x7D && e4[2] == 0x31 && e4[3] == 0x00);
  CHECK(xbee_encoded_size(XBeeApiMode::kApi2, d4352.data(), d4352.size()) == e4.size());
}

static void test_encode_sizes() {
  for (int mi = 0; mi < 2; ++mi) {
    XBeeApiMode m = kModes[mi];
    for (int it = 0; it < 3000; ++it) {
      Bytes d = random_frame(300, true);
      Bytes e(xbee_max_encoded_size(m, d.size()));
      size_t es = xbee_encoded_size(m, d.data(), d.size());
      size_t n = xbee_encode(m, d.data(), d.size(), e.data(), e.size());
      CHECK(n == es);
      CHECK(n <= xbee_max_encoded_size(m, d.size()));
      CHECK(n >= d.size() + 4);
      if (m == XBeeApiMode::kApi2) {
        // only the start byte may be 0x7E; no raw XON/XOFF after it
        for (size_t i = 1; i < n; ++i) {
          CHECK(e[i] != 0x7E && e[i] != 0x11 && e[i] != 0x13);
        }
      } else {
        CHECK(n == d.size() + 4);
      }
      // too-small caps fail cleanly without touching memory beyond cap
      Bytes small(n - 1 + 8, 0xAA);
      CHECK(xbee_encode(m, d.data(), d.size(), small.data(), n - 1) == 0);
      for (size_t i = 0; i < small.size(); ++i) CHECK(small[i] == 0xAA);
    }
  }
}

static void test_roundtrip_parser() {
  for (int mi = 0; mi < 2; ++mi) {
    XBeeApiMode m = kModes[mi];
    // byte at a time, random chunks, both feed() and poll()
    for (int variant = 0; variant < 4; ++variant) {
      std::vector<Bytes> sent;
      Bytes wire;
      for (int i = 0; i < 400; ++i) {
        Bytes d = random_frame(120, true);
        sent.push_back(d);
        Bytes e = encode(m, d);
        wire.insert(wire.end(), e.begin(), e.end());
      }
      XBeeApiParser<128> p(m);
      std::vector<Bytes> got;
      size_t chunk = variant == 0 ? 1 : (variant == 1 ? 7 : 1000);
      if (variant < 2 || variant == 2) run_feed(p, wire, chunk, got);
      else run_poll(p, wire, 13, got);
      CHECK(got == sent);
      CHECK(p.stats().frames == sent.size());
      CHECK(p.stats().bad_checksum == 0 && p.stats().bad_length == 0);
      CHECK(p.stats().resyncs == 0 && p.stats().skipped == 0);
    }
  }
}

static void test_max_and_limits() {
  for (int mi = 0; mi < 2; ++mi) {
    XBeeApiMode m = kModes[mi];
    XBeeApiParser<16> p(m);
    // exactly MaxFrame is accepted
    Bytes d(16);
    for (size_t i = 0; i < d.size(); ++i) d[i] = (uint8_t)(0x7E - (int)i);  // escapey
    std::vector<Bytes> got;
    run_feed(p, encode(m, d), 5, got);
    CHECK(got.size() == 1 && got[0] == d);
    // MaxFrame+1 is dropped, and a good frame after it still arrives
    Bytes big(17, 0x42);
    Bytes small = B({0x8A, 0x02});
    Bytes wire = encode(m, big);
    Bytes e2 = encode(m, small);
    wire.insert(wire.end(), e2.begin(), e2.end());
    got.clear();
    XBeeApiParser<16> q(m);
    run_feed(q, wire, 3, got);
    CHECK(got.size() == 1 && got[0] == small);
    CHECK(q.stats().bad_length == 1);
    // length 0
    Bytes zero = B({0x7E, 0x00, 0x00, 0xFF});
    wire = zero;
    wire.insert(wire.end(), e2.begin(), e2.end());
    got.clear();
    XBeeApiParser<16> z(m);
    run_feed(z, wire, 2, got);
    CHECK(got.size() == 1 && got[0] == small);
    CHECK(z.stats().bad_length == 1);
    // huge length field
    wire = B({0x7E, 0xFF, 0xFF, 0x01, 0x02});
    wire.insert(wire.end(), e2.begin(), e2.end());
    got.clear();
    XBeeApiParser<16> h(m);
    run_feed(h, wire, 4, got);
    CHECK(got.size() == 1 && got[0] == small);
  }
  // 1-byte frames are fine
  XBeeApiParser<1> one(XBeeApiMode::kApi1);
  std::vector<Bytes> got;
  run_feed(one, encode(XBeeApiMode::kApi1, B({0x05})), 1, got);
  CHECK(got.size() == 1 && got[0] == B({0x05}));
}

static void test_bad_checksum_and_noise() {
  for (int mi = 0; mi < 2; ++mi) {
    XBeeApiMode m = kModes[mi];
    Bytes good = encode(m, B({0x90, 1, 2, 3}));
    Bytes bad = good;
    bad[bad.size() - 1] ^= 0x01;
    if (m == XBeeApiMode::kApi2 && (bad.back() == 0x7E || bad.back() == 0x7D || bad.back() == 0x11 ||
                                    bad.back() == 0x13))
      bad.back() ^= 0x02;
    Bytes wire = bad;
    wire.insert(wire.end(), good.begin(), good.end());
    XBeeApiParser<32> p(m);
    std::vector<Bytes> got;
    run_feed(p, wire, 100, got);
    CHECK(got.size() == 1 && got[0] == B({0x90, 1, 2, 3}));
    CHECK(p.stats().bad_checksum == 1);

    // noise (no 0x7E) between frames
    wire.clear();
    std::vector<Bytes> sent;
    for (int i = 0; i < 200; ++i) {
      for (size_t k = rnd_below(6); k > 0; --k) {
        uint8_t nb = (uint8_t)rnd32();
        if (nb == 0x7E) nb = 0x7F;
        wire.push_back(nb);
      }
      Bytes d = random_frame(30, true);
      sent.push_back(d);
      Bytes e = encode(m, d);
      wire.insert(wire.end(), e.begin(), e.end());
    }
    XBeeApiParser<32> q(m);
    got.clear();
    run_feed(q, wire, 9, got);
    // the noise sits between frames, so in AP=2 it is ignored when idle; in AP=1
    // it is counted as skipped
    CHECK(got == sent);
    if (m == XBeeApiMode::kApi1) CHECK(q.stats().skipped > 0);
  }
}

static void test_api1_rescan() {
  XBeeApiParser<64> p(XBeeApiMode::kApi1);
  Bytes frame = B({0x8A, 0x06});
  Bytes e = encode(XBeeApiMode::kApi1, frame);
  // stray start with a plausible length that swallows the start of the real frame
  Bytes prefixes[] = {
      B({0x7E, 0x00, 0x05, 0xAA}),
      B({0x7E, 0x00, 0x05}),
      B({0x7E}),
      B({0x7E, 0x7E}),
      B({0x7E, 0x7E, 0x7E, 0x00}),
      B({0x7E, 0x00, 0x20, 0x01, 0x02}),
      B({0x7E, 0x00, 0x02, 0x01}),
      B({0x7E, 0x00, 0x00}),
      B({0x7E, 0xFF}),
  };
  for (size_t i = 0; i < sizeof prefixes / sizeof prefixes[0]; ++i) {
    for (int chunked = 0; chunked < 2; ++chunked) {
      Bytes wire = prefixes[i];
      for (int c = 0; c < 8; ++c) wire.insert(wire.end(), e.begin(), e.end());
      XBeeApiParser<64> q(XBeeApiMode::kApi1);
      std::vector<Bytes> got;
      run_feed(q, wire, chunked ? 1 : 1000, got);
      CHECK(got.size() == 8);
      for (size_t k = 0; k < got.size(); ++k) CHECK(got[k] == frame);
    }
  }
  // a frame whose data contains 0x7E is still read as one frame
  Bytes d = B({0x90, 0x7E, 0x00, 0x02, 0x7E, 0x11});
  std::vector<Bytes> got;
  XBeeApiParser<64> q(XBeeApiMode::kApi1);
  run_feed(q, encode(XBeeApiMode::kApi1, d), 2, got);
  CHECK(got.size() == 1 && got[0] == d);
  (void)p;
}

static void test_api2_resync() {
  XBeeApiParser<64> p(XBeeApiMode::kApi2);
  Bytes good = encode(XBeeApiMode::kApi2, B({0x8A, 0x00}));
  // truncated frame followed by a good one
  Bytes wire = B({0x7E, 0x00, 0x10, 0x90, 0x01, 0x02});
  wire.insert(wire.end(), good.begin(), good.end());
  std::vector<Bytes> got;
  run_feed(p, wire, 3, got);
  CHECK(got.size() == 1 && got[0] == B({0x8A, 0x00}));
  CHECK(p.stats().resyncs == 1);
  // truncated in the middle of an escape
  wire = B({0x7E, 0x00, 0x04, 0x01, 0x7D});
  wire.insert(wire.end(), good.begin(), good.end());
  XBeeApiParser<64> q(XBeeApiMode::kApi2);
  got.clear();
  run_feed(q, wire, 1, got);
  CHECK(got.size() == 1 && got[0] == B({0x8A, 0x00}));
  // escaped byte after escape is not treated as a delimiter: 7D 5E = 0x7E data
  Bytes d = B({0x90, 0x7E, 0x7D, 0x11, 0x13, 0x00});
  XBeeApiParser<64> r(XBeeApiMode::kApi2);
  got.clear();
  run_feed(r, encode(XBeeApiMode::kApi2, d), 1, got);
  CHECK(got.size() == 1 && got[0] == d);
  CHECK(r.stats().resyncs == 0);
  // an API2 parser fed raw 0x11/0x13 passes them through as data
  XBeeApiParser<64> s(XBeeApiMode::kApi2);
  got.clear();
  run_feed(s, B({0x7E, 0x00, 0x02, 0x11, 0x13, 0xDB}), 6, got);  // checksum FF-(0x11+0x13)=0xDB
  CHECK(got.size() == 1 && got[0] == B({0x11, 0x13}));
}

static void test_parser_misc() {
  XBeeApiParser<32> p(XBeeApiMode::kApi2);
  CHECK(p.mode() == XBeeApiMode::kApi2);
  CHECK(!p.frame_ready() && p.frame_length() == 0 && p.frame_type() == 0);
  Bytes e = encode(XBeeApiMode::kApi2, B({0x8A, 0x02}));
  size_t used = p.feed(e.data(), e.size());
  CHECK(used == e.size() && p.frame_ready() && p.frame_length() == 2 && p.frame_type() == 0x8A);
  p.feed(0, 0);
  CHECK(!p.frame_ready() && p.frame_length() == 0);
  // feed stops right after a frame and reports the used count
  Bytes two = e;
  two.insert(two.end(), e.begin(), e.end());
  used = p.feed(two.data(), two.size());
  CHECK(used == e.size() && p.frame_ready());
  used = p.feed(two.data() + used, two.size() - used);
  CHECK(used == e.size() && p.frame_ready());
  CHECK(p.stats().frames == 3);
  // switching mode resets state and stats
  p.set_mode(XBeeApiMode::kApi1);
  CHECK(p.mode() == XBeeApiMode::kApi1 && p.stats().frames == 0 && !p.frame_ready());
  // partial frame, then reset
  p.feed(e.data(), 3);
  p.reset();
  Wire w;
  used = p.feed(e.data(), e.size());  // API1 now: encoded API2 bytes equal API1 here (no escapes)
  CHECK(p.frame_ready());
  // poll on an empty stream
  XBeeApiParser<32> q(XBeeApiMode::kApi2);
  CHECK(!q.poll(w));
  w.inject(e.data(), e.size() - 1);
  CHECK(!q.poll(w));
  w.inject(e.data() + e.size() - 1, 1);
  CHECK(q.poll(w) && q.frame_type() == 0x8A);
  CHECK(!q.poll(w) && q.frame_length() == 0);
}

// A stream that says it has room but accepts only `cap` bytes per call overall.
class StingyStream final : public Stream {
 public:
  explicit StingyStream(size_t accept, size_t reported) : accept_(accept), reported_(reported) {}
  size_t read(uint8_t*, size_t) override { return 0; }
  size_t write(const uint8_t* s, size_t n) override {
    size_t k = n < accept_ ? n : accept_;
    accept_ -= k;
    got.insert(got.end(), s, s + k);
    return k;
  }
  size_t available() const override { return 0; }
  size_t write_space() const override { return reported_; }
  Bytes got;

 private:
  size_t accept_, reported_;
};

static void test_send() {
  for (int mi = 0; mi < 2; ++mi) {
    XBeeApiMode m = kModes[mi];
    // exact fit, one short, unbounded
    for (int it = 0; it < 500; ++it) {
      Bytes d = random_frame(200, true);
      Bytes want = encode(m, d);
      Wire w;
      CHECK(xbee_send(w, m, d.data(), d.size()) == XBeeSendResult::kSent);
      Bytes got(want.size() + 4);
      size_t n = w.take(got.data(), got.size());
      got.resize(n);
      CHECK(got == want);

      StingyStream tight(1000, want.size());
      CHECK(xbee_send(tight, m, d.data(), d.size()) == XBeeSendResult::kSent);
      CHECK(tight.got == want);
      StingyStream short1(1000, want.size() - 1);
      CHECK(xbee_send(short1, m, d.data(), d.size()) == XBeeSendResult::kNoSpace);
      CHECK(short1.got.empty());
      StingyStream unb(1000, kUnboundedSpace);
      CHECK(xbee_send(unb, m, d.data(), d.size()) == XBeeSendResult::kSent);
      CHECK(unb.got == want);
      // stream lies about space
      StingyStream liar(want.size() - 1, kUnboundedSpace);
      CHECK(xbee_send(liar, m, d.data(), d.size()) == XBeeSendResult::kPartial);
    }
    uint8_t x = 1;
    Wire w;
    CHECK(xbee_send(w, m, &x, 0) == XBeeSendResult::kTooLong);
    CHECK(w.tx_size() == 0);
    Bytes huge(65536, 0x55);
    CHECK(xbee_send(w, m, huge.data(), huge.size()) == XBeeSendResult::kTooLong);
    CHECK(w.tx_size() == 0);
  }
  // a frame bigger than the wire buffer is refused, nothing written
  LoopbackStream<4, 16> tiny;
  Bytes d(30, 0x42);
  CHECK(xbee_send(tiny, XBeeApiMode::kApi1, d.data(), d.size()) == XBeeSendResult::kNoSpace);
  CHECK(tiny.tx_size() == 0);
  // 65535-byte frame through an unbounded stream
  StingyStream sink(1u << 20, kUnboundedSpace);
  Bytes big(65535);
  for (size_t i = 0; i < big.size(); ++i) big[i] = (uint8_t)i;
  CHECK(xbee_send(sink, XBeeApiMode::kApi2, big.data(), big.size()) == XBeeSendResult::kSent);
  CHECK(sink.got == encode(XBeeApiMode::kApi2, big));
}

static void test_builders_parsers() {
  uint8_t f[64];
  // AT: set ID = 0x7FFF
  const uint8_t pv[2] = {0x7F, 0xFF};
  size_t n = xbee_build_at(f, sizeof f, 7, "ID", pv, 2);
  CHECK(n == 6 && f[0] == 0x08 && f[1] == 7 && f[2] == 'I' && f[3] == 'D' && f[4] == 0x7F &&
        f[5] == 0xFF);
  n = xbee_build_at(f, sizeof f, 7, "ID", pv, 2, true);
  CHECK(n == 6 && f[0] == 0x09);
  CHECK(xbee_build_at(f, 5, 7, "ID", pv, 2) == 0);
  CHECK(xbee_build_at(f, 6, 7, "ID", pv, 2) == 6);
  CHECK(xbee_build_at(f, sizeof f, 7, "I") == 0);
  CHECK(xbee_build_at(f, sizeof f, 7, "") == 0);
  CHECK(xbee_build_at(f, sizeof f, 7, 0) == 0);
  CHECK(xbee_build_at(f, sizeof f, 7, "ID", 0, 2) == 0);
  CHECK(xbee_build_at(f, 3, 7, "ID") == 0);

  // remote AT
  n = xbee_build_remote_at(f, sizeof f, 9, 0x0013A20041ABCDEFull, "NI", (const uint8_t*)"Hi", 2);
  CHECK(n == 17);
  CHECK(f[0] == 0x17 && f[1] == 9 && f[10] == 0xFF && f[11] == 0xFE && f[12] == 0x02 &&
        f[13] == 'N' && f[14] == 'I' && f[15] == 'H' && f[16] == 'i');
  CHECK(detail::get_be64(f + 2) == 0x0013A20041ABCDEFull);
  CHECK(xbee_build_remote_at(f, 16, 9, 1, "NI", (const uint8_t*)"Hi", 2) == 0);
  CHECK(xbee_build_remote_at(f, 17, 9, 1, "NI", (const uint8_t*)"Hi", 2) == 17);
  n = xbee_build_remote_at(f, sizeof f, 9, 1, "AC", 0, 0, xbee_frame::kRemoteDisableAck, 0x1234);
  CHECK(n == 15 && f[10] == 0x12 && f[11] == 0x34 && f[12] == 0x01);

  // TX request
  const uint8_t pl[3] = {'a', 0x7E, 'c'};
  n = xbee_build_tx(f, sizeof f, 3, 0x0013A20000000001ull, pl, 3, 0x01, 5);
  CHECK(n == 17 && f[0] == 0x10 && f[1] == 3 && f[9] == 0x01 && f[10] == 0xFF && f[11] == 0xFE &&
        f[12] == 5 && f[13] == 0x01 && f[14] == 'a' && f[15] == 0x7E && f[16] == 'c');
  CHECK(xbee_build_tx(f, 16, 3, 1, pl, 3) == 0);
  CHECK(xbee_build_tx(f, 17, 3, 1, pl, 3) == 17);
  CHECK(xbee_build_tx(f, sizeof f, 0, 1, 0, 0) == 14);
  CHECK(xbee_build_tx(f, sizeof f, 0, 1, 0, 1) == 0);

  // AT response parse
  const uint8_t ar[] = {0x88, 5, 'I', 'D', 0, 0x7F, 0xFF};
  XBeeAtResponse a = XBeeAtResponse();
  CHECK(xbee_parse_at_response(ar, sizeof ar, a));
  CHECK(a.frame_id == 5 && a.cmd[0] == 'I' && a.cmd[1] == 'D' && a.status == 0 &&
        a.data_len == 2 && a.data == ar + 5 && a.data[0] == 0x7F);
  CHECK(xbee_parse_at_response(ar, 5, a) && a.data_len == 0);
  CHECK(!xbee_parse_at_response(ar, 4, a));
  CHECK(!xbee_parse_at_response(ar + 1, 6, a));
  CHECK(!xbee_parse_at_response(0, 7, a));
  // short frames of every length never read past n
  for (size_t k = 0; k < sizeof ar; ++k) {
    bool ok = xbee_parse_at_response(ar, k, a);
    CHECK(ok == (k >= 5));
  }

  // Remote AT response
  uint8_t rr[20] = {0x97, 4, 0x00, 0x13, 0xA2, 0x00, 0x40, 0x11, 0x22, 0x33, 0xFF, 0xFE,
                    'S',  'L', 0x00, 0xDE, 0xAD};
  XBeeRemoteAtResponse r = XBeeRemoteAtResponse();
  CHECK(xbee_parse_remote_at_response(rr, 17, r));
  CHECK(r.frame_id == 4 && r.src64 == 0x0013A20040112233ull && r.src16 == 0xFFFE &&
        r.cmd[0] == 'S' && r.cmd[1] == 'L' && r.status == 0 && r.data_len == 2 && r.data[1] == 0xAD);
  CHECK(!xbee_parse_remote_at_response(rr, 14, r));
  CHECK(xbee_parse_remote_at_response(rr, 15, r) && r.data_len == 0);
  rr[0] = 0x88;
  CHECK(!xbee_parse_remote_at_response(rr, 17, r));

  // RX packet
  const uint8_t rx[] = {0x90, 0x00, 0x13, 0xA2, 0x00, 0x41, 0x52, 0x53, 0x54, 0xAB, 0xCD, 0x01, 'h', 'i'};
  XBeeRxPacket p = XBeeRxPacket();
  CHECK(xbee_parse_rx(rx, sizeof rx, p));
  CHECK(p.src64 == 0x0013A20041525354ull && p.src16 == 0xABCD && p.options == 1 &&
        p.data_len == 2 && p.data[0] == 'h');
  CHECK(xbee_parse_rx(rx, 12, p) && p.data_len == 0);
  CHECK(!xbee_parse_rx(rx, 11, p));

  // TX status
  const uint8_t ts[] = {0x89, 0x52, 0x00};
  XBeeTxStatus t = XBeeTxStatus();
  CHECK(xbee_parse_tx_status(ts, 3, t) && t.frame_id == 0x52 && t.delivery == 0 && !t.extended);
  CHECK(!xbee_parse_tx_status(ts, 2, t));
  const uint8_t te[] = {0x8B, 0x10, 0x12, 0x34, 0x02, 0x21, 0x02};
  CHECK(xbee_parse_tx_status(te, 7, t) && t.extended && t.frame_id == 0x10 && t.addr16 == 0x1234 &&
        t.retries == 2 && t.delivery == 0x21 && t.discovery == 2);
  CHECK(!xbee_parse_tx_status(te, 6, t));
  CHECK(!xbee_parse_tx_status(rx, sizeof rx, t));

  // modem status
  uint8_t st = 99;
  const uint8_t ms[] = {0x8A, 0x06};
  CHECK(xbee_parse_modem_status(ms, 2, st) && st == 6);
  CHECK(!xbee_parse_modem_status(ms, 1, st));
  CHECK(!xbee_parse_modem_status(rx, 2, st));
}

static void test_api_class() {
  Wire wire;
  XBeeApi<> api(wire, XBeeApiMode::kApi2);
  uint8_t buf[300];

  // ids cycle 1..255 and skip 0
  for (int i = 1; i <= 600; ++i) {
    uint8_t id = api.send_at("NJ");
    int expect = ((i - 1) % 255) + 1;
    CHECK(id == expect);
    CHECK(id != 0);
    size_t n = wire.take(buf, sizeof buf);
    XBeeApiParser<> pp(XBeeApiMode::kApi2);
    CHECK(pp.feed(buf, n) == n && pp.frame_ready());
    CHECK(pp.frame_type() == 0x08 && pp.frame()[1] == id && pp.frame()[2] == 'N');
  }

  // send_at with params / queued
  const uint8_t pv[1] = {0x11};  // 0x11 needs escaping on the wire
  uint8_t id = api.send_at("NJ", pv, 1, true);
  CHECK(id != 0);
  size_t n = wire.take(buf, sizeof buf);
  XBeeApiParser<> pp(XBeeApiMode::kApi2);
  CHECK(pp.feed(buf, n) == n && pp.frame_ready());
  CHECK(pp.frame_type() == 0x09 && pp.frame_length() == 5 && pp.frame()[4] == 0x11);
  bool escaped = false;
  for (size_t i = 0; i + 1 < n; ++i) escaped = escaped || (buf[i] == 0x7D && buf[i + 1] == 0x31);
  CHECK(escaped);

  // remote AT, tx with status and without
  id = api.send_remote_at(0x0013A20041ABCDEFull, "AC");
  CHECK(id != 0);
  n = wire.take(buf, sizeof buf);
  pp.reset();
  CHECK(pp.feed(buf, n) == n && pp.frame_type() == 0x17 && pp.frame()[12] == 0x02);

  const uint8_t pl[4] = {1, 2, 0x7E, 4};
  id = api.send_tx(0x000000000000FFFFull, pl, 4);
  CHECK(id != 0);
  n = wire.take(buf, sizeof buf);
  pp.reset();
  CHECK(pp.feed(buf, n) == n && pp.frame_type() == 0x10 && pp.frame()[1] == id &&
        pp.frame_length() == 18 && pp.frame()[16] == 0x7E);
  uint8_t r = api.send_tx(1, pl, 4, 0, 0, false);
  CHECK(r == 1);
  n = wire.take(buf, sizeof buf);
  pp.reset();
  CHECK(pp.feed(buf, n) == n && pp.frame()[1] == 0);
  // the id counter was not consumed by the no-status send
  uint8_t id2 = api.send_at("NJ");
  wire.take(buf, sizeof buf);
  CHECK(id2 != 0);

  // too long for MaxOut: nothing sent
  Bytes big(200, 1);
  CHECK(api.send_tx(1, big.data(), big.size()) == 0);
  CHECK(api.last_send_result() == XBeeSendResult::kTooLong);
  CHECK(wire.tx_size() == 0);
  CHECK(api.send_tx(1, big.data(), big.size(), 0, 0, false) == 0);
  CHECK(api.send_at("X") == 0);
  CHECK(wire.tx_size() == 0);

  // no space: nothing written, result says so; later it works
  LoopbackStream<64, 16> small;
  XBeeApi<> a2(small, XBeeApiMode::kApi1);
  CHECK(a2.send_tx(1, pl, 4) == 0);  // 18+4 bytes needed, 16 free
  CHECK(a2.last_send_result() == XBeeSendResult::kNoSpace);
  CHECK(small.tx_size() == 0);
  CHECK(a2.send_at("NJ") != 0 && a2.last_send_result() == XBeeSendResult::kSent);
  CHECK(!a2.send_frame(buf, 0));
  CHECK(a2.last_send_result() == XBeeSendResult::kTooLong);

  // full duplex round trip: a "radio" answers every AT query
  Wire dev;
  XBeeApi<> host(dev, XBeeApiMode::kApi2);
  for (int i = 0; i < 300; ++i) {
    uint8_t qid = host.send_at("ID");
    uint8_t out[64];
    size_t on = dev.take(out, sizeof out);
    XBeeApiParser<> radio(XBeeApiMode::kApi2);
    CHECK(radio.feed(out, on) == on && radio.frame_ready());
    // answer: 88 id 'I' 'D' 00 7F FF (escape-heavy values too)
    uint8_t resp[8] = {0x88, radio.frame()[1], 'I', 'D', 0, (uint8_t)(i & 1 ? 0x7E : 0x11), 0x7D};
    uint8_t enc[32];
    size_t en = xbee_encode(XBeeApiMode::kApi2, resp, 7, enc, sizeof enc);
    CHECK(en != 0);
    dev.inject(enc, en);
    bool got = false;
    for (int k = 0; k < 4 && !got; ++k) got = host.poll();
    CHECK(got && host.frame_type() == 0x88);
    XBeeAtResponse ar = XBeeAtResponse();
    CHECK(xbee_parse_at_response(host.frame(), host.frame_length(), ar));
    CHECK(ar.frame_id == qid && ar.data_len == 2 && ar.data[1] == 0x7D);
    CHECK(!host.poll());
  }
  CHECK(host.stats().frames == 300);
  host.set_mode(XBeeApiMode::kApi1);
  CHECK(host.stats().frames == 0);
}

static void test_more_frames() {
  uint8_t f[64];
  // 802.15.4 16-bit transmit: Digi example frames
  const uint8_t txd[6] = {'T', 'x', 'D', 'a', 't', 'a'};
  size_t n = xbee_build_tx16(f, sizeof f, 0x87, 0x1234, txd, 6);
  CHECK(n == 11);
  CHECK(encode(XBeeApiMode::kApi1, Bytes(f, f + n)) ==
        B({0x7E, 0x00, 0x0B, 0x01, 0x87, 0x12, 0x34, 0x00, 0x54, 0x78, 0x44, 0x61, 0x74, 0x61, 0xEB}));
  n = xbee_build_tx16(f, sizeof f, 0, 0xFFFF, (const uint8_t*)"Broadcast", 9);
  CHECK(encode(XBeeApiMode::kApi1, Bytes(f, f + n)) ==
        B({0x7E, 0x00, 0x0E, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x42, 0x72, 0x6F, 0x61, 0x64, 0x63, 0x61,
           0x73, 0x74, 0x6D}));
  CHECK(xbee_build_tx16(f, 10, 1, 1, txd, 6) == 0 && xbee_build_tx16(f, 11, 1, 1, txd, 6) == 11);
  CHECK(xbee_build_tx16(f, sizeof f, 1, 1, 0, 3) == 0);

  // 64-bit transmit
  n = xbee_build_tx64(f, sizeof f, 2, 0x0013A20041ABCDEFull, txd, 2, xbee_frame::kTx802DisableAck);
  CHECK(n == 13 && f[0] == 0x00 && f[1] == 2 && f[2] == 0x00 && f[9] == 0xEF && f[10] == 0x01 &&
        f[11] == 'T' && f[12] == 'x');
  CHECK(xbee_build_tx64(f, 12, 2, 1, txd, 2) == 0 && xbee_build_tx64(f, 13, 2, 1, txd, 2) == 13);

  // 16-bit receive: Digi example 7E 00 0B 81 12 34 5E 01 'TxData' 93
  XBeeRx802 r = XBeeRx802();
  Bytes ex = B({0x81, 0x12, 0x34, 0x5E, 0x01, 0x54, 0x78, 0x44, 0x61, 0x74, 0x61});
  CHECK(encode(XBeeApiMode::kApi1, ex).back() == 0x93);
  CHECK(xbee_parse_rx802(ex.data(), ex.size(), r));
  CHECK(!r.addr64_valid && r.src16 == 0x1234 && r.rssi == 0x5E && r.options == 1 &&
        r.data_len == 6 && r.data[0] == 'T');
  CHECK(xbee_parse_rx802(ex.data(), 5, r) && r.data_len == 0);
  CHECK(!xbee_parse_rx802(ex.data(), 4, r));
  // 64-bit receive
  Bytes e64 = B({0x80, 0x00, 0x13, 0xA2, 0x00, 0x41, 0x52, 0x53, 0x54, 0x28, 0x02, 'o', 'k'});
  CHECK(xbee_parse_rx802(e64.data(), e64.size(), r));
  CHECK(r.addr64_valid && r.src64 == 0x0013A20041525354ull && r.rssi == 0x28 && r.options == 2 &&
        r.data_len == 2);
  CHECK(xbee_parse_rx802(e64.data(), 11, r) && r.data_len == 0);
  CHECK(!xbee_parse_rx802(e64.data(), 10, r));
  CHECK(!xbee_parse_rx802(0, 11, r));
  const uint8_t wrong = 0x90;
  CHECK(!xbee_parse_rx802(&wrong, 1, r));
  for (size_t k = 0; k <= e64.size(); ++k) CHECK(xbee_parse_rx802(e64.data(), k, r) == (k >= 11));

  // explicit transmit
  const uint8_t pl[2] = {0xAA, 0x7E};
  n = xbee_build_explicit_tx(f, sizeof f, 5, 0x0013A20041ABCDEFull, xbee_frame::kSerialEndpoint,
                             xbee_frame::kSerialEndpoint, xbee_frame::kSerialCluster,
                             xbee_frame::kSerialProfile, pl, 2, 0x01, 3);
  CHECK(n == 22 && f[0] == 0x11 && f[1] == 5 && f[10] == 0xFF && f[11] == 0xFE && f[12] == 0xE8 &&
        f[13] == 0xE8 && f[14] == 0x00 && f[15] == 0x11 && f[16] == 0xC1 && f[17] == 0x05 &&
        f[18] == 3 && f[19] == 0x01 && f[20] == 0xAA && f[21] == 0x7E);
  CHECK(xbee_build_explicit_tx(f, 21, 5, 1, 1, 1, 1, 1, pl, 2) == 0);
  CHECK(xbee_build_explicit_tx(f, 22, 5, 1, 1, 1, 1, 1, pl, 2) == 22);
  CHECK(xbee_build_explicit_tx(f, sizeof f, 5, 1, 1, 1, 1, 1, 0, 1) == 0);

  // explicit receive
  uint8_t xr[24] = {0x91, 0x00, 0x13, 0xA2, 0x00, 0x41, 0x52, 0x53, 0x54, 0xFF, 0xFE, 0xE8,
                    0xE8, 0x00, 0x11, 0xC1, 0x05, 0xC1, 'h', 'i'};
  XBeeExplicitRx x = XBeeExplicitRx();
  CHECK(xbee_parse_explicit_rx(xr, 20, x));
  CHECK(x.src64 == 0x0013A20041525354ull && x.src16 == 0xFFFE && x.src_endpoint == 0xE8 &&
        x.dst_endpoint == 0xE8 && x.cluster == 0x0011 && x.profile == 0xC105 && x.options == 0xC1 &&
        x.data_len == 2 && x.data[1] == 'i');
  CHECK(xbee_parse_explicit_rx(xr, 18, x) && x.data_len == 0);
  CHECK(!xbee_parse_explicit_rx(xr, 17, x));
  xr[0] = 0x90;
  CHECK(!xbee_parse_explicit_rx(xr, 20, x));

  // through XBeeApi and the codec
  Wire w;
  XBeeApi<> api(w, XBeeApiMode::kApi2);
  uint8_t buf[200];
  uint8_t id = api.send_explicit(1, 0xE8, 0xE8, 0x0011, 0xC105, pl, 2);
  CHECK(id != 0);
  size_t bn = w.take(buf, sizeof buf);
  XBeeApiParser<> pp(XBeeApiMode::kApi2);
  CHECK(pp.feed(buf, bn) == bn && pp.frame_type() == 0x11 && pp.frame()[1] == id);
  id = api.send_tx64(1, pl, 2);
  CHECK(id != 0);
  bn = w.take(buf, sizeof buf);
  pp.reset();
  CHECK(pp.feed(buf, bn) == bn && pp.frame_type() == 0x00 && pp.frame_length() == 13);
  id = api.send_tx16(0xFFFF, pl, 2, xbee_frame::kTx802BroadcastPan);
  CHECK(id != 0);
  bn = w.take(buf, sizeof buf);
  pp.reset();
  CHECK(pp.feed(buf, bn) == bn && pp.frame_type() == 0x01 && pp.frame()[4] == 0x04);
  Bytes big(120, 1);
  CHECK(api.send_explicit(1, 1, 1, 1, 1, big.data(), big.size()) == 0 && w.tx_size() == 0);
}

static void test_fuzz() {
  // 1. random bytes never break invariants
  for (int mi = 0; mi < 2; ++mi) {
    XBeeApiMode m = kModes[mi];
    XBeeApiParser<24> p(m);
    for (int it = 0; it < 200; ++it) {
      Bytes junk(1 + rnd_below(400));
      for (size_t i = 0; i < junk.size(); ++i) {
        uint32_t r = rnd_below(8);
        junk[i] = r == 0 ? 0x7E : (r == 1 ? 0x7D : (uint8_t)rnd32());
        if (r == 2) junk[i] = (uint8_t)rnd_below(4);  // small lengths
      }
      std::vector<Bytes> got;
      run_feed(p, junk, 1 + rnd_below(20), got);
      for (size_t i = 0; i < got.size(); ++i) {
        CHECK(got[i].size() >= 1 && got[i].size() <= 24);
      }
    }
    CHECK(p.stats().frames + p.stats().bad_checksum + p.stats().bad_length + p.stats().resyncs +
              p.stats().skipped >
          0);
  }

  // 2. good frames among noise that contains 0x7E: nearly all survive
  for (int mi = 0; mi < 2; ++mi) {
    XBeeApiMode m = kModes[mi];
    Bytes wire;
    size_t sent = 0;
    for (int i = 0; i < 2000; ++i) {
      for (size_t k = rnd_below(5); k > 0; --k) {
        uint32_t r = rnd_below(4);
        wire.push_back(r == 0 ? 0x7E : (uint8_t)rnd32());
      }
      Bytes d = random_frame(20, true);
      Bytes e = encode(m, d);
      wire.insert(wire.end(), e.begin(), e.end());
      ++sent;
    }
    XBeeApiParser<24> p(m);
    std::vector<Bytes> got;
    run_feed(p, wire, 11, got);
    // every wire byte of every good frame is accounted for; allow rare collisions
    // where noise looks like a valid frame that swallows a real one (AP=1)
    printf("  fuzz %s: %zu of %zu good frames delivered\n", m == XBeeApiMode::kApi1 ? "AP=1" : "AP=2",
           got.size(), sent);
    CHECK(got.size() >= sent * 9 / 10);
    // delivery is identical regardless of chunking
    XBeeApiParser<24> q(m);
    std::vector<Bytes> got2;
    run_feed(q, wire, 1, got2);
    CHECK(got == got2);
    XBeeApiParser<24> q3(m);
    std::vector<Bytes> got3;
    run_poll(q3, wire, 300, got3);
    CHECK(got == got3);
  }

  // 3. no noise inside a stream with 0x7E: AP=2 must deliver every frame
  {
    Bytes wire;
    std::vector<Bytes> sent;
    for (int i = 0; i < 3000; ++i) {
      Bytes d = random_frame(40, true);
      sent.push_back(d);
      Bytes e = encode(XBeeApiMode::kApi2, d);
      // sometimes cut the frame short; the next 0x7E must recover
      if (rnd_below(10) == 0 && e.size() > 3) {
        sent.pop_back();
        e.resize(1 + rnd_below((uint32_t)e.size() - 2));
        // a prefix that happens to complete a valid frame is possible only for the whole frame
      }
      wire.insert(wire.end(), e.begin(), e.end());
    }
    XBeeApiParser<64> p(XBeeApiMode::kApi2);
    std::vector<Bytes> got;
    run_feed(p, wire, 5, got);
    CHECK(got == sent);
  }
}

int main() {
  test_vectors();
  test_encode_sizes();
  test_roundtrip_parser();
  test_max_and_limits();
  test_bad_checksum_and_noise();
  test_api1_rescan();
  test_api2_resync();
  test_parser_misc();
  test_send();
  test_builders_parsers();
  test_api_class();
  test_more_frames();
  test_fuzz();
  printf("%lu checks, %lu failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
