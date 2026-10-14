// Host-side tests for storage/kv_store.hpp and hal/ram_flash.hpp.
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_kv_store.cpp -o test_kv_store && ./test_kv_store
//
// The central test cuts the power at every single flash write operation of a
// long scenario (program or erase, including compaction and recovery), reboots,
// and requires that the store mounts and holds exactly the state from before or
// after the interrupted call.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <map>
#include <vector>

#include "hal/ram_flash.hpp"
#include "storage/kv_store.hpp"

using namespace mcu;
typedef std::vector<uint8_t> Bytes;
typedef std::map<uint16_t, Bytes> Model;

static unsigned long g_checks = 0;
static unsigned long g_fails = 0;
static bool g_debug = false;

#define CHECK(...)                                                                    \
  do {                                                                                \
    ++g_checks;                                                                       \
    if (!(__VA_ARGS__)) {                                                             \
      ++g_fails;                                                                      \
      if (g_fails < 40) printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__); \
    }                                                                                 \
  } while (0)

static uint64_t g_rng = 0xA11CE5EED5ull;
static uint32_t rnd32() {
  g_rng ^= g_rng >> 12;
  g_rng ^= g_rng << 25;
  g_rng ^= g_rng >> 27;
  return (uint32_t)((g_rng * 0x2545F4914F6CDD1Dull) >> 32);
}
static uint32_t rnd_below(uint32_t n) { return rnd32() % n; }

static Bytes make_value(size_t len, uint32_t seed) {
  Bytes v(len);
  uint32_t s = seed * 2654435761u + 1;
  for (size_t i = 0; i < len; ++i) {
    s = s * 1664525u + 1013904223u;
    v[i] = (uint8_t)(s >> 24);
  }
  return v;
}

// ---- model comparison ---------------------------------------------------------

template <typename KV>
static bool matches(KV& kv, const Model& m, uint16_t key_space) {
  for (uint16_t k = 0; k < key_space; ++k) {
    uint8_t buf[600];
    size_t n = 0;
    KvResult r = kv.get(k, buf, sizeof buf, &n);
    Model::const_iterator it = m.find(k);
    if (it == m.end()) {
      if (r != KvResult::kNotFound) { if (g_debug) printf("  key %u: r=%d, expected missing\n", k, (int)r); return false; }
    } else {
      if (r != KvResult::kOk || n != it->second.size()) { if (g_debug) printf("  key %u: r=%d n=%zu expected len %zu\n", k, (int)r, n, it->second.size()); return false; }
      if (n != 0 && memcmp(buf, &it->second[0], n) != 0) { if (g_debug) printf("  key %u: content differs\n", k); return false; }
    }
  }
  // key iteration
  uint16_t k = 0;
  KvResult r = kv.first_key(k);
  for (Model::const_iterator it = m.begin(); it != m.end(); ++it) {
    if (r != KvResult::kOk || k != it->first) return false;
    r = kv.next_key(k);
  }
  return r == KvResult::kNotFound;
}

template <typename KV>
static bool apply_put(KV& kv, Model& m, uint16_t key, const Bytes& v, KvResult& r) {
  r = kv.put(key, v.empty() ? 0 : &v[0], v.size());
  if (r == KvResult::kOk) m[key] = v;
  return r == KvResult::kOk;
}

// ---- tests --------------------------------------------------------------------

typedef RamFlash<256, 4, 4> Flash4;

static void test_basics() {
  RamFlash<256, 4, 1> f;
  KvStore<4> kv(f);
  CHECK(!kv.ready());
  uint8_t b[8];
  CHECK(kv.put(1, "a", 1) == KvResult::kNotReady);
  CHECK(kv.get(1, b, 8) == KvResult::kNotReady);
  CHECK(kv.begin() == KvResult::kOk && kv.ready());
  CHECK(kv.get(1, b, sizeof b) == KvResult::kNotFound);
  CHECK(!kv.contains(1));

  CHECK(kv.put(1, "hello", 5) == KvResult::kOk);
  size_t n = 0;
  CHECK(kv.get(1, b, sizeof b, &n) == KvResult::kOk && n == 5 && memcmp(b, "hello", 5) == 0);
  CHECK(kv.contains(1));
  CHECK(kv.length(1, &n) == KvResult::kOk && n == 5);
  CHECK(kv.get(1, b, 4, &n) == KvResult::kBufferTooSmall && n == 5);
  CHECK(kv.get(1, 0, 0, &n) == KvResult::kBufferTooSmall);
  CHECK(kv.put(1, "world!", 6) == KvResult::kOk);
  CHECK(kv.get(1, b, sizeof b, &n) == KvResult::kOk && n == 6 && memcmp(b, "world!", 6) == 0);

  // empty value is a value
  CHECK(kv.put(2, 0, 0) == KvResult::kOk && kv.contains(2));
  CHECK(kv.get(2, b, sizeof b, &n) == KvResult::kOk && n == 0);
  CHECK(kv.get(2, 0, 0, &n) == KvResult::kOk);

  // remove
  CHECK(kv.remove(1) == KvResult::kOk && !kv.contains(1));
  CHECK(kv.remove(1) == KvResult::kNotFound);
  CHECK(kv.remove(77) == KvResult::kNotFound);
  CHECK(kv.put(1, "again", 5) == KvResult::kOk && kv.contains(1));

  // bad arguments
  CHECK(kv.put(0xFFFF, "x", 1) == KvResult::kBadArg);
  CHECK(kv.put(3, 0, 1) == KvResult::kBadArg);
  CHECK(kv.put(3, "x", 0x8000) == KvResult::kBadArg);
  CHECK(kv.get(0xFFFF, b, 8) == KvResult::kBadArg);
  CHECK(kv.remove(0xFFFF) == KvResult::kBadArg);
  CHECK(kv.length(1, 0) == KvResult::kBadArg);
  CHECK(kv.max_value_size() > 0);
  Bytes big(kv.max_value_size() + 1, 1);
  CHECK(kv.put(4, &big[0], big.size()) == KvResult::kTooBig);
  big.resize(kv.max_value_size());
  CHECK(kv.put(4, &big[0], big.size()) == KvResult::kOk);
  std::vector<uint8_t> back(big.size());
  CHECK(kv.get(4, &back[0], back.size(), &n) == KvResult::kOk && back == big);

  // typed helpers
  CHECK(kv.put_u8(10, 0xAB) == KvResult::kOk && kv.get_u8_or(10, 0) == 0xAB);
  CHECK(kv.put_u16(11, 0xBEEF) == KvResult::kOk && kv.get_u16_or(11, 0) == 0xBEEF);
  CHECK(kv.put_u32(12, 0xDEADBEEFu) == KvResult::kOk && kv.get_u32_or(12, 0) == 0xDEADBEEFu);
  CHECK(kv.put_i32(13, -123456) == KvResult::kOk && kv.get_i32_or(13, 0) == -123456);
  CHECK(kv.get_u32_or(99, 7) == 7);
  CHECK(kv.get_u32_or(10, 7) == 7);  // stored as 1 byte, asked as 4: default
  CHECK(kv.get_u8_or(12, 9) == 9);   // 4 bytes stored, asked as 1
  uint32_t u32 = 0;
  CHECK(kv.get_u32(12, u32) == KvResult::kOk && u32 == 0xDEADBEEFu);
  CHECK(kv.get_u32(99, u32) == KvResult::kNotFound);
  CHECK(kv.get_u32(10, u32) == KvResult::kBadArg);
  // little-endian on the wire
  CHECK(kv.get(12, b, sizeof b, &n) == KvResult::kOk && b[0] == 0xEF && b[3] == 0xDE);
  CHECK(kv.put_str(20, "Base-1") == KvResult::kOk);
  char s[16];
  CHECK(kv.get_str(20, s, sizeof s) == KvResult::kOk && strcmp(s, "Base-1") == 0);
  char tiny[4];
  CHECK(kv.get_str(20, tiny, sizeof tiny) == KvResult::kBufferTooSmall);
  CHECK(kv.put_str(21, "") == KvResult::kOk && kv.get_str(21, s, sizeof s) == KvResult::kOk && s[0] == 0);
  CHECK(kv.put_str(22, 0) == KvResult::kBadArg);
  CHECK(kv.get_str(20, 0, 5) == KvResult::kBadArg);

  // survives a reboot
  KvStore<4> kv2(f);
  CHECK(kv2.begin() == KvResult::kOk);
  CHECK(kv2.get_u32_or(12, 0) == 0xDEADBEEFu);
  CHECK(kv2.get_str(20, s, sizeof s) == KvResult::kOk && strcmp(s, "Base-1") == 0);
  CHECK(kv2.get(4, &back[0], back.size(), &n) == KvResult::kOk && back == big);

  // format wipes
  CHECK(kv2.format() == KvResult::kOk);
  CHECK(!kv2.contains(12) && !kv2.contains(4));
  CHECK(f.violations() == 0);
}

static void test_bad_geometry() {
  RamFlash<256, 4, 4> f;
  KvStore<4> a(f, 0, 1);  // one sector is not enough
  CHECK(a.begin() == KvResult::kBadArg);
  KvStore<4> b(f, 3, 2);  // runs off the end
  CHECK(b.begin() == KvResult::kBadArg);
  KvStore<3> c(f);  // 4 sectors, room for 3
  CHECK(c.begin() == KvResult::kBadArg);
  RamFlash<32, 4, 4> tiny;
  KvStore<4> d(tiny);
  CHECK(d.begin() == KvResult::kBadArg);
  KvStore<4> e(f, 5, 0);
  CHECK(e.begin() == KvResult::kBadArg);
}

static void test_skip_identical() {
  Flash4 f;
  KvStore<4> kv(f);
  CHECK(kv.begin() == KvResult::kOk);
  CHECK(kv.put_u32(1, 1234) == KvResult::kOk);
  CHECK(kv.put_str(2, "name") == KvResult::kOk);
  const uint64_t written = f.programmed_bytes();
  const uint32_t records = kv.stats().records_written;
  for (int i = 0; i < 50; ++i) {
    CHECK(kv.put_u32(1, 1234) == KvResult::kOk);
    CHECK(kv.put_str(2, "name") == KvResult::kOk);
  }
  CHECK(f.programmed_bytes() == written);
  CHECK(kv.stats().records_written == records);
  CHECK(kv.stats().skipped_writes == 100);
  CHECK(kv.put_u32(1, 1235) == KvResult::kOk);
  CHECK(f.programmed_bytes() > written);
  // a removed key written again with its old value is written
  CHECK(kv.remove(2) == KvResult::kOk);
  const uint64_t w2 = f.programmed_bytes();
  CHECK(kv.put_str(2, "name") == KvResult::kOk);
  CHECK(f.programmed_bytes() > w2);
  // remove of a missing key writes nothing
  const uint64_t w3 = f.programmed_bytes();
  CHECK(kv.remove(9) == KvResult::kNotFound);
  CHECK(f.programmed_bytes() == w3);
}

// Random operations against a std::map, with clean reboots, on many geometries.
template <size_t SS, size_t NS, size_t U>
static void fuzz_geometry(int steps, uint16_t key_space, size_t max_len) {
  const unsigned long fails0 = g_fails;
  RamFlash<SS, NS, U> f;
  KvStore<NS> kv(f);
  Model m;
  CHECK(kv.begin() == KvResult::kOk);
  for (int i = 0; i < steps; ++i) {
    const uint32_t what = rnd_below(100);
    const uint16_t key = (uint16_t)rnd_below(key_space);
    if (what < 60) {
      Bytes v = make_value(rnd_below((uint32_t)max_len + 1), rnd32());
      KvResult r;
      apply_put(kv, m, key, v, r);
      CHECK(r == KvResult::kOk || r == KvResult::kFull);
    } else if (what < 80) {
      KvResult r = kv.remove(key);
      if (m.count(key)) {
        CHECK(r == KvResult::kOk);
        m.erase(key);
      } else {
        CHECK(r == KvResult::kNotFound);
      }
    } else if (what < 85) {
      KvStore<NS> kv2(f);  // a second instance mounts the same flash
      CHECK(kv2.begin() == KvResult::kOk);
      CHECK(matches(kv2, m, key_space));
      CHECK(kv.begin() == KvResult::kOk);  // and the working instance re-mounts
    } else {
      CHECK(matches(kv, m, key_space));
    }
  }
  CHECK(matches(kv, m, key_space));
  CHECK(f.violations() == 0);
  // wear: erases are spread over the sectors
  uint32_t mn = 0xFFFFFFFFu, mx = 0;
  for (size_t s = 0; s < NS; ++s) {
    const uint32_t e = f.erase_count(s);
    if (e < mn) mn = e;
    if (e > mx) mx = e;
  }
  CHECK(mx - mn <= 3);
  if (g_fails != fails0) printf("  fuzz %zux%zu unit %zu: %lu failures (erases %u..%u)\n", SS, NS, U, g_fails - fails0, mn, mx);
}

static void test_fuzz() {
  fuzz_geometry<256, 4, 1>(6000, 10, 40);
  fuzz_geometry<256, 4, 2>(6000, 10, 40);
  fuzz_geometry<256, 4, 4>(6000, 10, 40);
  fuzz_geometry<512, 3, 8>(6000, 12, 60);
  fuzz_geometry<128, 2, 16>(4000, 4, 30);
  fuzz_geometry<128, 6, 4>(6000, 8, 30);
  fuzz_geometry<1024, 2, 4>(4000, 6, 200);
  fuzz_geometry<256, 8, 1>(6000, 12, 20);
}

static void test_value_lengths() {
  RamFlash<512, 3, 4> f;
  KvStore<3> kv(f);
  CHECK(kv.begin() == KvResult::kOk);
  Model m;
  for (size_t len = 0; len <= 140; ++len) {
    KvResult r;
    apply_put(kv, m, (uint16_t)(len % 5), make_value(len, (uint32_t)len), r);
    CHECK(r == KvResult::kOk);
    CHECK(matches(kv, m, 5));
  }
  // values at the largest the store accepts, repeatedly (forces compaction every time)
  CHECK(kv.format() == KvResult::kOk);
  m.clear();
  const size_t mx = kv.max_value_size();
  for (int i = 0; i < 20; ++i) {
    KvResult r;
    apply_put(kv, m, 7, make_value(mx, (uint32_t)i), r);
    CHECK(r == KvResult::kOk);
    CHECK(matches(kv, m, 8));
  }
  CHECK(kv.stats().compactions > 5);
  CHECK(f.violations() == 0);
}

static void test_full() {
  RamFlash<256, 3, 4> f;
  KvStore<3> kv(f);
  CHECK(kv.begin() == KvResult::kOk);
  Model m;
  // fill with distinct keys until full
  uint16_t k = 0;
  KvResult r = KvResult::kOk;
  for (; k < 200; ++k) {
    apply_put(kv, m, k, make_value(30, k), r);
    if (r != KvResult::kOk) break;
  }
  CHECK(r == KvResult::kFull && k > 10 && k < 100);
  CHECK(matches(kv, m, (uint16_t)(k + 1)));
  // repeated failing puts must not keep wearing the flash
  const uint32_t erases = f.total_erases();
  for (int i = 0; i < 50; ++i) {
    KvResult rr;
    apply_put(kv, m, (uint16_t)(k + 1), make_value(30, 999), rr);
    CHECK(rr == KvResult::kFull);
  }
  CHECK(f.total_erases() <= erases + 3);
  CHECK(matches(kv, m, (uint16_t)(k + 1)));
  // overwriting with a same-sized value does not need new keys... but the log is full:
  // free space by removing, then the put works again
  CHECK(kv.remove(0) == KvResult::kOk);
  m.erase(0);
  CHECK(kv.remove(1) == KvResult::kOk);
  m.erase(1);
  apply_put(kv, m, (uint16_t)(k + 1), make_value(30, 5), r);
  CHECK(r == KvResult::kOk);
  CHECK(matches(kv, m, (uint16_t)(k + 2)));
  // reboot keeps everything
  KvStore<3> kv2(f);
  CHECK(kv2.begin() == KvResult::kOk);
  CHECK(matches(kv2, m, (uint16_t)(k + 2)));
  CHECK(f.violations() == 0);
}

static void test_window() {
  // The store must stay inside its sectors.
  RamFlash<256, 8, 4> f;
  for (size_t s = 0; s < 8; ++s) {
    if (s >= 2 && s < 5) continue;
    memset(f.raw(s), 0x5A, 256);
  }
  f.mark_all_dirty_from_content();
  Bytes before[8];
  for (size_t s = 0; s < 8; ++s) before[s] = Bytes(f.raw(s), f.raw(s) + 256);
  KvStore<3> kv(f, 2, 3);
  CHECK(kv.begin() == KvResult::kOk);
  Model m;
  for (int i = 0; i < 400; ++i) {
    KvResult r;
    apply_put(kv, m, (uint16_t)rnd_below(6), make_value(rnd_below(50), rnd32()), r);
    CHECK(r == KvResult::kOk);
  }
  CHECK(matches(kv, m, 6));
  for (size_t s = 0; s < 8; ++s) {
    if (s >= 2 && s < 5) continue;
    CHECK(Bytes(f.raw(s), f.raw(s) + 256) == before[s]);
  }
  CHECK(f.violations() == 0);
}

static void test_io_error() {
  Flash4 f;
  KvStore<4> kv(f);
  CHECK(kv.begin() == KvResult::kOk);
  Model m;
  KvResult r;
  for (int i = 0; i < 20; ++i) apply_put(kv, m, (uint16_t)(i % 5), make_value(20, (uint32_t)i), r);
  f.fail_programs(true);
  CHECK(kv.put_u32(30, 1) == KvResult::kIoError);
  CHECK(!kv.ready());
  CHECK(kv.put_u32(30, 1) == KvResult::kNotReady);
  CHECK(kv.get_u32_or(30, 5) == 5);
  f.fail_programs(false);
  CHECK(kv.begin() == KvResult::kOk);  // re-mount repairs
  CHECK(matches(kv, m, 31));
  CHECK(!kv.contains(30));
  CHECK(kv.put_u32(30, 1) == KvResult::kOk && kv.get_u32_or(30, 0) == 1);
  m[30] = Bytes(4, 0);
  m[30][0] = 1;
  // erase failure during compaction
  for (int i = 0; i < 40; ++i) apply_put(kv, m, (uint16_t)(i % 5), make_value(40, (uint32_t)i + 100), r);
  f.fail_erases(true);
  bool sawio = false;
  for (int i = 0; i < 200 && !sawio; ++i) {
    r = kv.put(1, make_value(60, (uint32_t)i).data(), 60);
    if (r == KvResult::kIoError) sawio = true;
    else if (r == KvResult::kOk) m[1] = make_value(60, (uint32_t)i);
  }
  CHECK(sawio);
  f.fail_erases(false);
  CHECK(kv.begin() == KvResult::kOk);
  uint8_t buf[80];
  size_t n = 0;
  CHECK(kv.get(1, buf, sizeof buf, &n) == KvResult::kOk);
  // key 1 holds the last acknowledged value
  CHECK(n == m[1].size() && memcmp(buf, &m[1][0], n) == 0);
  CHECK(matches(kv, m, 31));
  CHECK(f.violations() == 0);
}

static void test_bit_rot() {
  // Random bit flips: the store must stay usable and never return garbage lengths.
  for (int round = 0; round < 300; ++round) {
    RamFlash<256, 4, 4> f;
    {
      KvStore<4> kv(f);
      kv.begin();
      for (int i = 0; i < 60; ++i) {
        Bytes v = make_value(rnd_below(40), rnd32());
        kv.put((uint16_t)rnd_below(8), v.empty() ? 0 : &v[0], v.size());
      }
    }
    for (int k = 0; k < 1 + (int)rnd_below(6); ++k)
      f.flip_bit(rnd_below(4), rnd_below(256), rnd_below(8));
    KvStore<4> kv(f);
    KvResult r = kv.begin();
    CHECK(r == KvResult::kOk || r == KvResult::kCorrupt || r == KvResult::kIoError);
    if (r != KvResult::kOk) continue;
    uint8_t buf[600];
    for (uint16_t key = 0; key < 8; ++key) {
      size_t n = 0;
      KvResult g = kv.get(key, buf, sizeof buf, &n);
      CHECK(g == KvResult::kOk || g == KvResult::kNotFound);
      if (g == KvResult::kOk) CHECK(n <= 40);
    }
    // and it still accepts writes
    Bytes v = make_value(20, 77);
    KvResult p = kv.put(3, &v[0], v.size());
    CHECK(p == KvResult::kOk || p == KvResult::kFull || p == KvResult::kCorrupt);
    if (p == KvResult::kOk) {
      uint8_t b2[20];
      CHECK(kv.get(3, b2, sizeof b2) == KvResult::kOk && memcmp(b2, &v[0], 20) == 0);
    }
  }
}

// Garbage in a flash that never held a store: mount must not crash.
static void test_garbage_flash() {
  for (int round = 0; round < 200; ++round) {
    RamFlash<256, 4, 4> f;
    for (size_t s = 0; s < 4; ++s)
      for (size_t i = 0; i < 256; ++i)
        if (rnd_below(3) == 0) f.raw(s)[i] = (uint8_t)rnd32();
    f.mark_all_dirty_from_content();
    KvStore<4> kv(f);
    CHECK(kv.begin() == KvResult::kOk);  // cleans up and formats
    CHECK(kv.put_u32(1, 5) == KvResult::kOk && kv.get_u32_or(1, 0) == 5);
    CHECK(f.violations() == 0);
  }
}

// ---- power-fail test ----------------------------------------------------------

struct Op {
  bool remove;
  uint16_t key;
  size_t len;
  uint32_t seed;
};

static std::vector<Op> make_ops(int count, uint16_t key_space, size_t max_len) {
  std::vector<Op> ops;
  for (int i = 0; i < count; ++i) {
    Op o;
    o.remove = rnd_below(100) < 20;
    o.key = (uint16_t)rnd_below(key_space);
    o.len = rnd_below((uint32_t)max_len + 1);
    o.seed = rnd32();
    ops.push_back(o);
  }
  return ops;
}

static void apply_model(const Op& op, Model& m) {
  if (op.remove) m.erase(op.key);
  else m[op.key] = make_value(op.len, op.seed);
}

// Runs one operation on the store; `after` becomes the model as if it had completed.
template <typename KV>
static KvResult do_op(KV& kv, const Op& op, Model& after) {
  apply_model(op, after);
  if (op.remove) {
    KvResult r = kv.remove(op.key);
    return r == KvResult::kNotFound ? KvResult::kOk : r;
  }
  Bytes v = make_value(op.len, op.seed);
  return kv.put(op.key, v.empty() ? 0 : &v[0], v.size());
}

// One power-cut trial; returns false (after printing why) if the store lost consistency.
template <size_t SS, size_t NS, size_t U>
static bool power_fail_trial(const char* name, const std::vector<Op>& ops, uint16_t key_space,
                             uint32_t k, uint32_t seed, bool verbose) {
  typedef RamFlash<SS, NS, U> F;
  uint32_t tr = k * 2654435761u + seed * 40503u + 1;  // local rng so a trial can be replayed
  struct Local {
    static uint32_t next(uint32_t& s) {
      s ^= s << 13;
      s ^= s >> 17;
      s ^= s << 5;
      return s;
    }
  };
  F f;
  Model model;
  size_t fail_i = ops.size();
  Model before, after;
  {
    KvStore<NS> kv(f);
    CHECK(kv.begin() == KvResult::kOk);
    f.reset_counters();
    f.set_trace(verbose);
    f.power_fail_at(k, seed * 7919u + k);
    for (size_t i = 0; i < ops.size(); ++i) {
      before = model;
      after = model;
      KvResult r = do_op(kv, ops[i], after);
      if (r == KvResult::kOk) {
        model = after;
        continue;
      }
      if (r == KvResult::kFull) continue;  // refused; nothing changed
      fail_i = i;
      CHECK(f.dead());
      CHECK(r == KvResult::kIoError);
      if (verbose) printf("  -- power cut during op %zu (%s key %u)\n", i, ops[i].remove ? "remove" : "put", ops[i].key);
      break;
    }
  }
  if (fail_i == ops.size()) {
    CHECK(!f.dead());
    return true;
  }
  // reboot, possibly cutting power again while recovering
  f.power_cycle();
  KvStore<NS> kv(f);
  KvResult br = KvResult::kIoError;
  for (uint32_t j = 1; br != KvResult::kOk && j < 60; ++j) {
    if (Local::next(tr) % 4 == 0) {
      f.power_fail_at(j, k + j + seed);  // another power cut during recovery
      br = kv.begin();
      if (f.dead()) {
        if (verbose) printf("  -- power cut again during recovery\n");
        f.power_cycle();
      } else {
        f.power_fail_at(0, 0);
      }
    } else {
      f.power_fail_at(0, 0);
      if (verbose) printf("  -- recovery\n");
      br = kv.begin();
    }
    if (br != KvResult::kOk) f.power_cycle();
  }
  CHECK(br == KvResult::kOk);
  if (br != KvResult::kOk) {
    printf("  (%s: begin failed after cut %u seed %u)\n", name, k, seed);
    return false;
  }
  const bool is_before = matches(kv, before, key_space);
  const bool is_after = matches(kv, after, key_space);
  CHECK(is_before || is_after);
  if (!(is_before || is_after)) {
    g_debug = true;
    printf("  before:\n");
    matches(kv, before, key_space);
    printf("  after:\n");
    matches(kv, after, key_space);
    g_debug = false;
    printf("  (%s: cut %u seed %u op %zu left a mixed state)\n", name, k, seed, fail_i);
    return false;
  }
  Model cur = is_after ? after : before;
  // carry on with the rest of the scenario
  f.violations();
  for (size_t i = fail_i + 1; i < ops.size(); ++i) {
    Model next = cur;
    KvResult r = do_op(kv, ops[i], next);
    CHECK(r == KvResult::kOk || r == KvResult::kFull);
    if (r != KvResult::kOk && r != KvResult::kFull) {
      printf("  (%s: cut %u seed %u: op %zu after recovery returned %d)\n", name, k, seed, i, (int)r);
      f.set_trace(false);
      return false;
    }
    if (r == KvResult::kOk) cur = next;
  }
  CHECK(matches(kv, cur, key_space));
  // and a second reboot sees the same
  KvStore<NS> kv3(f);
  CHECK(kv3.begin() == KvResult::kOk);
  CHECK(matches(kv3, cur, key_space));
  CHECK(f.violations() == 0);
  return true;
}

template <size_t SS, size_t NS, size_t U>
static void power_fail_sweep(const char* name, int op_count, uint16_t key_space, size_t max_len,
                             uint32_t stride, uint32_t seeds) {
  typedef RamFlash<SS, NS, U> F;
  std::vector<Op> ops = make_ops(op_count, key_space, max_len);

  // reference run: how many write operations does the scenario take?
  uint32_t total = 0;
  {
    F f;
    KvStore<NS> kv(f);
    CHECK(kv.begin() == KvResult::kOk);
    f.reset_counters();
    f.power_fail_at(0xFFFFFFFFu, 1);  // count ops without failing
    Model m;
    for (size_t i = 0; i < ops.size(); ++i) {
      Model after = m;
      KvResult r = do_op(kv, ops[i], after);
      CHECK(r == KvResult::kOk || r == KvResult::kFull);
      if (r == KvResult::kOk) m = after;
    }
    total = f.ops();
    CHECK(f.total_erases() > NS);  // the scenario really wraps around the flash
  }

  unsigned long trials = 0;
  for (uint32_t k = 1; k <= total + 1; k += stride) {
    for (uint32_t seed = 1; seed <= seeds; ++seed) {
      ++trials;
      if (!power_fail_trial<SS, NS, U>(name, ops, key_space, k, seed, false)) {
        printf("  replaying with a flash trace:\n");
        power_fail_trial<SS, NS, U>(name, ops, key_space, k, seed, true);
        return;
      }
    }
  }
  printf("  power-fail %-22s %6u write ops, %lu cut points tested\n", name, total, trials);
}

static void test_power_fail() {
  power_fail_sweep<256, 4, 4>("256B x4, unit 4", 300, 8, 40, 1, 2);
  power_fail_sweep<256, 4, 1>("256B x4, unit 1", 300, 8, 40, 1, 2);
  power_fail_sweep<192, 3, 8>("192B x3, unit 8", 300, 6, 30, 1, 2);
  power_fail_sweep<128, 2, 16>("128B x2, unit 16", 120, 4, 24, 1, 2);
  power_fail_sweep<256, 5, 2>("256B x5, unit 2", 400, 10, 60, 1, 2);
  power_fail_sweep<512, 3, 4>("512B x3, unit 4", 200, 12, 120, 3, 2);
}

int main() {
  test_basics();
  test_bad_geometry();
  test_skip_identical();
  test_value_lengths();
  test_full();
  test_window();
  test_io_error();
  test_garbage_flash();
  test_bit_rot();
  test_fuzz();
  test_power_fail();
  printf("%lu checks, %lu failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
