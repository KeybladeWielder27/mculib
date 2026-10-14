// Host-side tests for the filter library: fixed.hpp, filters.hpp, debounce.hpp.
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_filter.cpp -o test_filter && ./test_filter

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "debounce.hpp"
#include "filters.hpp"
#include "fixed.hpp"

static unsigned long g_checks = 0;
static unsigned long g_fails = 0;

#define CHECK(...)                                                       \
  do {                                                                   \
    ++g_checks;                                                          \
    if (!(__VA_ARGS__)) {                                                \
      ++g_fails;                                                         \
      if (g_fails < 40) printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__); \
    }                                                                    \
  } while (0)

// ---------------------------------------------------------------------------
// Deterministic random numbers (xorshift64*)
// ---------------------------------------------------------------------------
static uint64_t g_rng = 0x9E3779B97F4A7C15ull;
static uint64_t rnd64() {
  g_rng ^= g_rng >> 12;
  g_rng ^= g_rng << 25;
  g_rng ^= g_rng >> 27;
  return g_rng * 0x2545F4914F6CDD1Dull;
}
static uint32_t rnd32() { return (uint32_t)(rnd64() >> 32); }
static uint32_t rnd_below(uint32_t n) { return rnd32() % n; }

// A random T, biased toward the interesting values.
template <typename T>
static T rnd_value() {
  const uint32_t k = rnd_below(10);
  const T mx = mcu::detail::int_max<T>();
  const T mn = mcu::detail::int_min<T>();
  switch (k) {
    case 0: return mx;
    case 1: return mn;
    case 2: return (T)(mx - 1);
    case 3: return (T)(mn + 1);
    case 4: return (T)0;
    case 5: return (T)1;
    default: return (T)rnd64();
  }
}

// ---------------------------------------------------------------------------
// fixed.hpp: limits, saturating math
// ---------------------------------------------------------------------------
static void test_limits() {
  CHECK(mcu::detail::int_max<int8_t>() == 127);
  CHECK(mcu::detail::int_min<int8_t>() == -128);
  CHECK(mcu::detail::int_max<uint8_t>() == 255);
  CHECK(mcu::detail::int_min<uint8_t>() == 0);
  CHECK(mcu::detail::int_max<int16_t>() == 32767);
  CHECK(mcu::detail::int_min<int16_t>() == -32768);
  CHECK(mcu::detail::int_max<uint16_t>() == 65535);
  CHECK(mcu::detail::int_max<int32_t>() == INT32_MAX);
  CHECK(mcu::detail::int_min<int32_t>() == INT32_MIN);
  CHECK(mcu::detail::int_max<uint32_t>() == UINT32_MAX);
  CHECK(mcu::detail::int_max<int64_t>() == INT64_MAX);
  CHECK(mcu::detail::int_min<int64_t>() == INT64_MIN);
  CHECK(mcu::detail::int_max<uint64_t>() == UINT64_MAX);

  CHECK(mcu::detail::IntTraits<int8_t>::is_signed == 1);
  CHECK(mcu::detail::IntTraits<uint8_t>::is_signed == 0);
  CHECK(mcu::detail::IntTraits<int64_t>::is_signed == 1);
  CHECK(mcu::detail::IntTraits<uint16_t>::is_signed == 0);
  CHECK(sizeof(mcu::detail::IntTraits<int16_t>::wide) >= 4);
  CHECK(sizeof(mcu::detail::IntTraits<uint32_t>::wide) == 8);

  // constexpr usable
  static_assert(mcu::detail::int_max<int16_t>() == 32767, "constexpr");
  static_assert(mcu::clamp(5, 0, 3) == 3, "constexpr");
}

template <typename T>
static void sat_check(T a, T b) {
  const int64_t mx = (int64_t)mcu::detail::int_max<T>();
  const int64_t mn = (int64_t)mcu::detail::int_min<T>();
  const int64_t s = (int64_t)a + (int64_t)b;
  const int64_t d = (int64_t)a - (int64_t)b;
  CHECK(mcu::sat_add<T>(a, b) == (T)(s > mx ? mx : (s < mn ? mn : s)));
  CHECK(mcu::sat_sub<T>(a, b) == (T)(d > mx ? mx : (d < mn ? mn : d)));
}

template <typename T>
static void test_sat_exhaustive8() {
  for (int a = (int)mcu::detail::int_min<T>(); a <= (int)mcu::detail::int_max<T>(); ++a)
    for (int b = (int)mcu::detail::int_min<T>(); b <= (int)mcu::detail::int_max<T>(); ++b)
      sat_check<T>((T)a, (T)b);
}

template <typename T>
static void test_sat_random(int n) {
  for (int i = 0; i < n; ++i) sat_check<T>(rnd_value<T>(), rnd_value<T>());
}

static void test_sat() {
  test_sat_exhaustive8<int8_t>();
  test_sat_exhaustive8<uint8_t>();
  test_sat_random<int16_t>(200000);
  test_sat_random<uint16_t>(200000);
  test_sat_random<int32_t>(200000);
  test_sat_random<uint32_t>(200000);

  CHECK(mcu::sat_add<uint8_t>(200, 100) == 255);
  CHECK(mcu::sat_sub<uint8_t>(10, 20) == 0);
  CHECK(mcu::sat_add<int8_t>(100, 100) == 127);
  CHECK(mcu::sat_sub<int8_t>(-100, 100) == -128);
  CHECK(mcu::sat_add<int8_t>(-100, -100) == -128);
  CHECK(mcu::sat_sub<int8_t>(100, -100) == 127);
  CHECK(mcu::sat_add<int64_t>(INT64_MAX, 1) == INT64_MAX);
  CHECK(mcu::sat_sub<int64_t>(INT64_MIN, 1) == INT64_MIN);
  CHECK(mcu::sat_add<int64_t>(INT64_MIN, INT64_MAX) == -1);
  CHECK(mcu::sat_add<uint64_t>(UINT64_MAX, 1) == UINT64_MAX);
  CHECK(mcu::sat_sub<uint64_t>(0, 1) == 0);
  CHECK(mcu::sat_add<int32_t>(INT32_MIN, INT32_MIN) == INT32_MIN);
}

// ---------------------------------------------------------------------------
// div_round, clamp, map_range
// ---------------------------------------------------------------------------
static void test_div_round() {
  using mcu::detail::div_round;
  CHECK(div_round<int32_t>(7, 2) == 4);
  CHECK(div_round<int32_t>(-7, 2) == -4);
  CHECK(div_round<int32_t>(5, 3) == 2);
  CHECK(div_round<int32_t>(-5, 3) == -2);
  CHECK(div_round<int32_t>(4, 3) == 1);
  CHECK(div_round<int32_t>(-4, 3) == -1);
  CHECK(div_round<int32_t>(0, 5) == 0);
  CHECK(div_round<uint32_t>(7, 2) == 4);
  CHECK(div_round<uint32_t>(6, 4) == 2);
  CHECK(div_round<uint32_t>(5, 4) == 1);
  for (int n = -200; n <= 200; ++n)
    for (int d = 1; d <= 17; ++d) {
      const double q = (double)n / (double)d;
      const int expect = q >= 0 ? (int)(q + 0.5) : -(int)(-q + 0.5);
      CHECK(div_round<int32_t>(n, d) == expect);
    }
}

static void test_map_range() {
  using mcu::map_range;
  CHECK(map_range(512, 0, 1023, 0, 3300) == 1652);
  CHECK(map_range(0, 0, 1023, 0, 3300) == 0);
  CHECK(map_range(1023, 0, 1023, 0, 3300) == 3300);
  CHECK(map_range(25, 0, 100, 255, 0) == 191);
  CHECK(map_range(-5, 0, 100, 0, 10) == 0);       // clamped low
  CHECK(map_range(500, 0, 100, 0, 10) == 10);     // clamped high
  CHECK(map_range(5, 10, 0, 0, 100) == 50);       // reversed input
  CHECK(map_range(2, 10, 0, 0, 100) == 80);
  CHECK(map_range(50, 0, 100, -100, 100) == 0);   // negative output
  CHECK(map_range(5, 7, 7, 3, 9) == 3);           // degenerate input range
  CHECK(map_range(1, 0, 3, 0, 1) == 0);           // 0.333 -> 0
  CHECK(map_range(2, 0, 3, 0, 1) == 1);           // 0.667 -> 1
  CHECK(map_range(-1, -3, 0, 0, 1) == 1);         // negative inputs round the same way
  // extremes must not overflow
  CHECK(map_range(INT32_MAX, 0, INT32_MAX, 0, INT32_MAX) == INT32_MAX);
  CHECK(map_range(INT32_MIN, INT32_MIN, INT32_MAX, INT32_MIN, INT32_MAX) == INT32_MIN);
  CHECK(map_range(0, INT32_MIN, INT32_MAX, INT32_MIN, INT32_MAX) == 0 ||
        map_range(0, INT32_MIN, INT32_MAX, INT32_MIN, INT32_MAX) == -1);

  // Random against a double reference (spans small enough that double is exact enough).
  for (int i = 0; i < 100000; ++i) {
    const int32_t in_lo = (int32_t)rnd_below(2001) - 1000;
    const int32_t in_hi = (int32_t)rnd_below(2001) - 1000;
    const int32_t out_lo = (int32_t)rnd_below(100001) - 50000;
    const int32_t out_hi = (int32_t)rnd_below(100001) - 50000;
    const int32_t x = (int32_t)rnd_below(3001) - 1500;
    const int32_t got = map_range(x, in_lo, in_hi, out_lo, out_hi);
    if (in_lo == in_hi) {
      CHECK(got == out_lo);
      continue;
    }
    const int32_t lo = in_lo < in_hi ? in_lo : in_hi;
    const int32_t hi = in_lo < in_hi ? in_hi : in_lo;
    const int32_t xc = x < lo ? lo : (x > hi ? hi : x);
    const double f = (double)(xc - in_lo) / (double)(in_hi - in_lo);
    const double exact = (double)out_lo + f * (double)(out_hi - out_lo);
    CHECK(std::abs((double)got - exact) <= 0.5 + 1e-9);
    const int32_t omin = out_lo < out_hi ? out_lo : out_hi;
    const int32_t omax = out_lo < out_hi ? out_hi : out_lo;
    CHECK(got >= omin && got <= omax);
  }
}

// ---------------------------------------------------------------------------
// isqrt
// ---------------------------------------------------------------------------
static void test_isqrt() {
  for (uint32_t n = 0; n < 200000; ++n) {
    const uint32_t r = mcu::isqrt(n);
    CHECK((uint64_t)r * r <= n && (uint64_t)(r + 1) * (r + 1) > n);
  }
  for (uint32_t k = 0; k <= 65535; ++k) {  // perfect squares and neighbours
    const uint32_t sq = k * k;
    CHECK(mcu::isqrt(sq) == k);
    if (sq > 0) CHECK(mcu::isqrt(sq - 1) == k - 1);
    if (k > 0 && k < 65535) CHECK(mcu::isqrt(sq + 1) == k);
  }
  CHECK(mcu::isqrt(0xFFFFFFFFu) == 65535u);
  CHECK(mcu::isqrt(1000000u) == 1000u);
  for (int i = 0; i < 200000; ++i) {
    const uint32_t n = rnd32();
    const uint32_t r = mcu::isqrt(n);
    CHECK((uint64_t)r * r <= n && (uint64_t)(r + 1) * (r + 1) > n);
  }

  CHECK(mcu::isqrt64(0) == 0);
  CHECK(mcu::isqrt64(UINT64_MAX) == 0xFFFFFFFFu);
  CHECK(mcu::isqrt64(1ull << 62) == (1u << 31));
  for (int i = 0; i < 200000; ++i) {
    uint64_t n = rnd64();
    n >>= rnd_below(64);
    const uint64_t r = mcu::isqrt64(n);
    CHECK(r * r <= n);
    CHECK(r == 0xFFFFFFFFull || (r + 1) * (r + 1) > n);
  }
}

// ---------------------------------------------------------------------------
// Q<F>
// ---------------------------------------------------------------------------
typedef mcu::Q<16> Q16;

static void test_q() {
  CHECK(Q16::from_int(3).raw() == 3 * 65536);
  CHECK(Q16::from_int(-3).raw() == -3 * 65536);
  CHECK(Q16::from_int(3).to_int() == 3);
  CHECK(Q16::from_ratio(3, 4).raw() == 49152);
  CHECK(Q16::from_ratio(1, 3).raw() == 21845);  // 21845.33 rounds down
  CHECK(Q16::from_ratio(2, 3).raw() == 43691);  // 43690.67 rounds up
  CHECK(Q16::from_ratio(-2, 3).raw() == -43691);
  CHECK(Q16::from_ratio(1, 0) == Q16::max_value());
  CHECK(Q16::from_ratio(-1, 0) == Q16::min_value());
  CHECK(Q16::from_ratio(0, 0).raw() == 0);

  CHECK((Q16::from_ratio(3, 4) * Q16::from_int(100)).to_int() == 75);
  CHECK((Q16::from_int(7) / Q16::from_int(2)).raw() == 7 * 65536 / 2);
  CHECK((Q16::from_int(7) / Q16::from_int(2)).to_int() == 4);    // 3.5 -> 4
  CHECK((Q16::from_int(-7) / Q16::from_int(2)).to_int() == -4);  // -3.5 -> -4
  CHECK((Q16::from_int(10) / Q16::from_int(-4)).raw() == -163840);  // -2.5
  CHECK((Q16::from_int(10) / Q16()) == Q16::max_value());
  CHECK((Q16::from_int(-10) / Q16()) == Q16::min_value());

  CHECK((Q16::from_int(2) + Q16::from_int(3)).to_int() == 5);
  CHECK((Q16::from_int(2) - Q16::from_int(3)).to_int() == -1);
  CHECK((-Q16::from_int(2)).to_int() == -2);
  CHECK(Q16::from_int(-5).abs().to_int() == 5);
  CHECK(Q16::from_int(5).abs().to_int() == 5);

  // saturation
  CHECK(Q16::from_int(30000) == Q16::from_int(30000));
  CHECK(Q16::from_int(40000) == Q16::max_value());
  CHECK(Q16::from_int(-40000) == Q16::min_value());
  CHECK(Q16::max_value() + Q16::from_int(1) == Q16::max_value());
  CHECK(Q16::min_value() - Q16::from_int(1) == Q16::min_value());
  CHECK(-Q16::min_value() == Q16::max_value());
  CHECK(Q16::from_int(20000) * Q16::from_int(20000) == Q16::max_value());
  CHECK(Q16::from_int(20000) * Q16::from_int(-20000) == Q16::min_value());
  CHECK(Q16::min_value().abs() == Q16::max_value());

  // multiplication rounds to nearest
  CHECK((Q16::from_raw(1) * Q16::from_ratio(1, 2)).raw() == 1);   // 0.5 raw -> 1 (half away)
  CHECK((Q16::from_raw(-1) * Q16::from_ratio(1, 2)).raw() == -1);
  CHECK((Q16::from_raw(1) * Q16::from_ratio(1, 4)).raw() == 0);
  CHECK((Q16::from_raw(3) * Q16::from_ratio(1, 4)).raw() == 1);   // 0.75 -> 1

  // compound ops and comparisons
  Q16 a = Q16::from_int(10);
  a += Q16::from_int(5);
  a -= Q16::from_int(3);
  a *= Q16::from_int(2);
  a /= Q16::from_int(4);
  CHECK(a.to_int() == 6);
  CHECK(Q16::from_int(1) < Q16::from_int(2));
  CHECK(Q16::from_int(2) > Q16::from_int(1));
  CHECK(Q16::from_int(2) <= Q16::from_int(2));
  CHECK(Q16::from_int(2) >= Q16::from_int(2));
  CHECK(Q16::from_int(2) != Q16::from_int(3));

  // constexpr from_double
  static_assert(Q16::from_double(0.75).raw() == 49152, "constexpr from_double");
  static_assert(Q16::from_double(-0.75).raw() == -49152, "constexpr from_double");
  static_assert(Q16::from_raw(5).raw() == 5, "constexpr from_raw");
  CHECK(Q16::from_double(1.5).to_double() == 1.5);
  CHECK(Q16::from_int(0).to_double() == 0.0);

  // other widths
  typedef mcu::Q<8> Q8;
  CHECK(Q8::from_ratio(1, 2).raw() == 128);
  CHECK(Q8::from_int(10000000) == Q8::max_value());
  typedef mcu::Q<30> Q30;
  CHECK(Q30::from_int(1).raw() == (1 << 30));
  CHECK(Q30::from_int(2) == Q30::max_value());  // 2.0 does not fit: range is about +/-2
  typedef mcu::Q<1> Q1;
  CHECK(Q1::from_ratio(3, 2).raw() == 3);

  // random arithmetic against a 64-bit reference
  for (int i = 0; i < 200000; ++i) {
    const int32_t x = (int32_t)rnd32();
    const int32_t y = (int32_t)rnd32();
    const Q16 qx = Q16::from_raw(x), qy = Q16::from_raw(y);
    const int64_t s = (int64_t)x + y;
    CHECK((qx + qy).raw() == (int32_t)(s > INT32_MAX ? INT32_MAX : (s < INT32_MIN ? INT32_MIN : s)));
    const int64_t d = (int64_t)x - y;
    CHECK((qx - qy).raw() == (int32_t)(d > INT32_MAX ? INT32_MAX : (d < INT32_MIN ? INT32_MIN : d)));
    // multiply: exact product / 65536, nearest, halves away from zero
    const int64_t p = (int64_t)x * y;
    int64_t pr = p >= 0 ? (p + 32768) / 65536 : -((-p + 32768) / 65536);
    pr = pr > INT32_MAX ? INT32_MAX : (pr < INT32_MIN ? INT32_MIN : pr);
    CHECK((qx * qy).raw() == (int32_t)pr);
  }
}

// ---------------------------------------------------------------------------
// MovingAverage
// ---------------------------------------------------------------------------
template <typename T, size_t N>
static void ma_vs_naive(int steps) {
  mcu::MovingAverage<T, N> ma;
  std::vector<T> hist;
  CHECK(ma.value() == 0);
  CHECK(!ma.full());
  for (int i = 0; i < steps; ++i) {
    const T x = rnd_value<T>();
    hist.push_back(x);
    const T got = ma.update(x);
    const size_t k = hist.size() < N ? hist.size() : N;
    typedef typename mcu::detail::IntBySize<8, mcu::detail::IntTraits<T>::is_signed != 0>::wide W;
    W sum = 0;
    for (size_t j = hist.size() - k; j < hist.size(); ++j) sum += (W)hist[j];
    const T expect = (T)mcu::detail::div_round<W>(sum, (W)k);
    CHECK(got == expect);
    CHECK(ma.value() == expect);
    CHECK(ma.count() == k);
    CHECK(ma.full() == (k == N));
    CHECK((W)ma.sum() == sum);
  }
}

static void test_moving_average() {
  ma_vs_naive<uint8_t, 1>(500);
  ma_vs_naive<uint8_t, 5>(5000);
  ma_vs_naive<int8_t, 4>(5000);
  ma_vs_naive<int16_t, 8>(5000);
  ma_vs_naive<uint16_t, 16>(5000);
  ma_vs_naive<int32_t, 3>(5000);
  ma_vs_naive<uint32_t, 7>(5000);
}

static void test_moving_average_basic() {
  mcu::MovingAverage<int, 4> ma;
  CHECK(ma.update(10) == 10);          // first output equals first input
  CHECK(ma.update(20) == 15);
  CHECK(ma.update(30) == 20);
  CHECK(ma.update(40) == 25);
  CHECK(ma.full());
  CHECK(ma.update(50) == 35);          // 20+30+40+50 = 140 / 4
  CHECK(mcu::MovingAverage<int, 4>::capacity() == 4);
  ma.reset();
  CHECK(ma.count() == 0 && ma.value() == 0 && !ma.full());
  CHECK(ma.update(7) == 7);

  mcu::MovingAverage<int16_t, 2> neg;
  neg.update(-1);
  CHECK(neg.update(-2) == -2);   // -1.5 rounds away from zero
  mcu::MovingAverage<int16_t, 2> pos;
  pos.update(1);
  CHECK(pos.update(2) == 2);     // 1.5 -> 2
}

// ---------------------------------------------------------------------------
// Ema
// ---------------------------------------------------------------------------
template <typename T, unsigned S>
static void ema_converge(T start, T target) {
  mcu::Ema<T, S> e;
  CHECK(e.update(start) == start);  // first sample primes
  CHECK(e.primed());
  T prev = start;
  bool reached = false;
  for (int i = 0; i < 20000; ++i) {
    const T y = e.update(target);
    if (target >= start) {
      CHECK(y >= prev && y <= target);  // monotonic, never overshoots
    } else {
      CHECK(y <= prev && y >= target);
    }
    prev = y;
    if (y == target) {
      reached = true;
      break;
    }
  }
  CHECK(reached);
  for (int i = 0; i < 50; ++i) CHECK(e.update(target) == target);  // stays put
}

static void test_ema() {
  ema_converge<int16_t, 1>(0, 1000);
  ema_converge<int16_t, 4>(1000, -1000);
  ema_converge<int16_t, 6>(-32768, 32767);
  ema_converge<int16_t, 6>(32767, -32768);
  ema_converge<uint8_t, 3>(0, 255);
  ema_converge<uint8_t, 3>(255, 0);
  ema_converge<uint16_t, 5>(0, 65535);
  ema_converge<uint16_t, 5>(65535, 0);
  ema_converge<int8_t, 2>(-128, 127);
  ema_converge<int8_t, 2>(127, -128);
  ema_converge<int32_t, 4>(INT32_MIN, INT32_MAX);
  ema_converge<int32_t, 4>(INT32_MAX, INT32_MIN);
  ema_converge<uint32_t, 3>(0, UINT32_MAX);
  ema_converge<uint32_t, 3>(UINT32_MAX, 0);

  // random start/target pairs
  for (int i = 0; i < 300; ++i) {
    const int16_t a = (int16_t)rnd32();
    const int16_t b = (int16_t)rnd32();
    ema_converge<int16_t, 3>(a, b);
    ema_converge<int16_t, 1>(a, b);
    ema_converge<int16_t, 8>(a, b);
  }

  // Step response of Shift 2 (alpha 1/4) tracks the float filter to within rounding.
  mcu::Ema<int32_t, 2> e;
  e.update(0);
  double f = 0;
  for (int i = 0; i < 40; ++i) {
    f += (1000.0 - f) / 4.0;
    const int32_t y = e.update(1000);
    CHECK(std::abs((double)y - f) <= 4.0);
  }

  // reset
  mcu::Ema<int, 3> r;
  CHECK(r.value() == 0 && !r.primed());
  r.reset(500);
  CHECK(r.primed() && r.value() == 500);
  CHECK(r.update(500) == 500);
  r.reset();
  CHECK(!r.primed());
  CHECK(r.update(77) == 77);

  // heavy smoothing barely moves for one outlier
  mcu::Ema<int16_t, 5> h;
  h.update(100);
  for (int i = 0; i < 10; ++i) h.update(100);
  CHECK(h.update(1000) < 140);
}

// ---------------------------------------------------------------------------
// MedianFilter
// ---------------------------------------------------------------------------
template <typename T, size_t N>
static void median_vs_naive(int steps) {
  mcu::MedianFilter<T, N> m;
  std::vector<T> hist;
  for (int i = 0; i < steps; ++i) {
    // small alphabet gives plenty of duplicates
    const T x = (i % 3 == 0) ? rnd_value<T>() : (T)(rnd_below(7));
    hist.push_back(x);
    const size_t k = hist.size() < N ? hist.size() : N;
    std::vector<T> w(hist.end() - (std::ptrdiff_t)k, hist.end());
    for (size_t a = 1; a < w.size(); ++a)  // insertion sort (kept simple on purpose)
      for (size_t b = a; b > 0 && w[b - 1] > w[b]; --b) std::swap(w[b - 1], w[b]);
    const T expect = w[(k - 1) / 2];
    CHECK(m.update(x) == expect);
    CHECK(m.value() == expect);
    CHECK(m.count() == k);
  }
}

static void test_median() {
  median_vs_naive<int16_t, 1>(200);
  median_vs_naive<int16_t, 2>(2000);
  median_vs_naive<int16_t, 3>(5000);
  median_vs_naive<uint8_t, 5>(5000);
  median_vs_naive<int8_t, 7>(5000);
  median_vs_naive<int32_t, 9>(5000);
  median_vs_naive<uint32_t, 15>(5000);
  median_vs_naive<uint16_t, 32>(5000);

  mcu::MedianFilter<int, 3> m;
  CHECK(m.value() == 0);
  CHECK(m.update(10) == 10);
  CHECK(m.update(1000) == 10);   // two samples: lower median
  CHECK(m.update(12) == 12);     // spike rejected
  CHECK(m.update(11) == 12);
  CHECK(m.update(13) == 12);
  CHECK(m.full());
  m.reset();
  CHECK(m.count() == 0 && m.value() == 0);
  CHECK(m.update(5) == 5);

  // Isolated spikes vanish with N = 3.
  mcu::MedianFilter<int, 3> sp;
  sp.update(100);
  sp.update(100);
  CHECK(sp.update(5000) == 100);
  CHECK(sp.update(100) == 100);
  CHECK(sp.update(100) == 100);
}

// ---------------------------------------------------------------------------
// SlewLimiter
// ---------------------------------------------------------------------------
static void test_slew() {
  mcu::SlewLimiter<int> s(10, 5);
  CHECK(s.update(100) == 100);   // first sample passes
  CHECK(s.update(200) == 110);
  CHECK(s.update(200) == 120);
  CHECK(s.update(125) == 125);   // within the limit: exact
  CHECK(s.update(0) == 120);
  CHECK(s.update(118) == 118);
  CHECK(s.update(1000) == 128);
  s.set_limits(100, 100);
  CHECK(s.update(1000) == 228);
  s.reset();
  CHECK(s.update(-50) == -50);
  s.reset(7);
  CHECK(s.update(8) == 8);

  // full-range steps for int8 (differences up to 255 exceed int8 range)
  mcu::SlewLimiter<int8_t> a(255);
  CHECK(a.update(-128) == -128);
  CHECK(a.update(127) == 127);
  CHECK(a.update(-128) == -128);
  mcu::SlewLimiter<int8_t> b(100, 100);
  CHECK(b.update(-128) == -128);
  CHECK(b.update(127) == -28);
  CHECK(b.update(127) == 72);
  CHECK(b.update(127) == 127);
  CHECK(b.update(-128) == 27);
  CHECK(b.update(-128) == -73);
  CHECK(b.update(-128) == -128);

  mcu::SlewLimiter<uint8_t> u(50, 50);
  u.update(0);
  CHECK(u.update(255) == 50);
  u.reset(250);
  CHECK(u.update(255) == 255);
  CHECK(u.update(0) == 205);

  mcu::SlewLimiter<int32_t> w(UINT32_MAX);
  w.update(INT32_MIN);
  CHECK(w.update(INT32_MAX) == INT32_MAX);

  // zero limit = freeze in that direction
  mcu::SlewLimiter<int> up_only(5, 0);
  up_only.update(10);
  CHECK(up_only.update(0) == 10);
  CHECK(up_only.update(100) == 15);

  // random: never moves more than the limit, never overshoots, reaches target
  for (int i = 0; i < 200; ++i) {
    mcu::SlewLimiter<int16_t> r((uint16_t)(1 + rnd_below(2000)), (uint16_t)(1 + rnd_below(2000)));
    int16_t prev = (int16_t)rnd32();
    r.update(prev);
    const int16_t target = (int16_t)rnd32();
    bool reached = false;
    for (int k = 0; k < 100 && !reached; ++k) {
      const int16_t y = r.update(target);
      if (target > prev) CHECK(y >= prev && y <= target);
      else CHECK(y <= prev && y >= target);
      prev = y;
      reached = (y == target);
    }
  }
}

// ---------------------------------------------------------------------------
// Schmitt, RunningStats
// ---------------------------------------------------------------------------
static void test_schmitt() {
  mcu::Schmitt<int> s(30, 70);
  CHECK(!s.state());
  CHECK(!s.update(50));
  CHECK(!s.update(69));
  CHECK(s.update(70));
  CHECK(s.update(50));
  CHECK(s.update(31));
  CHECK(!s.update(30));
  CHECK(!s.update(50));
  s.reset(true);
  CHECK(s.state());
  CHECK(s.update(50));
  mcu::Schmitt<int> swapped(70, 30);  // arguments in the wrong order are fixed
  CHECK(swapped.update(80));
  CHECK(!swapped.update(20));
  mcu::Schmitt<uint8_t> u(10, 200, true);
  CHECK(u.state());
  CHECK(!u.update(0));
  CHECK(u.update(255));
}

static void test_stats() {
  mcu::RunningStats<int16_t> st;
  CHECK(st.count() == 0 && st.min() == 0 && st.max() == 0 && st.mean() == 0);
  st.update(10);
  CHECK(st.min() == 10 && st.max() == 10 && st.mean() == 10);
  st.update(-5);
  st.update(20);
  CHECK(st.count() == 3);
  CHECK(st.min() == -5 && st.max() == 20);
  CHECK(st.sum() == 25);
  CHECK(st.mean() == 8);  // 8.33
  st.reset();
  CHECK(st.count() == 0);
  st.update(-1);
  st.update(-2);
  CHECK(st.mean() == -2);  // -1.5 away from zero

  mcu::RunningStats<uint8_t> u;
  for (int i = 0; i < 100000; ++i) u.update(255);
  CHECK(u.sum() == 25500000u);
  CHECK(u.mean() == 255 && u.count() == 100000);

  mcu::RunningStats<int32_t> big;
  big.update(INT32_MAX);
  big.update(INT32_MAX);
  big.update(INT32_MIN);
  CHECK(big.min() == INT32_MIN && big.max() == INT32_MAX);
  CHECK(big.sum() == (int64_t)INT32_MAX * 2 + INT32_MIN);

  mcu::RunningStats<uint32_t> ub;
  ub.update(UINT32_MAX);
  ub.update(UINT32_MAX);
  CHECK(ub.mean() == UINT32_MAX);

  // random against vector reference
  mcu::RunningStats<int16_t> r;
  std::vector<int16_t> v;
  for (int i = 0; i < 5000; ++i) {
    const int16_t x = (int16_t)rnd32();
    r.update(x);
    v.push_back(x);
    int64_t sum = 0;
    if (i % 97 == 0) {
      for (size_t j = 0; j < v.size(); ++j) sum += v[j];
      CHECK(r.sum() == sum);
      CHECK(r.min() == *std::min_element(v.begin(), v.end()));
      CHECK(r.max() == *std::max_element(v.begin(), v.end()));
      CHECK(r.mean() == (int16_t)mcu::detail::div_round<int64_t>(sum, (int64_t)v.size()));
    }
  }
}

// ---------------------------------------------------------------------------
// SampleDebouncer
// ---------------------------------------------------------------------------
static void test_sample_debouncer() {
  mcu::SampleDebouncer d(3);
  CHECK(!d.state());
  CHECK(!d.update(true));    // 1
  CHECK(!d.update(true));    // 2
  CHECK(d.update(true));     // 3 -> on
  CHECK(d.changed() && d.rose() && !d.fell());
  CHECK(d.update(true));
  CHECK(!d.changed());
  CHECK(d.update(false));    // 2
  CHECK(d.update(false));    // 1
  CHECK(d.update(true));     // back to 2: glitch forgiven
  CHECK(d.update(true));     // 3
  CHECK(d.update(false));
  CHECK(d.update(false));
  CHECK(!d.update(false));   // counter 3 -> 2 -> 1 -> 0: off
  CHECK(!d.state());
  CHECK(d.fell() && d.changed() && !d.rose());
  CHECK(!d.update(false));
  CHECK(!d.changed());

  // alternating noise never flips it
  mcu::SampleDebouncer n(4);
  for (int i = 0; i < 1000; ++i) CHECK(!n.update((i & 1) != 0));

  // initial state / reset / zero threshold
  mcu::SampleDebouncer hi(2, true);
  CHECK(hi.state());
  CHECK(hi.update(false));
  CHECK(!hi.update(false));
  hi.reset(true);
  CHECK(hi.state() && !hi.changed());
  CHECK(hi.update(false));
  mcu::SampleDebouncer z(0);  // treated as 1: follows the input
  CHECK(z.update(true));
  CHECK(!z.update(false));

  // property: N equal samples always flip it; fewer than N never do from rest
  for (int t = 1; t <= 10; ++t) {
    mcu::SampleDebouncer p((uint16_t)t);
    for (int i = 0; i < t - 1; ++i) CHECK(!p.update(true));
    CHECK(p.update(true));
  }
}

// ---------------------------------------------------------------------------
// Debouncer
// ---------------------------------------------------------------------------
static void test_debouncer_basic() {
  mcu::Debouncer d(20, 50);  // press 20 ms, release 50 ms
  CHECK(!d.state());
  CHECK(!d.update(false, 0));
  CHECK(!d.update(true, 100));    // starts timing
  CHECK(!d.update(true, 119));
  CHECK(d.update(true, 120));     // 20 ms of continuous high
  CHECK(d.rose() && d.changed());
  CHECK(d.update(true, 121));
  CHECK(!d.changed());
  CHECK(d.held_ms(150) == 30);

  CHECK(d.update(false, 200));    // starts release timing
  CHECK(d.update(false, 249));
  CHECK(!d.update(false, 250));   // 50 ms
  CHECK(d.fell());
  CHECK(d.held_ms(260) == 10);

  // glitch: low for 30 ms in a high state, then high again, cancels release
  mcu::Debouncer g(20, 50, true);
  CHECK(g.update(true, 0));
  CHECK(g.update(false, 10));
  CHECK(g.update(false, 40));
  CHECK(g.update(true, 41));      // cancelled
  CHECK(g.update(false, 42));     // timing restarts at 42
  CHECK(g.update(false, 91));
  CHECK(!g.update(false, 92));    // 50 ms after 42

  // zero threshold is immediate
  mcu::Debouncer z(0, 0);
  CHECK(z.update(true, 5));
  CHECK(!z.update(false, 6));

  // single-argument constructor and default
  mcu::Debouncer s(30);
  CHECK(!s.update(true, 0));
  CHECK(s.update(true, 30));
  mcu::Debouncer def;
  CHECK(!def.update(true, 0));
  CHECK(!def.update(true, 19));
  CHECK(def.update(true, 20));

  // set_thresholds, reset
  s.set_thresholds(1, 1);
  CHECK(s.update(false, 100));    // timing starts, still on
  CHECK(!s.update(false, 101));   // 1 ms later: off
  s.reset(true);
  CHECK(s.state() && s.held_ms(1234) == 0);

  // held_ms counts from first update when nothing changed yet
  mcu::Debouncer h(10, 10);
  CHECK(h.held_ms(77) == 0);
  h.update(false, 100);
  CHECK(h.held_ms(160) == 60);
}

static void test_debouncer_wrap() {
  // same timeline as above, but across the 32-bit wrap
  const uint32_t base = 0xFFFFFFF0u;
  mcu::Debouncer d(20, 50);
  CHECK(!d.update(false, base));
  CHECK(!d.update(true, base + 100 - 0x10u * 0));     // base+100 wraps
  CHECK(!d.update(true, (uint32_t)(base + 100 + 19)));
  CHECK(d.update(true, (uint32_t)(base + 100 + 20)));
  CHECK(d.held_ms((uint32_t)(base + 100 + 60)) == 40);
}

// Property test: sampled every millisecond starting near the wrap, the output
// follows the input exactly when the input has been steady for the threshold,
// and never changes sooner.
static void test_debouncer_property() {
  for (int trial = 0; trial < 200; ++trial) {
    const uint32_t press = rnd_below(30);
    const uint32_t release = rnd_below(30);
    const bool initial = (rnd32() & 1) != 0;
    mcu::Debouncer d(press, release, initial);
    uint32_t t = 0xFFFFFF00u + rnd_below(512);
    bool raw = initial;
    uint32_t run_start = t;  // when raw last changed
    bool prev_state = initial;
    bool first = true;
    for (int i = 0; i < 4000; ++i, ++t) {
      // change the raw value occasionally; mostly short runs, sometimes long
      if (!first && rnd_below(rnd_below(4) == 0 ? 80 : 12) == 0) {
        raw = !raw;
        run_start = t;
      }
      first = false;
      const bool out = d.update(raw, t);
      const uint32_t run = t - run_start;
      const uint32_t need = raw ? press : release;
      if (raw != prev_state && out == raw && prev_state != raw) {
        CHECK(run >= need);          // never faster than the threshold
      }
      if (run >= need) CHECK(out == raw);   // never slower than the threshold
      if (raw == out) CHECK(d.state() == raw);
      CHECK(d.changed() == (out != prev_state));
      prev_state = out;
    }
  }
}

// ---------------------------------------------------------------------------
// Button
// ---------------------------------------------------------------------------
struct Ev {
  uint32_t t;
  uint8_t ev;
  uint8_t clicks;
};

struct Step {
  uint32_t t;
  bool level;  // raw pin level from time t on
};

// Runs a button for `end` ms sampling every millisecond; `pre` offsets the clock.
template <size_t N>
static std::vector<Ev> run_button(mcu::Button& b, const Step (&steps)[N], uint32_t end,
                                  uint32_t offset = 0) {
  std::vector<Ev> out;
  bool level = steps[0].level;
  size_t next = 0;
  for (uint32_t t = 0; t <= end; ++t) {
    while (next < N && steps[next].t <= t) level = steps[next++].level;
    const uint8_t ev = b.update(level, (uint32_t)(offset + t));
    if (ev != 0) {
      Ev e = {t, ev, b.clicks()};
      out.push_back(e);
    }
  }
  return out;
}

static void test_button_single_click(uint32_t offset) {
  // Active low, pull-up: idle high, pressed low.
  mcu::Button b;  // 20 ms debounce, 800 ms long press, no multi-click
  const Step s[] = {{0, true}, {100, false}, {300, true}};
  const std::vector<Ev> ev = run_button(b, s, 2000, offset);
  CHECK(ev.size() == 2);
  if (ev.size() == 2) {
    CHECK(ev[0].t == 120 && ev[0].ev == mcu::Button::kPress);
    CHECK(ev[1].t == 320 && ev[1].ev == (mcu::Button::kRelease | mcu::Button::kClick));
    CHECK(ev[1].clicks == 1);
  }
  CHECK(!b.down());
}

static void test_button() {
  test_button_single_click(0);
  test_button_single_click(0xFFFFFF00u);  // across the wrap

  {  // long press
    mcu::Button b;
    const Step s[] = {{0, true}, {100, false}, {1500, true}};
    const std::vector<Ev> ev = run_button(b, s, 3000);
    CHECK(ev.size() == 3);
    if (ev.size() == 3) {
      CHECK(ev[0].t == 120 && ev[0].ev == mcu::Button::kPress);
      CHECK(ev[1].t == 920 && ev[1].ev == mcu::Button::kLongPress);  // 800 ms after the press
      CHECK(ev[2].t == 1520 && ev[2].ev == mcu::Button::kRelease);   // no click after a long press
    }
  }
  {  // held_ms / down / long_pressed while held
    mcu::Button b;
    b.update(true, 0);
    b.update(false, 100);
    for (uint32_t t = 101; t <= 500; ++t) b.update(false, t);
    CHECK(b.down());
    CHECK(b.held_ms(500) == 380);
    CHECK(!b.long_pressed());
    for (uint32_t t = 501; t <= 1000; ++t) b.update(false, t);
    CHECK(b.long_pressed());
    CHECK(b.held_ms(1000) == 880);
    b.update(true, 1001);
    CHECK(b.held_ms(1001) == 0 || b.down());
  }
  {  // bounce on press and release
    mcu::Button b;
    const Step s[] = {{0, true},  {100, false}, {105, true}, {110, false}, {115, true},
                      {118, false}, {400, true},  {403, false}, {406, true}};
    const std::vector<Ev> ev = run_button(b, s, 1000);
    CHECK(ev.size() == 2);
    if (ev.size() == 2) {
      CHECK(ev[0].t == 138 && ev[0].ev == mcu::Button::kPress);          // 118 + 20
      CHECK(ev[1].t == 426 && ev[1].ev == (mcu::Button::kRelease | mcu::Button::kClick));  // 406 + 20
    }
  }
  {  // multi-click: two quick clicks give one kClick with clicks()==2
    mcu::Button b(true, 20, 800, 300);
    const Step s[] = {{0, true}, {100, false}, {300, true}, {400, false}, {500, true}};
    const std::vector<Ev> ev = run_button(b, s, 2000);
    CHECK(ev.size() == 5);
    if (ev.size() == 5) {
      CHECK(ev[0].t == 120 && ev[0].ev == mcu::Button::kPress);
      CHECK(ev[1].t == 320 && ev[1].ev == mcu::Button::kRelease);
      CHECK(ev[2].t == 420 && ev[2].ev == mcu::Button::kPress);
      CHECK(ev[3].t == 520 && ev[3].ev == mcu::Button::kRelease);
      CHECK(ev[4].t == 820 && ev[4].ev == mcu::Button::kClick && ev[4].clicks == 2);
    }
  }
  {  // multi-click: a single click is reported after the window
    mcu::Button b(true, 20, 800, 300);
    const Step s[] = {{0, true}, {100, false}, {300, true}};
    const std::vector<Ev> ev = run_button(b, s, 2000);
    CHECK(ev.size() == 3);
    if (ev.size() == 3) {
      CHECK(ev[1].t == 320 && ev[1].ev == mcu::Button::kRelease);
      CHECK(ev[2].t == 620 && ev[2].ev == mcu::Button::kClick && ev[2].clicks == 1);
    }
  }
  {  // multi-click: three clicks
    mcu::Button b(true, 10, 800, 200);
    const Step s[] = {{0, true},   {100, false}, {150, true}, {200, false},
                      {250, true}, {300, false}, {350, true}};
    const std::vector<Ev> ev = run_button(b, s, 2000);
    uint32_t clicks = 0, kclicks = 0;
    for (size_t i = 0; i < ev.size(); ++i)
      if (ev[i].ev & mcu::Button::kClick) {
        clicks = ev[i].clicks;
        ++kclicks;
      }
    CHECK(kclicks == 1 && clicks == 3);
  }
  {  // click then hold: only the long press, the earlier click is discarded
    mcu::Button b(true, 20, 800, 300);
    const Step s[] = {{0, true}, {100, false}, {300, true}, {400, false}, {3000, true}};
    const std::vector<Ev> ev = run_button(b, s, 5000);
    int kclick = 0, klong = 0;
    uint32_t long_at = 0;
    for (size_t i = 0; i < ev.size(); ++i) {
      if (ev[i].ev & mcu::Button::kClick) ++kclick;
      if (ev[i].ev & mcu::Button::kLongPress) {
        ++klong;
        long_at = ev[i].t;
      }
    }
    CHECK(kclick == 0);
    CHECK(klong == 1);
    CHECK(long_at == 420 + 800);
  }
  {  // clicks separated by more than the window are separate sequences
    mcu::Button b(true, 20, 800, 200);
    const Step s[] = {{0, true}, {100, false}, {150, true}, {700, false}, {750, true}};
    const std::vector<Ev> ev = run_button(b, s, 3000);
    int kclick = 0;
    for (size_t i = 0; i < ev.size(); ++i)
      if (ev[i].ev & mcu::Button::kClick) {
        ++kclick;
        CHECK(ev[i].clicks == 1);
      }
    CHECK(kclick == 2);
  }
  {  // active high
    mcu::Button b(false);
    const Step s[] = {{0, false}, {100, true}, {300, false}};
    const std::vector<Ev> ev = run_button(b, s, 2000);
    CHECK(ev.size() == 2);
    if (ev.size() == 2) {
      CHECK(ev[0].t == 120 && ev[0].ev == mcu::Button::kPress);
      CHECK(ev[1].t == 320 && ev[1].ev == (mcu::Button::kRelease | mcu::Button::kClick));
    }
  }
  {  // long press disabled
    mcu::Button b(true, 20, 0);
    const Step s[] = {{0, true}, {100, false}, {6000, true}};
    const std::vector<Ev> ev = run_button(b, s, 8000);
    CHECK(ev.size() == 2);
    for (size_t i = 0; i < ev.size(); ++i) CHECK((ev[i].ev & mcu::Button::kLongPress) == 0);
    if (ev.size() == 2) CHECK(ev[1].ev == (mcu::Button::kRelease | mcu::Button::kClick));
  }
  {  // a press shorter than the debounce time is ignored completely
    mcu::Button b;
    const Step s[] = {{0, true}, {100, false}, {115, true}};
    const std::vector<Ev> ev = run_button(b, s, 1000);
    CHECK(ev.empty());
  }
  {  // reset
    mcu::Button b;
    b.update(false, 0);
    b.update(false, 30);
    CHECK(b.down());
    b.reset();
    CHECK(!b.down() && b.clicks() == 0);
  }
}

// Random button traffic: invariants only.
static void test_button_fuzz() {
  for (int trial = 0; trial < 100; ++trial) {
    const uint32_t multi = (trial & 1) ? 0 : 50 + rnd_below(300);
    mcu::Button b((trial & 2) != 0, 1 + rnd_below(30), (trial & 4) ? 0 : 100 + rnd_below(500), multi);
    const bool active_low = (trial & 2) != 0;
    uint32_t t = 0xFFFFF000u + rnd_below(8192);
    bool level = active_low;  // idle level
    int presses = 0, releases = 0, clicks = 0, longs = 0;
    bool down_by_events = false;
    for (int i = 0; i < 20000; ++i, ++t) {
      if (rnd_below(rnd_below(3) == 0 ? 300 : 25) == 0) level = !level;
      const uint8_t ev = b.update(level, t);
      if (ev & mcu::Button::kPress) {
        CHECK(!down_by_events);
        down_by_events = true;
        ++presses;
      }
      if (ev & mcu::Button::kRelease) {
        CHECK(down_by_events);
        down_by_events = false;
        ++releases;
      }
      if (ev & mcu::Button::kClick) ++clicks;
      if (ev & mcu::Button::kLongPress) {
        CHECK(down_by_events);
        ++longs;
      }
      CHECK(b.down() == down_by_events);
      CHECK(!(ev & ~15));
      if ((ev & mcu::Button::kClick) != 0) CHECK(b.clicks() >= 1);
    }
    CHECK(releases <= presses && presses - releases <= 1);
    CHECK(clicks <= releases);
    CHECK(longs <= presses);
  }
}

int main() {
  test_limits();
  test_sat();
  test_div_round();
  test_map_range();
  test_isqrt();
  test_q();
  test_moving_average();
  test_moving_average_basic();
  test_ema();
  test_median();
  test_slew();
  test_schmitt();
  test_stats();
  test_sample_debouncer();
  test_debouncer_basic();
  test_debouncer_wrap();
  test_debouncer_property();
  test_button();
  test_button_fuzz();

  printf("%lu checks, %lu failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
