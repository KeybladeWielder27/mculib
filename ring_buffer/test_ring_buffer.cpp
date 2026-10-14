// Host-side tests for ring_buffer.hpp. Build and run on a PC:
//   g++ -std=c++11 -Wall -Wextra -Werror -I. test_ring_buffer.cpp -o test_ring_buffer && ./test_ring_buffer

#include <stdio.h>
#include <string.h>

#include "ring_buffer.hpp"

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                                   \
  do {                                                                \
    ++g_checks;                                                       \
    if (!(cond)) {                                                    \
      ++g_failures;                                                   \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
    }                                                                 \
  } while (0)

static void test_basic() {
  mcu::RingBuffer<uint8_t, 8> rb;
  CHECK(rb.capacity() == 8);
  CHECK(rb.empty());
  CHECK(!rb.full());
  CHECK(rb.size() == 0);
  CHECK(rb.free_space() == 8);

  uint8_t v = 0;
  CHECK(!rb.pop(v));
  CHECK(!rb.peek(v));

  CHECK(rb.push(10));
  CHECK(rb.push(20));
  CHECK(rb.size() == 2);
  CHECK(rb.peek(v) && v == 10);
  CHECK(rb.size() == 2);  // peek does not consume
  CHECK(rb.pop(v) && v == 10);
  CHECK(rb.pop(v) && v == 20);
  CHECK(rb.empty());
}

static void test_full_and_overflow() {
  mcu::RingBuffer<uint8_t, 4> rb;
  for (uint8_t i = 0; i < 4; ++i) CHECK(rb.push(i));
  CHECK(rb.full());
  CHECK(rb.size() == 4);
  CHECK(rb.free_space() == 0);
  CHECK(!rb.push(99));  // rejected, nothing overwritten
  CHECK(rb.size() == 4);

  uint8_t v;
  for (uint8_t i = 0; i < 4; ++i) CHECK(rb.pop(v) && v == i);
  CHECK(rb.empty());
}

static void test_wraparound() {
  // Push/pop far more items than the capacity so indices wrap many times,
  // including past the 8-bit index limit.
  mcu::RingBuffer<uint16_t, 8> rb;
  uint16_t next_in = 0, next_out = 0, v;
  for (int round = 0; round < 5000; ++round) {
    int n = 1 + (round % 8);
    for (int i = 0; i < n; ++i) CHECK(rb.push(next_in++));
    for (int i = 0; i < n; ++i) {
      CHECK(rb.pop(v));
      CHECK(v == next_out);
      ++next_out;
    }
    CHECK(rb.empty());
  }
}

static void test_wraparound_partially_filled() {
  // Keep the buffer partially full while the indices run past 255.
  mcu::RingBuffer<uint8_t, 16> rb;
  uint8_t next_in = 0, next_out = 0, v = 0;
  for (int i = 0; i < 10; ++i) rb.push(next_in++);
  for (int i = 0; i < 100000; ++i) {
    CHECK(rb.push(next_in++));
    CHECK(rb.pop(v));
    CHECK(v == next_out++);
    CHECK(rb.size() == 10);
  }
}

static void test_bulk() {
  mcu::RingBuffer<uint8_t, 16> rb;
  uint8_t in[20], out[20];
  for (int i = 0; i < 20; ++i) in[i] = (uint8_t)(i + 1);

  // Writing more than fits only writes what fits.
  CHECK(rb.write(in, 20) == 16);
  CHECK(rb.full());
  CHECK(rb.write(in, 1) == 0);

  // Partial read.
  CHECK(rb.read(out, 5) == 5);
  for (int i = 0; i < 5; ++i) CHECK(out[i] == in[i]);
  CHECK(rb.size() == 11);

  // Write again so the data wraps across the end of the storage.
  CHECK(rb.write(in + 16, 4) == 4);
  CHECK(rb.size() == 15);

  // Read everything back (asking for more than available).
  memset(out, 0, sizeof out);
  CHECK(rb.read(out, 20) == 15);
  for (int i = 0; i < 11; ++i) CHECK(out[i] == in[5 + i]);
  for (int i = 0; i < 4; ++i) CHECK(out[11 + i] == in[16 + i]);
  CHECK(rb.empty());
  CHECK(rb.read(out, 3) == 0);
}

static void test_discard_clear() {
  mcu::RingBuffer<uint8_t, 8> rb;
  for (uint8_t i = 0; i < 6; ++i) rb.push(i);
  CHECK(rb.discard(2) == 2);
  uint8_t v;
  CHECK(rb.peek(v) && v == 2);
  CHECK(rb.discard(100) == 4);  // clamps to what is available
  CHECK(rb.empty());

  for (uint8_t i = 0; i < 5; ++i) rb.push(i);
  rb.clear();
  CHECK(rb.empty());
  CHECK(rb.push(42));
  CHECK(rb.pop(v) && v == 42);
}

struct Sample {
  int16_t x;
  int16_t y;
  uint32_t t;
};

static void test_struct_elements() {
  mcu::RingBuffer<Sample, 4> rb;
  Sample a = {1, -2, 1000};
  Sample b = {3, -4, 2000};
  CHECK(rb.push(a));
  CHECK(rb.push(b));
  Sample out = {0, 0, 0};
  CHECK(rb.pop(out));
  CHECK(out.x == 1 && out.y == -2 && out.t == 1000);
  CHECK(rb.pop(out));
  CHECK(out.x == 3 && out.y == -4 && out.t == 2000);
}

static void test_index_types() {
  // Index type is chosen from capacity: 8-bit up to 128, 16-bit above.
  static_assert(sizeof(mcu::RingBuffer<uint8_t, 128>::index_type) == 1, "");
  static_assert(sizeof(mcu::RingBuffer<uint8_t, 256>::index_type) == 2, "");

  mcu::RingBuffer<uint8_t, 256> big;
  uint8_t next_in = 0, next_out = 0, v = 0;
  for (int i = 0; i < 200000; ++i) {
    CHECK(big.push(next_in++));
    CHECK(big.pop(v));
    CHECK(v == next_out++);
  }

  mcu::RingBuffer<uint8_t, 128> edge;  // 8-bit index, 2*N-1 == 255 fits exactly
  for (int i = 0; i < 128; ++i) CHECK(edge.push((uint8_t)i));
  CHECK(edge.full());
  CHECK(edge.size() == 128);
  for (int i = 0; i < 128; ++i) CHECK(edge.pop(v) && v == (uint8_t)i);
  for (int i = 0; i < 300; ++i) {  // wrap the 8-bit index while full-ish
    CHECK(edge.push((uint8_t)i));
    CHECK(edge.pop(v) && v == (uint8_t)i);
  }
}

int main() {
  test_basic();
  test_full_and_overflow();
  test_wraparound();
  test_wraparound_partially_filled();
  test_bulk();
  test_discard_clear();
  test_struct_elements();
  test_index_types();

  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
