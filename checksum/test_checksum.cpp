// Host-side tests for crc.hpp and checksum.hpp. Build and run on a PC:
//   g++ -std=c++11 -Wall -Wextra -Werror -I. test_checksum.cpp -o test_checksum && ./test_checksum

#include <stdio.h>
#include <string.h>

#include "checksum.hpp"
#include "crc.hpp"

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

#define CHECK_MSG(cond, name)                                        \
  do {                                                               \
    ++g_checks;                                                      \
    if (!(cond)) {                                                   \
      ++g_failures;                                                  \
      printf("FAIL %s:%d: %s [%s]\n", __FILE__, __LINE__, #cond, name); \
    }                                                                \
  } while (0)

// Small deterministic PRNG so failures are reproducible.
static uint32_t g_rng = 12345;
static uint8_t rnd8() {
  g_rng = g_rng * 1664525u + 1013904223u;
  return (uint8_t)(g_rng >> 24);
}

// ---------------------------------------------------------------------------
// Independent reference CRC: textbook Rocksoft model, always MSB-first with
// explicit reflection of each input byte and of the final register. This is
// deliberately a different formulation from the reflected-register algorithm
// used by the library.
// ---------------------------------------------------------------------------
static uint64_t reflect_ref(uint64_t v, int bits) {
  uint64_t r = 0;
  for (int i = 0; i < bits; ++i)
    if (v & (1ull << i)) r |= 1ull << (bits - 1 - i);
  return r;
}

static uint32_t ref_crc(int width, uint32_t poly, uint32_t init, bool refl, uint32_t xorout,
                        const uint8_t* d, size_t n) {
  const uint64_t mask = (width == 32) ? 0xFFFFFFFFull : ((1ull << width) - 1);
  const uint64_t top = 1ull << (width - 1);
  uint64_t reg = init;
  for (size_t i = 0; i < n; ++i) {
    uint64_t b = refl ? reflect_ref(d[i], 8) : d[i];
    reg ^= b << (width - 8);
    for (int k = 0; k < 8; ++k) reg = ((reg & top) ? ((reg << 1) ^ poly) : (reg << 1)) & mask;
  }
  if (refl) reg = reflect_ref(reg, width);
  return (uint32_t)((reg ^ xorout) & mask);
}

template <typename C>
static void test_preset(const char* name, int width, uint32_t poly, uint32_t init, bool refl,
                        uint32_t xorout, uint32_t check) {
  const char* s = "123456789";

  // Published check value, bitwise and nibble-table.
  CHECK_MSG((uint32_t)C::compute(s, 9) == check, name);
  CHECK_MSG((uint32_t)C::Fast::compute(s, 9) == check, name);

  // Against the independent reference, many lengths including empty.
  uint8_t buf[300];
  for (size_t i = 0; i < sizeof buf; ++i) buf[i] = rnd8();
  for (size_t len = 0; len <= 70; ++len) {
    uint32_t want = ref_crc(width, poly, init, refl, xorout, buf, len);
    CHECK_MSG((uint32_t)C::compute(buf, len) == want, name);
    CHECK_MSG((uint32_t)C::Fast::compute(buf, len) == want, name);
  }
  {
    uint32_t want = ref_crc(width, poly, init, refl, xorout, buf, sizeof buf);
    CHECK_MSG((uint32_t)C::compute(buf, sizeof buf) == want, name);
    CHECK_MSG((uint32_t)C::Fast::compute(buf, sizeof buf) == want, name);
  }

  // Streaming: every split point gives the same answer as one shot.
  uint32_t whole = (uint32_t)C::compute(buf, 100);
  for (size_t split = 0; split <= 100; ++split) {
    C c;
    c.update(buf, split);
    c.update(buf + split, 100 - split);
    CHECK_MSG((uint32_t)c.value() == whole, name);
  }

  // Byte-at-a-time, value() mid-stream must not disturb the state.
  {
    C c;
    for (size_t i = 0; i < 100; ++i) {
      c.update(buf[i]);
      (void)c.value();
    }
    CHECK_MSG((uint32_t)c.value() == whole, name);
  }

  // reset() restarts.
  {
    C c;
    c.update(buf, 17);
    c.reset();
    c.update(buf, 100);
    CHECK_MSG((uint32_t)c.value() == whole, name);
  }

  // resume() continues from an earlier value().
  for (size_t split = 0; split <= 100; split += 7) {
    C first;
    first.update(buf, split);
    C second;
    second.resume(first.value());
    second.update(buf + split, 100 - split);
    CHECK_MSG((uint32_t)second.value() == whole, name);
  }
}

static void test_crc_presets() {
  using namespace mcu;
  //                      name       w  poly        init        refl   xorout      check
  test_preset<Crc8>("Crc8", 8, 0x07, 0x00, false, 0x00, 0xF4);
  test_preset<Crc8Maxim>("Crc8Maxim", 8, 0x31, 0x00, true, 0x00, 0xA1);
  test_preset<Crc8SaeJ1850>("Crc8SaeJ1850", 8, 0x1D, 0xFF, false, 0xFF, 0x4B);

  test_preset<Crc16Xmodem>("Crc16Xmodem", 16, 0x1021, 0x0000, false, 0x0000, 0x31C3);
  test_preset<Crc16CcittFalse>("Crc16CcittFalse", 16, 0x1021, 0xFFFF, false, 0x0000, 0x29B1);
  test_preset<Crc16Kermit>("Crc16Kermit", 16, 0x1021, 0x0000, true, 0x0000, 0x2189);
  test_preset<Crc16X25>("Crc16X25", 16, 0x1021, 0xFFFF, true, 0xFFFF, 0x906E);
  test_preset<Crc16Arc>("Crc16Arc", 16, 0x8005, 0x0000, true, 0x0000, 0xBB3D);
  test_preset<Crc16Modbus>("Crc16Modbus", 16, 0x8005, 0xFFFF, true, 0x0000, 0x4B37);
  test_preset<Crc16Usb>("Crc16Usb", 16, 0x8005, 0xFFFF, true, 0xFFFF, 0xB4C8);

  test_preset<Crc32>("Crc32", 32, 0x04C11DB7u, 0xFFFFFFFFu, true, 0xFFFFFFFFu, 0xCBF43926u);
  test_preset<Crc32c>("Crc32c", 32, 0x1EDC6F41u, 0xFFFFFFFFu, true, 0xFFFFFFFFu, 0xE3069283u);
  test_preset<Crc32Mpeg2>("Crc32Mpeg2", 32, 0x04C11DB7u, 0xFFFFFFFFu, false, 0x00000000u,
                          0x0376E6E7u);
  test_preset<Crc32Bzip2>("Crc32Bzip2", 32, 0x04C11DB7u, 0xFFFFFFFFu, false, 0xFFFFFFFFu,
                          0xFC891918u);
}

static void test_crc_known_values() {
  using namespace mcu;
  // Empty input: CRC-32 of nothing is 0.
  CHECK(Crc32::compute("", 0) == 0x00000000u);
  // The classic.
  CHECK(Crc32::compute("The quick brown fox jumps over the lazy dog", 43) == 0x414FA339u);
  // Appending the CRC (little-endian) to the data makes the CRC-32 residue constant.
  {
    const char* s = "123456789";
    uint32_t c = Crc32::compute(s, 9);
    uint8_t msg[13];
    memcpy(msg, s, 9);
    msg[9] = (uint8_t)c;
    msg[10] = (uint8_t)(c >> 8);
    msg[11] = (uint8_t)(c >> 16);
    msg[12] = (uint8_t)(c >> 24);
    CHECK(Crc32::compute(msg, 13) == 0x2144DF1Cu);
  }
  // Modbus: appending the CRC low byte first makes the CRC of the whole frame zero.
  {
    uint8_t frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A, 0, 0};
    uint16_t c = Crc16Modbus::compute(frame, 6);
    frame[6] = (uint8_t)(c & 0xFF);
    frame[7] = (uint8_t)(c >> 8);
    CHECK(Crc16Modbus::compute(frame, 8) == 0x0000);
  }
}

// ---------------------------------------------------------------------------
// Fletcher references: plain definitions with modulo.
// ---------------------------------------------------------------------------
static uint16_t ref_fletcher16(const uint8_t* d, size_t n) {
  uint32_t a = 0, b = 0;
  for (size_t i = 0; i < n; ++i) {
    a = (a + d[i]) % 255;
    b = (b + a) % 255;
  }
  return (uint16_t)((b << 8) | a);
}

static uint32_t ref_fletcher32(const uint8_t* d, size_t n) {
  uint64_t a = 0, b = 0;
  for (size_t i = 0; i < n; i += 2) {
    uint32_t w = d[i] | (i + 1 < n ? ((uint32_t)d[i + 1] << 8) : 0u);
    a = (a + w) % 65535;
    b = (b + a) % 65535;
  }
  return (uint32_t)((b << 16) | a);
}

static void test_fletcher() {
  using namespace mcu;
  // Published vectors.
  CHECK(Fletcher16::compute("abcde", 5) == 0xC8F0);
  CHECK(Fletcher16::compute("abcdef", 6) == 0x2057);
  CHECK(Fletcher16::compute("abcdefgh", 8) == 0x0627);
  CHECK(Fletcher32::compute("abcde", 5) == 0xF04FC729u);
  CHECK(Fletcher32::compute("abcdef", 6) == 0x56502D2Au);
  CHECK(Fletcher32::compute("abcdefgh", 8) == 0xEBE19591u);

  CHECK(Fletcher16::compute("", 0) == 0);
  CHECK(Fletcher32::compute("", 0) == 0);

  // Known weakness, documented in the header: 0x00 and 0xFF are the same to Fletcher-16.
  {
    uint8_t z[3] = {0x00, 0x01, 0x02};
    uint8_t f[3] = {0xFF, 0x01, 0x02};
    CHECK(Fletcher16::compute(z, 3) == Fletcher16::compute(f, 3));
  }

  // Against the straightforward modulo definition.
  uint8_t buf[600];
  for (size_t i = 0; i < sizeof buf; ++i) buf[i] = rnd8();
  for (size_t len = 0; len <= 100; ++len) {
    CHECK(Fletcher16::compute(buf, len) == ref_fletcher16(buf, len));
    CHECK(Fletcher32::compute(buf, len) == ref_fletcher32(buf, len));
  }
  CHECK(Fletcher16::compute(buf, sizeof buf) == ref_fletcher16(buf, sizeof buf));
  CHECK(Fletcher32::compute(buf, sizeof buf) == ref_fletcher32(buf, sizeof buf));

  // Worst cases for the conditional-subtract reduction: all 0xFF bytes.
  {
    uint8_t ff[4096];
    memset(ff, 0xFF, sizeof ff);
    CHECK(Fletcher16::compute(ff, sizeof ff) == ref_fletcher16(ff, sizeof ff));
    CHECK(Fletcher32::compute(ff, sizeof ff) == ref_fletcher32(ff, sizeof ff));
    CHECK(Fletcher32::compute(ff, sizeof ff - 1) == ref_fletcher32(ff, sizeof ff - 1));
  }

  // Streaming at every split point, including odd ones for Fletcher-32.
  for (size_t len = 0; len <= 33; ++len) {
    uint16_t w16 = Fletcher16::compute(buf, len);
    uint32_t w32 = Fletcher32::compute(buf, len);
    for (size_t split = 0; split <= len; ++split) {
      Fletcher16 a;
      a.update(buf, split);
      a.update(buf + split, len - split);
      CHECK(a.value() == w16);
      Fletcher32 b;
      b.update(buf, split);
      b.update(buf + split, len - split);
      CHECK(b.value() == w32);
    }
  }

  // value() mid-stream does not disturb Fletcher-32's pending odd byte.
  {
    Fletcher32 c;
    for (size_t i = 0; i < 51; ++i) {
      c.update(buf[i]);
      (void)c.value();
    }
    CHECK(c.value() == ref_fletcher32(buf, 51));
  }

  // reset() and Fletcher-16 resume().
  {
    Fletcher16 c;
    c.update(buf, 10);
    c.reset();
    CHECK(c.value() == 0);
    Fletcher16 first;
    first.update(buf, 20);
    Fletcher16 second;
    second.resume(first.value());
    second.update(buf + 20, 30);
    CHECK(second.value() == ref_fletcher16(buf, 50));
  }
}

static void test_simple_sums() {
  using namespace mcu;
  const uint8_t d[3] = {0x01, 0x02, 0x04};
  CHECK(Xor8::compute(d, 3) == 0x07);
  CHECK(Sum8::compute(d, 3) == 0x07);
  CHECK(Lrc8::compute(d, 3) == 0xF9);

  // Sum wraps modulo 256.
  const uint8_t w[3] = {0xFF, 0xFF, 0x03};
  CHECK(Sum8::compute(w, 3) == 0x01);

  // Intel HEX record ":0300300002337A1E": data 03 00 30 00 02 33 7A, checksum 1E.
  const uint8_t rec[7] = {0x03, 0x00, 0x30, 0x00, 0x02, 0x33, 0x7A};
  CHECK(Lrc8::compute(rec, 7) == 0x1E);

  // Appending the LRC makes the whole message sum to zero.
  {
    uint8_t msg[8];
    memcpy(msg, rec, 7);
    msg[7] = Lrc8::compute(rec, 7);
    CHECK(Sum8::compute(msg, 8) == 0x00);
  }

  // XOR of data plus its own XOR is zero (NMEA style).
  {
    uint8_t msg[6] = {'G', 'P', 'G', 'L', 'L', 0};
    msg[5] = Xor8::compute(msg, 5);
    CHECK(Xor8::compute(msg, 6) == 0x00);
  }

  // Streaming, reset, resume.
  {
    Xor8 x;
    Sum8 s;
    Lrc8 l;
    for (int i = 0; i < 3; ++i) {
      x.update(d[i]);
      s.update(d[i]);
      l.update(d[i]);
    }
    CHECK(x.value() == 0x07 && s.value() == 0x07 && l.value() == 0xF9);
    Lrc8 l2;
    l2.resume(l.value());
    CHECK(l2.value() == l.value());
    l2.update(0x10);
    l.update(0x10);
    CHECK(l2.value() == l.value());
    x.reset();
    s.reset();
    l.reset();
    CHECK(x.value() == 0 && s.value() == 0 && l.value() == 0);
  }
}

// All checksum classes share one interface; this must compile for each of them.
template <typename C>
static typename C::value_type generic_check(const uint8_t* d, size_t n) {
  C c;
  c.update(d[0]);
  c.update(d + 1, n - 1);
  return c.value();
}

static void test_generic_interface() {
  const uint8_t d[5] = {1, 2, 3, 4, 5};
  CHECK(generic_check<mcu::Crc32>(d, 5) == mcu::Crc32::compute(d, 5));
  CHECK(generic_check<mcu::Crc16Modbus::Fast>(d, 5) == mcu::Crc16Modbus::compute(d, 5));
  CHECK(generic_check<mcu::Crc8>(d, 5) == mcu::Crc8::compute(d, 5));
  CHECK(generic_check<mcu::Fletcher16>(d, 5) == mcu::Fletcher16::compute(d, 5));
  CHECK(generic_check<mcu::Fletcher32>(d, 5) == mcu::Fletcher32::compute(d, 5));
  CHECK(generic_check<mcu::Xor8>(d, 5) == mcu::Xor8::compute(d, 5));
  CHECK(generic_check<mcu::Sum8>(d, 5) == mcu::Sum8::compute(d, 5));
  CHECK(generic_check<mcu::Lrc8>(d, 5) == mcu::Lrc8::compute(d, 5));
}

int main() {
  test_crc_presets();
  test_crc_known_values();
  test_fletcher();
  test_simple_sums();
  test_generic_interface();

  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
