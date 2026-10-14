// Host-side tests for codec.hpp and framer.hpp. Build and run from this folder:
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_framer.cpp -o test_framer && ./test_framer
// (-I.. puts the root of the collection on the include path.)

#include <stdio.h>
#include <string.h>

#include "checksum/checksum.hpp"
#include "checksum/crc.hpp"
#include "framing/framer.hpp"
#include "hal/loopback_stream.hpp"

using mcu::Cobs;
using mcu::FrameResult;
using mcu::NoCheck;
using mcu::Slip;

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

#define CHECK_MSG(cond, name)                                             \
  do {                                                                    \
    ++g_checks;                                                           \
    if (!(cond)) {                                                        \
      ++g_failures;                                                       \
      printf("FAIL %s:%d: %s [%s]\n", __FILE__, __LINE__, #cond, name);   \
    }                                                                     \
  } while (0)

static uint32_t g_rng = 987654321u;
static uint8_t rnd8() {
  g_rng = g_rng * 1664525u + 1013904223u;
  return (uint8_t)(g_rng >> 24);
}

// Payload full of the bytes that stress both codecs: 0x00, 0xC0, 0xDB, ...
static void fill_special(uint8_t* p, size_t n) {
  static const uint8_t sp[] = {0x00, 0xC0, 0xDB, 0xDC, 0xDD, 0x01, 0xFF, 0x7E};
  for (size_t i = 0; i < n; ++i) p[i] = (rnd8() & 1) ? sp[rnd8() % sizeof sp] : rnd8();
}

// ---------------------------------------------------------------------------
// Published COBS test vectors (Wikipedia), plus the 254/255-byte boundaries.
// ---------------------------------------------------------------------------
typedef mcu::FrameEncoder<Cobs, NoCheck, 600> CobsEnc;
typedef mcu::FrameDecoder<Cobs, NoCheck, 600> CobsDec;

static void expect_cobs(const uint8_t* in, size_t n, const uint8_t* want, size_t wn) {
  uint8_t out[CobsEnc::max_frame_size()];
  size_t got = CobsEnc::encode(in, n, out, sizeof out);
  CHECK(got == wn);
  CHECK(got == wn && memcmp(out, want, wn) == 0);
  CHECK(CobsEnc::frame_size(in, n) == wn);

  // and it decodes back
  CobsDec dec;
  FrameResult r = FrameResult::kNoFrame;
  for (size_t i = 0; i < wn; ++i) {
    r = dec.push(want[i]);
    if (i + 1 < wn) CHECK(r == FrameResult::kNoFrame);
  }
  CHECK(r == FrameResult::kFrame);
  CHECK(dec.size() == n);
  CHECK(n == 0 || memcmp(dec.data(), in, n) == 0);
}

static void test_cobs_vectors() {
  {
    const uint8_t in[] = {0x00};
    const uint8_t want[] = {0x01, 0x01, 0x00};
    expect_cobs(in, sizeof in, want, sizeof want);
  }
  {
    const uint8_t in[] = {0x00, 0x00};
    const uint8_t want[] = {0x01, 0x01, 0x01, 0x00};
    expect_cobs(in, sizeof in, want, sizeof want);
  }
  {
    const uint8_t in[] = {0x00, 0x11, 0x00};
    const uint8_t want[] = {0x01, 0x02, 0x11, 0x01, 0x00};
    expect_cobs(in, sizeof in, want, sizeof want);
  }
  {
    const uint8_t in[] = {0x11, 0x22, 0x00, 0x33};
    const uint8_t want[] = {0x03, 0x11, 0x22, 0x02, 0x33, 0x00};
    expect_cobs(in, sizeof in, want, sizeof want);
  }
  {
    const uint8_t in[] = {0x11, 0x22, 0x33, 0x44};
    const uint8_t want[] = {0x05, 0x11, 0x22, 0x33, 0x44, 0x00};
    expect_cobs(in, sizeof in, want, sizeof want);
  }
  {
    const uint8_t in[] = {0x11, 0x00, 0x00, 0x00};
    const uint8_t want[] = {0x02, 0x11, 0x01, 0x01, 0x01, 0x00};
    expect_cobs(in, sizeof in, want, sizeof want);
  }
  {  // empty payload is a legal COBS frame
    const uint8_t want[] = {0x01, 0x00};
    expect_cobs(NULL, 0, want, sizeof want);
  }
  {  // 254 non-zero bytes: one full block, no trailing block
    uint8_t in[254], want[256];
    for (int i = 0; i < 254; ++i) in[i] = (uint8_t)(i + 1);
    want[0] = 0xFF;
    for (int i = 0; i < 254; ++i) want[1 + i] = (uint8_t)(i + 1);
    want[255] = 0x00;
    expect_cobs(in, sizeof in, want, sizeof want);
  }
  {  // 255 bytes 00..FE
    uint8_t in[255], want[258];
    for (int i = 0; i < 255; ++i) in[i] = (uint8_t)i;
    want[0] = 0x01;
    want[1] = 0xFF;
    for (int i = 0; i < 254; ++i) want[2 + i] = (uint8_t)(i + 1);
    want[256] = 0x00;
    expect_cobs(in, sizeof in - 0, want, 257);
  }
  {  // 255 bytes 01..FF
    uint8_t in[255], want[258];
    for (int i = 0; i < 255; ++i) in[i] = (uint8_t)(i + 1);
    want[0] = 0xFF;
    for (int i = 0; i < 254; ++i) want[1 + i] = (uint8_t)(i + 1);
    want[255] = 0x02;
    want[256] = 0xFF;
    want[257] = 0x00;
    expect_cobs(in, sizeof in, want, 258);
  }
  {  // 255 bytes 02..FF followed by 00
    uint8_t in[255], want[259];
    for (int i = 0; i < 254; ++i) in[i] = (uint8_t)(i + 2);
    in[254] = 0x00;
    want[0] = 0xFF;
    for (int i = 0; i < 254; ++i) want[1 + i] = (uint8_t)(i + 2);
    want[255] = 0x01;
    want[256] = 0x01;
    want[257] = 0x00;
    expect_cobs(in, sizeof in, want, 258);
  }
  {  // 255 bytes 03..FF, 00, 01
    uint8_t in[255], want[259];
    for (int i = 0; i < 253; ++i) in[i] = (uint8_t)(i + 3);
    in[253] = 0x00;
    in[254] = 0x01;
    want[0] = 0xFE;
    for (int i = 0; i < 253; ++i) want[1 + i] = (uint8_t)(i + 3);
    want[254] = 0x02;
    want[255] = 0x01;
    want[256] = 0x00;
    expect_cobs(in, sizeof in, want, 257);
  }
}

// ---------------------------------------------------------------------------
// SLIP vectors (RFC 1055)
// ---------------------------------------------------------------------------
typedef mcu::FrameEncoder<Slip, NoCheck, 64> SlipEnc;
typedef mcu::FrameDecoder<Slip, NoCheck, 64> SlipDec;

static void expect_slip(const uint8_t* in, size_t n, const uint8_t* want, size_t wn) {
  uint8_t out[SlipEnc::max_frame_size()];
  size_t got = SlipEnc::encode(in, n, out, sizeof out);
  CHECK(got == wn);
  CHECK(got == wn && memcmp(out, want, wn) == 0);
  SlipDec dec;
  FrameResult r = FrameResult::kNoFrame;
  for (size_t i = 0; i < wn; ++i) r = dec.push(want[i]);
  CHECK(r == FrameResult::kFrame);
  CHECK(dec.size() == n && memcmp(dec.data(), in, n) == 0);
}

static void test_slip_vectors() {
  {
    const uint8_t in[] = {0xC0};
    const uint8_t want[] = {0xDB, 0xDC, 0xC0};
    expect_slip(in, sizeof in, want, sizeof want);
  }
  {
    const uint8_t in[] = {0xDB};
    const uint8_t want[] = {0xDB, 0xDD, 0xC0};
    expect_slip(in, sizeof in, want, sizeof want);
  }
  {
    const uint8_t in[] = {0x01, 0xC0, 0x02, 0xDB, 0x03};
    const uint8_t want[] = {0x01, 0xDB, 0xDC, 0x02, 0xDB, 0xDD, 0x03, 0xC0};
    expect_slip(in, sizeof in, want, sizeof want);
  }
  {  // plain data passes through untouched
    const uint8_t in[] = {'h', 'e', 'l', 'l', 'o'};
    const uint8_t want[] = {'h', 'e', 'l', 'l', 'o', 0xC0};
    expect_slip(in, sizeof in, want, sizeof want);
  }
  {  // 0xDC / 0xDD on their own are ordinary data
    const uint8_t in[] = {0xDC, 0xDD};
    const uint8_t want[] = {0xDC, 0xDD, 0xC0};
    expect_slip(in, sizeof in, want, sizeof want);
  }
  {  // a zero-length frame cannot exist in SLIP: refused, and a bare END is idle
    uint8_t out[8];
    CHECK(SlipEnc::encode(NULL, 0, out, sizeof out) == 0);
    CHECK(SlipEnc::frame_size(NULL, 0) == 0);
    SlipDec dec;
    CHECK(dec.push(0xC0) == FrameResult::kNoFrame);
    CHECK(dec.push(0xC0) == FrameResult::kNoFrame);
  }
}

static void test_slip_bad_escape() {
  SlipDec dec;
  CHECK(dec.push(0x01) == FrameResult::kNoFrame);
  CHECK(dec.push(0xDB) == FrameResult::kNoFrame);
  CHECK(dec.push(0x42) == FrameResult::kNoFrame);  // ESC followed by junk
  CHECK(dec.push(0x05) == FrameResult::kNoFrame);
  CHECK(dec.push(0xC0) == FrameResult::kBadEncoding);
  // END straight after ESC is also malformed
  CHECK(dec.push(0x01) == FrameResult::kNoFrame);
  CHECK(dec.push(0xDB) == FrameResult::kNoFrame);
  CHECK(dec.push(0xC0) == FrameResult::kBadEncoding);
  // and the decoder is clean afterwards
  CHECK(dec.push(0x07) == FrameResult::kNoFrame);
  CHECK(dec.push(0xC0) == FrameResult::kFrame);
  CHECK(dec.size() == 1 && dec.data()[0] == 0x07);
  CHECK(dec.stats().bad_encoding == 2 && dec.stats().ok == 1);
}

static void test_cobs_truncated() {
  CobsDec dec;
  CHECK(dec.push(0x05) == FrameResult::kNoFrame);  // promises 4 data bytes
  CHECK(dec.push(0x11) == FrameResult::kNoFrame);
  CHECK(dec.push(0x00) == FrameResult::kBadEncoding);  // delimiter arrives early
  CHECK(dec.push(0x00) == FrameResult::kNoFrame);      // idle delimiters are ignored
  CHECK(dec.push(0x00) == FrameResult::kNoFrame);
  CHECK(dec.push(0x02) == FrameResult::kNoFrame);
  CHECK(dec.push(0x42) == FrameResult::kNoFrame);
  CHECK(dec.push(0x00) == FrameResult::kFrame);
  CHECK(dec.size() == 1 && dec.data()[0] == 0x42);
}

// ---------------------------------------------------------------------------
// Round trips across codec x check combinations
// ---------------------------------------------------------------------------
template <typename Codec, typename Check, size_t N, bool LE>
static void test_roundtrip(const char* name) {
  typedef mcu::FrameEncoder<Codec, Check, N, LE> Enc;
  typedef mcu::FrameDecoder<Codec, Check, N, LE> Dec;

  Dec dec;
  uint8_t payload[N];
  uint8_t frame[Enc::max_frame_size()];
  uint32_t expected_frames = 0;

  for (size_t len = 0; len <= N; ++len) {
    for (int variant = 0; variant < 3; ++variant) {
      fill_special(payload, len);
      const bool leading = (variant == 1);
      const bool empty_unsendable = (len + Enc::kCheckBytes == 0) && !Codec::allows_empty();

      size_t n = Enc::encode(payload, len, frame, sizeof frame, leading);
      if (empty_unsendable) {
        CHECK_MSG(n == 0, name);
        continue;
      }
      CHECK_MSG(n > 0, name);
      CHECK_MSG(n <= Enc::max_frame_size(), name);
      CHECK_MSG(n == Enc::frame_size(payload, len, leading), name);
      CHECK_MSG(frame[n - 1] == Codec::delimiter(), name);

      // A buffer one byte too small must be refused, not overrun.
      CHECK_MSG(Enc::encode(payload, len, frame, n - 1, leading) == 0, name);

      // Byte at a time: only the last byte completes the frame.
      FrameResult r = FrameResult::kNoFrame;
      for (size_t i = 0; i < n; ++i) {
        r = dec.push(frame[i]);
        if (i + 1 < n) CHECK_MSG(r == FrameResult::kNoFrame, name);
      }
      CHECK_MSG(r == FrameResult::kFrame, name);
      CHECK_MSG(dec.size() == len, name);
      CHECK_MSG(len == 0 || memcmp(dec.data(), payload, len) == 0, name);
      ++expected_frames;
    }
  }
  CHECK_MSG(dec.stats().ok == expected_frames, name);
  CHECK_MSG(dec.stats().bad_check == 0 && dec.stats().bad_encoding == 0 &&
                dec.stats().overflow == 0,
            name);

  // Too long is refused.
  uint8_t big[N + 1];
  fill_special(big, N + 1);
  CHECK_MSG(Enc::encode(big, N + 1, frame, sizeof frame) == 0, name);
  CHECK_MSG(Enc::frame_size(big, N + 1) == 0, name);
}

static void test_roundtrips() {
  using namespace mcu;
  test_roundtrip<Cobs, NoCheck, 40, false>("cobs/none");
  test_roundtrip<Cobs, Crc8, 40, false>("cobs/crc8");
  test_roundtrip<Cobs, Crc16Modbus, 40, true>("cobs/crc16-modbus-le");
  test_roundtrip<Cobs, Crc16CcittFalse, 40, false>("cobs/crc16-ccitt");
  test_roundtrip<Cobs, Crc32, 40, false>("cobs/crc32");
  test_roundtrip<Cobs, Crc32c, 40, true>("cobs/crc32c-le");
  test_roundtrip<Cobs, Fletcher16, 40, false>("cobs/fletcher16");
  test_roundtrip<Cobs, Fletcher32, 40, false>("cobs/fletcher32");
  test_roundtrip<Cobs, Xor8, 40, false>("cobs/xor8");
  test_roundtrip<Cobs, Lrc8, 40, false>("cobs/lrc8");

  test_roundtrip<Slip, NoCheck, 40, false>("slip/none");
  test_roundtrip<Slip, Crc8, 40, false>("slip/crc8");
  test_roundtrip<Slip, Crc16Modbus, 40, true>("slip/crc16-modbus-le");
  test_roundtrip<Slip, Crc16CcittFalse, 40, false>("slip/crc16-ccitt");
  test_roundtrip<Slip, Crc32, 40, false>("slip/crc32");
  test_roundtrip<Slip, Crc32c, 40, true>("slip/crc32c-le");
  test_roundtrip<Slip, Fletcher16, 40, false>("slip/fletcher16");
  test_roundtrip<Slip, Fletcher32, 40, false>("slip/fletcher32");
  test_roundtrip<Slip, Xor8, 40, false>("slip/xor8");
  test_roundtrip<Slip, Lrc8, 40, false>("slip/lrc8");

  // The nibble-table CRCs are the same CRC.
  test_roundtrip<Cobs, Crc32::Fast, 40, false>("cobs/crc32-fast");
  test_roundtrip<Slip, Crc16Modbus::Fast, 40, true>("slip/crc16-fast");
}

// Long payloads: many COBS blocks, and the 254/255-byte block boundaries.
template <typename Codec>
static void test_long_payloads(const char* name) {
  typedef mcu::FrameEncoder<Codec, mcu::Crc16Modbus, 1000, true> Enc;
  typedef mcu::FrameDecoder<Codec, mcu::Crc16Modbus, 1000, true> Dec;
  static uint8_t payload[1000];
  static uint8_t frame[Enc::max_frame_size()];
  Dec dec;
  const size_t lens[] = {1, 2, 252, 253, 254, 255, 256, 507, 508, 509, 761, 762, 1000};
  for (size_t li = 0; li < sizeof lens / sizeof lens[0]; ++li) {
    const size_t len = lens[li];
    for (int pattern = 0; pattern < 4; ++pattern) {
      for (size_t i = 0; i < len; ++i) {
        switch (pattern) {
          case 0: payload[i] = 0x55; break;                 // no zeros: longest COBS blocks
          case 1: payload[i] = 0x00; break;                 // all zeros
          case 2: payload[i] = 0xC0; break;                 // worst case for SLIP
          default: payload[i] = (uint8_t)(rnd8() | 1); break;  // random, never zero
        }
      }
      size_t n = Enc::encode(payload, len, frame, sizeof frame);
      CHECK_MSG(n > 0 && n <= Enc::max_frame_size(), name);
      FrameResult r = FrameResult::kNoFrame;
      for (size_t i = 0; i < n; ++i) r = dec.push(frame[i]);
      CHECK_MSG(r == FrameResult::kFrame, name);
      CHECK_MSG(dec.size() == len && memcmp(dec.data(), payload, len) == 0, name);
    }
  }
}

// ---------------------------------------------------------------------------
// Check byte order on the wire
// ---------------------------------------------------------------------------
static void test_check_byte_order() {
  const uint8_t p[3] = {0x10, 0x20, 0x30};
  const uint16_t crc = mcu::Crc16Modbus::compute(p, 3);

  uint8_t f[16];
  mcu::FrameDecoder<Cobs, NoCheck, 8> raw;  // sees payload + check as plain bytes

  size_t n = mcu::FrameEncoder<Cobs, mcu::Crc16Modbus, 8, true>::encode(p, 3, f, sizeof f);
  FrameResult r = FrameResult::kNoFrame;
  for (size_t i = 0; i < n; ++i) r = raw.push(f[i]);
  CHECK(r == FrameResult::kFrame && raw.size() == 5);
  CHECK(raw.data()[3] == (uint8_t)(crc & 0xFF) && raw.data()[4] == (uint8_t)(crc >> 8));

  n = mcu::FrameEncoder<Cobs, mcu::Crc16Modbus, 8, false>::encode(p, 3, f, sizeof f);
  for (size_t i = 0; i < n; ++i) r = raw.push(f[i]);
  CHECK(r == FrameResult::kFrame && raw.size() == 5);
  CHECK(raw.data()[3] == (uint8_t)(crc >> 8) && raw.data()[4] == (uint8_t)(crc & 0xFF));

  // A decoder with the wrong byte order rejects the frame.
  mcu::FrameDecoder<Cobs, mcu::Crc16Modbus, 8, true> le;
  for (size_t i = 0; i < n; ++i) r = le.push(f[i]);
  CHECK(r == FrameResult::kBadCheck);
}

// ---------------------------------------------------------------------------
// Corruption: every single-bit flip must be rejected, and the decoder must
// recover for the next frame.
// ---------------------------------------------------------------------------
template <typename Codec>
static void test_corruption(const char* name) {
  typedef mcu::FrameEncoder<Codec, mcu::Crc32, 64> Enc;
  typedef mcu::FrameDecoder<Codec, mcu::Crc32, 64> Dec;

  uint8_t payload[60], good[20];
  fill_special(payload, sizeof payload);
  for (size_t i = 0; i < sizeof good; ++i) good[i] = (uint8_t)(0xA0 + i);

  uint8_t frame[Enc::max_frame_size()], next[Enc::max_frame_size()];
  const size_t n = Enc::encode(payload, sizeof payload, frame, sizeof frame);
  const size_t m = Enc::encode(good, sizeof good, next, sizeof next);
  CHECK_MSG(n > 0 && m > 0, name);

  for (size_t i = 0; i < n; ++i) {
    for (int bit = 0; bit < 8; ++bit) {
      uint8_t bad[Enc::max_frame_size()];
      memcpy(bad, frame, n);
      bad[i] = (uint8_t)(bad[i] ^ (1u << bit));

      Dec dec;
      int frames = 0, good_frames = 0;
      FrameResult r;
      for (size_t k = 0; k < n; ++k) {
        r = dec.push(bad[k]);
        if (r == FrameResult::kFrame) ++frames;  // corrupted data must never pass
      }
      // An idle delimiter flushes a frame whose own delimiter was destroyed;
      // then a good frame must get through.
      r = dec.push(Codec::delimiter());
      if (r == FrameResult::kFrame) ++frames;
      for (size_t k = 0; k < m; ++k) {
        r = dec.push(next[k]);
        if (r == FrameResult::kFrame) {
          ++good_frames;
          CHECK_MSG(dec.size() == sizeof good && memcmp(dec.data(), good, sizeof good) == 0, name);
        }
      }
      CHECK_MSG(frames == 0, name);
      CHECK_MSG(good_frames == 1, name);
    }
  }
}

// Garbage before a frame: the leading delimiter discards it (SLIP), and COBS
// always resynchronises on the next 0x00.
template <typename Codec>
static void test_resync(const char* name) {
  typedef mcu::FrameEncoder<Codec, mcu::Crc16Modbus, 32, true> Enc;
  typedef mcu::FrameDecoder<Codec, mcu::Crc16Modbus, 32, true> Dec;
  const uint8_t payload[4] = {1, 2, 3, 4};
  const uint8_t garbage[5] = {0x55, 0x56, 0x57, 0x58, 0x59};  // no delimiter, no escape

  uint8_t frame[Enc::max_frame_size()];
  const size_t n = Enc::encode(payload, 4, frame, sizeof frame, /*leading_delimiter=*/true);

  Dec dec;
  for (size_t i = 0; i < sizeof garbage; ++i) CHECK_MSG(dec.push(garbage[i]) == FrameResult::kNoFrame, name);
  // the leading delimiter terminates the garbage as a rejected frame
  FrameResult r = dec.push(frame[0]);
  CHECK_MSG(r == FrameResult::kBadEncoding || r == FrameResult::kBadCheck, name);
  for (size_t i = 1; i < n; ++i) r = dec.push(frame[i]);
  CHECK_MSG(r == FrameResult::kFrame, name);
  CHECK_MSG(dec.size() == 4 && memcmp(dec.data(), payload, 4) == 0, name);
  CHECK_MSG(dec.stats().ok == 1, name);
}

static void test_overflow() {
  typedef mcu::FrameEncoder<Cobs, mcu::Crc16Modbus, 64, true> BigEnc;
  typedef mcu::FrameDecoder<Cobs, mcu::Crc16Modbus, 16, true> SmallDec;
  typedef mcu::FrameEncoder<Cobs, mcu::Crc16Modbus, 16, true> SmallEnc;

  uint8_t payload[40];
  fill_special(payload, sizeof payload);
  uint8_t big[BigEnc::max_frame_size()];
  const size_t n = BigEnc::encode(payload, sizeof payload, big, sizeof big);

  SmallDec dec;
  FrameResult r = FrameResult::kNoFrame;
  for (size_t i = 0; i < n; ++i) {
    r = dec.push(big[i]);
    if (i + 1 < n) CHECK(r == FrameResult::kNoFrame);
  }
  CHECK(r == FrameResult::kOverflow);
  CHECK(dec.stats().overflow == 1);

  // a frame of exactly the maximum size still fits
  uint8_t full[16], buf[SmallEnc::max_frame_size()];
  fill_special(full, sizeof full);
  const size_t m = SmallEnc::encode(full, sizeof full, buf, sizeof buf);
  for (size_t i = 0; i < m; ++i) r = dec.push(buf[i]);
  CHECK(r == FrameResult::kFrame);
  CHECK(dec.size() == 16 && memcmp(dec.data(), full, 16) == 0);
}

static void test_too_short_for_check() {
  // A well-formed frame holding fewer bytes than the check is a check failure.
  mcu::FrameEncoder<Cobs, NoCheck, 8> raw;
  uint8_t f[8];
  const uint8_t one[1] = {0x42};
  size_t n = raw.encode(one, 1, f, sizeof f);
  mcu::FrameDecoder<Cobs, mcu::Crc32, 8> dec;
  FrameResult r = FrameResult::kNoFrame;
  for (size_t i = 0; i < n; ++i) r = dec.push(f[i]);
  CHECK(r == FrameResult::kBadCheck);
  CHECK(dec.stats().bad_check == 1);
}

// ---------------------------------------------------------------------------
// Over a Stream
// ---------------------------------------------------------------------------
template <typename Codec>
static void test_stream(const char* name) {
  typedef mcu::FrameEncoder<Codec, mcu::Crc16Modbus, 24, true> Enc;
  typedef mcu::FrameDecoder<Codec, mcu::Crc16Modbus, 24, true> Dec;

  mcu::LoopbackStream<256, 256> wire;
  Dec dec;

  // Send several frames; nothing is buffered by the encoder.
  uint8_t msgs[5][24];
  size_t lens[5] = {1, 7, 24, 3, 12};
  for (int i = 0; i < 5; ++i) {
    fill_special(msgs[i], lens[i]);
    CHECK_MSG(Enc::send(wire, msgs[i], lens[i]), name);
  }

  // Move the bytes across the "wire" (tx side -> rx side), in awkward chunks.
  uint8_t tmp[7];
  size_t moved;
  while ((moved = wire.take(tmp, sizeof tmp)) > 0) CHECK_MSG(wire.inject(tmp, moved) == moved, name);

  // Receive them in order, one poll() per event.
  int got = 0;
  for (;;) {
    FrameResult r = dec.poll(wire);
    if (r == FrameResult::kNoFrame) break;
    CHECK_MSG(r == FrameResult::kFrame, name);
    CHECK_MSG(got < 5 && dec.size() == lens[got] && memcmp(dec.data(), msgs[got], lens[got]) == 0, name);
    ++got;
  }
  CHECK_MSG(got == 5, name);
  CHECK_MSG(wire.available() == 0, name);

  // A frame split across two polls is reassembled.
  uint8_t frame[Enc::max_frame_size()];
  const size_t n = Enc::encode(msgs[1], lens[1], frame, sizeof frame);
  wire.inject(frame, n / 2);
  CHECK_MSG(dec.poll(wire) == FrameResult::kNoFrame, name);
  wire.inject(frame + n / 2, n - n / 2);
  CHECK_MSG(dec.poll(wire) == FrameResult::kFrame, name);
  CHECK_MSG(dec.size() == lens[1] && memcmp(dec.data(), msgs[1], lens[1]) == 0, name);
}

static void test_send_backpressure() {
  typedef mcu::FrameEncoder<Slip, mcu::Crc16Modbus, 16, true> Enc;
  mcu::LoopbackStream<8, 16> port;
  const uint8_t p[4] = {1, 2, 3, 4};  // SLIP + 2 check bytes: 7 bytes if nothing needs escaping
  const size_t need = Enc::frame_size(p, 4);
  CHECK(need >= 7 && need <= 14);

  CHECK(Enc::send(port, p, 4));
  CHECK(port.tx_size() == need);

  // Not enough space for another whole frame: nothing is written at all.
  const size_t before = port.tx_size();
  if (port.write_space() < need) {
    CHECK(!Enc::send(port, p, 4));
    CHECK(port.tx_size() == before);
  }

  // Payloads that can never be sent are refused without touching the stream.
  uint8_t big[17] = {0};
  CHECK(!Enc::send(port, big, 17));
  CHECK(port.tx_size() == before);

  // Wire output equals encode() output.
  uint8_t direct[Enc::max_frame_size()];
  uint8_t sent[32];
  port.clear();
  CHECK(Enc::send(port, p, 4, true));
  const size_t s = port.take(sent, sizeof sent);
  const size_t d = Enc::encode(p, 4, direct, sizeof direct, true);
  CHECK(s == d && memcmp(sent, direct, d) == 0);
  CHECK(sent[0] == Slip::delimiter());
}

// A Stream that lies: write_space() says plenty, but write() takes only a few.
class StingyStream final : public mcu::Stream {
 public:
  StingyStream() : taken_(0) {}
  size_t read(uint8_t*, size_t) override { return 0; }
  size_t write(const uint8_t*, size_t len) override {
    size_t room = 10 - taken_;
    size_t n = len < room ? len : room;
    taken_ += n;
    return n;
  }
  size_t available() const override { return 0; }
  size_t write_space() const override { return mcu::kUnboundedSpace; }

 private:
  size_t taken_;
};

static void test_send_reports_stream_failure() {
  typedef mcu::FrameEncoder<Cobs, NoCheck, 100> Enc;
  StingyStream s;
  uint8_t p[60];
  fill_special(p, sizeof p);
  CHECK(!Enc::send(s, p, sizeof p));  // stream stopped accepting: reported, not ignored
}

static void test_stats() {
  mcu::FrameEncoder<Slip, mcu::Crc16Modbus, 16> enc;
  mcu::FrameDecoder<Slip, mcu::Crc16Modbus, 16> dec;
  uint8_t f[40];
  const uint8_t p[3] = {1, 2, 3};
  size_t n = enc.encode(p, 3, f, sizeof f);
  for (size_t i = 0; i < n; ++i) dec.push(f[i]);
  f[1] ^= 0x10;
  for (size_t i = 0; i < n; ++i) dec.push(f[i]);
  CHECK(dec.stats().ok == 1);
  CHECK(dec.stats().bad_check == 1);
  dec.clear_stats();
  CHECK(dec.stats().ok == 0 && dec.stats().bad_check == 0);
}

static void test_payload_valid_until_next_push() {
  mcu::FrameEncoder<Cobs, NoCheck, 8> enc;
  mcu::FrameDecoder<Cobs, NoCheck, 8> dec;
  uint8_t f[16];
  const uint8_t p[2] = {7, 8};
  size_t n = enc.encode(p, 2, f, sizeof f);
  FrameResult r = FrameResult::kNoFrame;
  for (size_t i = 0; i < n; ++i) r = dec.push(f[i]);
  CHECK(r == FrameResult::kFrame);
  CHECK(dec.size() == 2 && dec.data()[0] == 7 && dec.data()[1] == 8);
  dec.reset();
  CHECK(dec.size() == 0);
}

int main() {
  test_cobs_vectors();
  test_slip_vectors();
  test_slip_bad_escape();
  test_cobs_truncated();
  test_roundtrips();
  test_long_payloads<Cobs>("long/cobs");
  test_long_payloads<Slip>("long/slip");
  test_check_byte_order();
  test_corruption<Cobs>("corrupt/cobs");
  test_corruption<Slip>("corrupt/slip");
  test_resync<Cobs>("resync/cobs");
  test_resync<Slip>("resync/slip");
  test_overflow();
  test_too_short_for_check();
  test_stream<Cobs>("stream/cobs");
  test_stream<Slip>("stream/slip");
  test_send_backpressure();
  test_send_reports_stream_failure();
  test_stats();
  test_payload_valid_until_next_push();

  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
