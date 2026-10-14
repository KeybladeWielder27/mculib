// checksum.hpp - Fletcher and simple additive/XOR checksums, microcontroller agnostic.
//
// Header-only, C++11, no heap, no platform headers, no division or modulo
// (the Fletcher sums reduce with a conditional subtract, which is cheap on
// small cores). Same interface as the CRCs in crc.hpp, so protocol code can
// be written once and used with any of them:
//
//   mcu::Fletcher16 c;
//   c.update(byte);  c.update(ptr, len);
//   uint16_t v = c.value();     // result so far, can keep updating afterwards
//   c.reset();
//   mcu::Fletcher16::compute(ptr, len);
//
// Note: Fletcher checksums are weaker than a CRC. Fletcher-16 cannot tell
// 0x00 from 0xFF, and neither Fletcher nor the simple sums detect reordering
// of bytes well in short messages. Use a CRC when you can afford it.

#ifndef MCU_CHECKSUM_HPP
#define MCU_CHECKSUM_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

// ---------------------------------------------------------------------------
// Fletcher-16: two running sums modulo 255 over bytes.
// Result is (sum2 << 8) | sum1.   Check: "abcde" -> 0xC8F0.
// ---------------------------------------------------------------------------
class Fletcher16 {
 public:
  typedef uint16_t value_type;

  Fletcher16() : s1_(0), s2_(0) {}

  void reset() { s1_ = 0; s2_ = 0; }

  void update(uint8_t byte) {
    s1_ = (uint16_t)(s1_ + byte);
    if (s1_ >= 255) s1_ = (uint16_t)(s1_ - 255);
    s2_ = (uint16_t)(s2_ + s1_);
    if (s2_ >= 255) s2_ = (uint16_t)(s2_ - 255);
  }

  void update(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    while (len--) update(*p++);
  }

  uint16_t value() const { return (uint16_t)((s2_ << 8) | s1_); }

  // Continue from a value returned earlier by value().
  void resume(uint16_t previous_value) {
    s1_ = (uint16_t)(previous_value & 0xFF);
    s2_ = (uint16_t)(previous_value >> 8);
  }

  static uint16_t compute(const void* data, size_t len) {
    Fletcher16 c;
    c.update(data, len);
    return c.value();
  }

 private:
  uint16_t s1_, s2_;
};

// ---------------------------------------------------------------------------
// Fletcher-32: two running sums modulo 65535 over 16-bit little-endian words.
// A trailing odd byte is treated as a word with a zero high byte.
// Result is (sum2 << 16) | sum1.   Check: "abcde" -> 0xF04FC729.
// Bytes may be fed in any chunking; the result is the same.
// ---------------------------------------------------------------------------
class Fletcher32 {
 public:
  typedef uint32_t value_type;

  Fletcher32() : s1_(0), s2_(0), lo_(0), have_lo_(false) {}

  void reset() { s1_ = 0; s2_ = 0; lo_ = 0; have_lo_ = false; }

  void update(uint8_t byte) {
    if (have_lo_) {
      add_word(s1_, s2_, (uint16_t)(lo_ | ((uint16_t)byte << 8)));
      have_lo_ = false;
    } else {
      lo_ = byte;
      have_lo_ = true;
    }
  }

  void update(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    while (len--) update(*p++);
  }

  // Does not disturb the running state, so odd-length data can keep growing.
  uint32_t value() const {
    uint32_t a = s1_, b = s2_;
    if (have_lo_) add_word(a, b, lo_);
    return (b << 16) | a;
  }

  static uint32_t compute(const void* data, size_t len) {
    Fletcher32 c;
    c.update(data, len);
    return c.value();
  }

 private:
  static void add_word(uint32_t& a, uint32_t& b, uint16_t word) {
    const uint32_t kMod = 65535;
    a += word;
    if (a >= kMod) a -= kMod;
    b += a;
    if (b >= kMod) b -= kMod;
  }

  uint32_t s1_, s2_;
  uint8_t lo_;
  bool have_lo_;
};

// ---------------------------------------------------------------------------
// Xor8: XOR of all bytes (NMEA 0183 sentences, many simple serial protocols).
// ---------------------------------------------------------------------------
class Xor8 {
 public:
  typedef uint8_t value_type;

  Xor8() : v_(0) {}
  void reset() { v_ = 0; }
  void update(uint8_t byte) { v_ = (uint8_t)(v_ ^ byte); }
  void update(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    while (len--) update(*p++);
  }
  uint8_t value() const { return v_; }
  void resume(uint8_t previous_value) { v_ = previous_value; }
  static uint8_t compute(const void* data, size_t len) {
    Xor8 c;
    c.update(data, len);
    return c.value();
  }

 private:
  uint8_t v_;
};

// ---------------------------------------------------------------------------
// Sum8: sum of all bytes modulo 256.
// ---------------------------------------------------------------------------
class Sum8 {
 public:
  typedef uint8_t value_type;

  Sum8() : v_(0) {}
  void reset() { v_ = 0; }
  void update(uint8_t byte) { v_ = (uint8_t)(v_ + byte); }
  void update(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    while (len--) update(*p++);
  }
  uint8_t value() const { return v_; }
  void resume(uint8_t previous_value) { v_ = previous_value; }
  static uint8_t compute(const void* data, size_t len) {
    Sum8 c;
    c.update(data, len);
    return c.value();
  }

 private:
  uint8_t v_;
};

// ---------------------------------------------------------------------------
// Lrc8: two's complement of the byte sum (Modbus ASCII LRC, Intel HEX
// record checksum). Appending the result to the data makes the whole
// message sum to zero modulo 256.
// ---------------------------------------------------------------------------
class Lrc8 {
 public:
  typedef uint8_t value_type;

  Lrc8() : sum_(0) {}
  void reset() { sum_ = 0; }
  void update(uint8_t byte) { sum_ = (uint8_t)(sum_ + byte); }
  void update(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    while (len--) update(*p++);
  }
  uint8_t value() const { return (uint8_t)(0u - sum_); }
  void resume(uint8_t previous_value) { sum_ = (uint8_t)(0u - previous_value); }
  static uint8_t compute(const void* data, size_t len) {
    Lrc8 c;
    c.update(data, len);
    return c.value();
  }

 private:
  uint8_t sum_;
};

}  // namespace mcu

#endif  // MCU_CHECKSUM_HPP
