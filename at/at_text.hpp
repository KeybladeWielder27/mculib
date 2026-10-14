// at_text.hpp - building and parsing the text of AT commands, without printf.
//
//   mcu::TextBuilder<24> cmd;
//   cmd.add("ATID").add_hex(0x7FFF);          // "ATID7FFF"
//   if (cmd.ok()) at.command(cmd.c_str(), now);
//
//   uint32_t pan;
//   if (mcu::parse_hex_u32(at.response(), pan)) { ... }   // reply "7FFF" -> 0x7FFF
//
// Header-only, C++11, no heap.

#ifndef MCU_AT_TEXT_HPP
#define MCU_AT_TEXT_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

// ---------------------------------------------------------------------------
// TextBuilder<N>: appends text into a fixed buffer of N bytes (N-1 characters
// plus the terminating NUL). Anything that does not fit is dropped and ok()
// becomes false, so one check after building is enough.
// ---------------------------------------------------------------------------
template <size_t N>
class TextBuilder {
  static_assert(N >= 2, "N must be at least 2");

 public:
  TextBuilder() : len_(0), overflow_(false) { buf_[0] = 0; }

  TextBuilder& add(char c) {
    if (len_ + 1 < N) {
      buf_[len_++] = c;
      buf_[len_] = 0;
    } else {
      overflow_ = true;
    }
    return *this;
  }

  TextBuilder& add(const char* s) {
    if (s == 0) return *this;
    while (*s) add(*s++);
    return *this;
  }

  // Upper-case hex, no prefix, at least `min_digits` digits (zero padded).
  TextBuilder& add_hex(uint64_t v, unsigned min_digits = 1) {
    char tmp[16];
    unsigned n = 0;
    do {
      const unsigned d = (unsigned)(v & 0xFu);
      tmp[n++] = (char)(d < 10 ? '0' + d : 'A' + (d - 10));
      v >>= 4;
    } while (v != 0 && n < 16);
    if (min_digits > 16) min_digits = 16;
    while (n < min_digits) tmp[n++] = '0';
    while (n > 0) add(tmp[--n]);
    return *this;
  }

  TextBuilder& add_dec(uint32_t v) {
    char tmp[10];
    unsigned n = 0;
    do {
      tmp[n++] = (char)('0' + v % 10u);
      v /= 10u;
    } while (v != 0);
    while (n > 0) add(tmp[--n]);
    return *this;
  }

  TextBuilder& add_dec(int32_t v) {
    if (v < 0) {
      add('-');
      return add_dec((uint32_t)(0u - (uint32_t)v));
    }
    return add_dec((uint32_t)v);
  }

  const char* c_str() const { return buf_; }
  size_t size() const { return len_; }
  static constexpr size_t capacity() { return N - 1; }
  bool ok() const { return !overflow_; }  // false if anything was dropped

  void clear() {
    len_ = 0;
    overflow_ = false;
    buf_[0] = 0;
  }

 private:
  char buf_[N];
  size_t len_;
  bool overflow_;
};

namespace detail {
inline int hex_digit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}
}  // namespace detail

// Bare hex as AT devices print it: 1 to 16 digits, upper or lower case, no
// prefix, no spaces, nothing after. Leading zeros are allowed beyond 16
// digits only if the value still fits. Returns false (out untouched) otherwise.
inline bool parse_hex_u64(const char* s, uint64_t& out) {
  if (s == 0 || *s == 0) return false;
  uint64_t v = 0;
  for (; *s; ++s) {
    const int d = detail::hex_digit(*s);
    if (d < 0) return false;
    if (v >> 60) return false;  // another digit would overflow
    v = (v << 4) | (uint64_t)d;
  }
  out = v;
  return true;
}

inline bool parse_hex_u32(const char* s, uint32_t& out) {
  uint64_t v;
  if (!parse_hex_u64(s, v) || v > 0xFFFFFFFFull) return false;
  out = (uint32_t)v;
  return true;
}

// Plain unsigned decimal, 1 or more digits, nothing else.
inline bool parse_dec_u32(const char* s, uint32_t& out) {
  if (s == 0 || *s == 0) return false;
  uint32_t v = 0;
  for (; *s; ++s) {
    if (*s < '0' || *s > '9') return false;
    const uint32_t d = (uint32_t)(*s - '0');
    if (v > (0xFFFFFFFFu - d) / 10u) return false;
    v = v * 10u + d;
  }
  out = v;
  return true;
}

}  // namespace mcu

#endif  // MCU_AT_TEXT_HPP
