// parse.hpp - strict number and value parsing for command arguments.
//
// Everything here validates: no trailing junk, no empty input, no overflow,
// range checked. On failure false is returned and the output value is left
// untouched (parse_hex_bytes may have written part of its buffer; ignore it).
// No floating point, no strtol, no heap.
//
//   uint32_t addr;  mcu::parse_uint("0x1F40", addr);          // 8000
//   int32_t  delta; mcu::parse_int("-250", delta);
//   bool     on;    mcu::parse_bool("ON", on);                 // true
//   int32_t  mv;    mcu::parse_scaled("3.30", mv, 3);          // 3300 (milli-units)
//   uint8_t  key[16]; size_t n; mcu::parse_hex_bytes("00FF10", key, sizeof key, n);
//
// Header-only, C++11.

#ifndef MCU_PARSE_HPP
#define MCU_PARSE_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

constexpr int32_t kInt32Min = (int32_t)(-2147483647 - 1);
constexpr int32_t kInt32Max = (int32_t)2147483647;

// Decimal, hex with a 0x prefix, or binary with a 0b prefix. No sign.
// Leading zeros are fine and mean decimal ("007" is 7). Fails above `max`.
inline bool parse_uint(const char* s, uint32_t& out, uint32_t max = 0xFFFFFFFFu) {
  if (s == nullptr || *s == '\0') return false;
  uint32_t base = 10;
  if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    base = 16;
    s += 2;
  } else if (s[0] == '0' && (s[1] == 'b' || s[1] == 'B')) {
    base = 2;
    s += 2;
  }
  if (*s == '\0') return false;
  uint32_t v = 0;
  for (; *s != '\0'; ++s) {
    uint32_t d;
    const char c = *s;
    if (c >= '0' && c <= '9') {
      d = (uint32_t)(c - '0');
    } else if (c >= 'a' && c <= 'f') {
      d = (uint32_t)(c - 'a') + 10u;
    } else if (c >= 'A' && c <= 'F') {
      d = (uint32_t)(c - 'A') + 10u;
    } else {
      return false;
    }
    if (d >= base) return false;
    if (v > (0xFFFFFFFFu - d) / base) return false;  // would overflow
    v = v * base + d;
  }
  if (v > max) return false;
  out = v;
  return true;
}

namespace detail {
// Applies a sign to a magnitude known to fit, without signed overflow.
inline int32_t apply_sign(bool neg, uint32_t mag) {
  if (neg && mag != 0) return -(int32_t)(mag - 1) - 1;
  return (int32_t)mag;
}
}  // namespace detail

// Optional + or -, then the same forms as parse_uint ("-0x10" is -16).
inline bool parse_int(const char* s, int32_t& out, int32_t min = kInt32Min,
                      int32_t max = kInt32Max) {
  if (s == nullptr) return false;
  bool neg = false;
  if (*s == '-') {
    neg = true;
    ++s;
  } else if (*s == '+') {
    ++s;
  }
  uint32_t mag;
  if (!parse_uint(s, mag, neg ? 2147483648u : 2147483647u)) return false;
  const int32_t v = detail::apply_sign(neg, mag);
  if (v < min || v > max) return false;
  out = v;
  return true;
}

namespace detail {
inline char lower_char(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c; }

inline bool iequals(const char* a, const char* b) {
  for (;; ++a, ++b) {
    if (lower_char(*a) != lower_char(*b)) return false;
    if (*a == '\0') return true;
  }
}
}  // namespace detail

// 1/0, on/off, true/false, yes/no, y/n, in any letter case.
inline bool parse_bool(const char* s, bool& out) {
  if (s == nullptr) return false;
  static const char* const kTrue[] = {"1", "on", "true", "yes", "y"};
  static const char* const kFalse[] = {"0", "off", "false", "no", "n"};
  for (size_t i = 0; i < sizeof kTrue / sizeof kTrue[0]; ++i) {
    if (detail::iequals(s, kTrue[i])) {
      out = true;
      return true;
    }
    if (detail::iequals(s, kFalse[i])) {
      out = false;
      return true;
    }
  }
  return false;
}

// A decimal with up to `decimals` fractional digits, returned as an integer in
// units of 10^-decimals, with no floating point:
//   parse_scaled("3.14", v, 2) -> 314      parse_scaled("-0.5", v, 1) -> -5
//   parse_scaled("5", v, 2)    -> 500      parse_scaled(".25", v, 2)  -> 25
// More fractional digits than `decimals` are rejected rather than rounded away.
// `decimals` is 0..9.
inline bool parse_scaled(const char* s, int32_t& out, unsigned decimals) {
  if (s == nullptr || decimals > 9) return false;
  bool neg = false;
  if (*s == '-') {
    neg = true;
    ++s;
  } else if (*s == '+') {
    ++s;
  }
  const uint32_t limit = neg ? 2147483648u : 2147483647u;
  uint32_t mag = 0;
  unsigned frac = 0;
  bool any = false, dot = false;
  for (; *s != '\0'; ++s) {
    if (*s == '.') {
      if (dot) return false;
      dot = true;
      continue;
    }
    if (*s < '0' || *s > '9') return false;
    if (dot) {
      if (frac == decimals) return false;
      ++frac;
    }
    any = true;
    const uint32_t d = (uint32_t)(*s - '0');
    if (mag > (limit - d) / 10u) return false;
    mag = mag * 10u + d;
  }
  if (!any) return false;
  for (; frac < decimals; ++frac) {
    if (mag > limit / 10u) return false;
    mag *= 10u;
  }
  out = detail::apply_sign(neg, mag);
  return true;
}

// An even number of hex digits, no separators or prefix, upper or lower case:
// "00ff10" -> {0x00, 0xFF, 0x10}. Fails if it would not fit in `cap` bytes.
inline bool parse_hex_bytes(const char* s, uint8_t* out, size_t cap, size_t& n) {
  if (s == nullptr) return false;
  size_t digits = 0;
  while (s[digits] != '\0') ++digits;
  if (digits == 0 || (digits & 1u) != 0 || digits / 2 > cap) return false;
  for (size_t i = 0; i < digits / 2; ++i) {
    uint32_t byte = 0;
    for (int k = 0; k < 2; ++k) {
      const char c = s[2 * i + (size_t)k];
      uint32_t d;
      if (c >= '0' && c <= '9') {
        d = (uint32_t)(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        d = (uint32_t)(c - 'a') + 10u;
      } else if (c >= 'A' && c <= 'F') {
        d = (uint32_t)(c - 'A') + 10u;
      } else {
        return false;
      }
      byte = (byte << 4) | d;
    }
    out[i] = (uint8_t)byte;
  }
  n = digits / 2;
  return true;
}

}  // namespace mcu

#endif  // MCU_PARSE_HPP
