// fixed.hpp - integer math helpers for microcontrollers: saturating arithmetic,
// range mapping, integer square root and Q-format fixed point.
//
// Everything here is plain integer code: no floating point, no <limits>, no
// heap. Header-only, C++11.
//
//   mcu::sat_add<uint8_t>(200, 100)              // 255, not 44
//   mcu::map_range(adc, 0, 1023, 0, 3300)        // ADC counts to millivolts, rounded and clamped
//   mcu::isqrt(1000000u)                         // 1000
//
//   typedef mcu::Q<16> Q16;                      // 16 integer bits . 16 fraction bits
//   Q16 gain = Q16::from_ratio(3, 4);            // 0.75
//   Q16 y = gain * Q16::from_int(100);           // 75
//   int32_t n = y.to_int();                      // 75
//
// Note: map_range() and Q multiply/divide use 64-bit intermediates so they
// cannot overflow. On 8-bit MCUs these are slower than 32-bit math; they are
// still fine at sensor rates.

#ifndef MCU_FIXED_HPP
#define MCU_FIXED_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

namespace detail {

template <bool B>
struct BoolTag {};

// Properties of an integer type chosen by its size and signedness, so it works
// for whatever int32_t or int happens to be on the platform.
template <size_t Bytes, bool Signed>
struct IntBySize;
template <>
struct IntBySize<1, true> { typedef int32_t wide; typedef uint8_t unsigned_type; enum { is_signed = 1 }; };
template <>
struct IntBySize<1, false> { typedef uint32_t wide; typedef uint8_t unsigned_type; enum { is_signed = 0 }; };
template <>
struct IntBySize<2, true> { typedef int32_t wide; typedef uint16_t unsigned_type; enum { is_signed = 1 }; };
template <>
struct IntBySize<2, false> { typedef uint32_t wide; typedef uint16_t unsigned_type; enum { is_signed = 0 }; };
template <>
struct IntBySize<4, true> { typedef int64_t wide; typedef uint32_t unsigned_type; enum { is_signed = 1 }; };
template <>
struct IntBySize<4, false> { typedef uint64_t wide; typedef uint32_t unsigned_type; enum { is_signed = 0 }; };
template <>
struct IntBySize<8, true> { typedef int64_t wide; typedef uint64_t unsigned_type; enum { is_signed = 1 }; };
template <>
struct IntBySize<8, false> { typedef uint64_t wide; typedef uint64_t unsigned_type; enum { is_signed = 0 }; };

// `wide` is a type that can hold sums of many T values; `unsigned_type` is T's
// unsigned counterpart. Only integer types are accepted.
template <typename T>
struct IntTraits : IntBySize<sizeof(T), ((T)(~(T)0) < (T)1)> {};

template <typename T>
constexpr T int_max() {
  return IntTraits<T>::is_signed ? (T)((T)(((T)1 << (sizeof(T) * 8 - 2)) - 1) * 2 + 1)
                                 : (T)~(T)0;
}
template <typename T>
constexpr T int_min() {
  return IntTraits<T>::is_signed ? (T)(-int_max<T>() - 1) : (T)0;
}

template <typename T>
inline T sat_add_impl(T a, T b, BoolTag<true>) {  // signed
  if (b > 0) {
    if (a > (T)(int_max<T>() - b)) return int_max<T>();
  } else if (b < 0) {
    if (a < (T)(int_min<T>() - b)) return int_min<T>();
  }
  return (T)(a + b);
}
template <typename T>
inline T sat_add_impl(T a, T b, BoolTag<false>) {  // unsigned
  if (a > (T)(int_max<T>() - b)) return int_max<T>();
  return (T)(a + b);
}
template <typename T>
inline T sat_sub_impl(T a, T b, BoolTag<true>) {  // signed
  if (b > 0) {
    if (a < (T)(int_min<T>() + b)) return int_min<T>();
  } else if (b < 0) {
    if (a > (T)(int_max<T>() + b)) return int_max<T>();
  }
  return (T)(a - b);
}
template <typename T>
inline T sat_sub_impl(T a, T b, BoolTag<false>) {  // unsigned
  return a < b ? (T)0 : (T)(a - b);
}

// num / den rounded to nearest, halves away from zero. den > 0.
template <typename T>
inline T div_round_impl(T num, T den, BoolTag<true>) {  // signed
  return num >= 0 ? (T)((num + den / 2) / den) : (T)((num - den / 2) / den);
}
template <typename T>
inline T div_round_impl(T num, T den, BoolTag<false>) {  // unsigned
  return (T)((num + den / 2) / den);
}
template <typename T>
inline T div_round(T num, T den) {
  return div_round_impl(num, den, BoolTag<(bool)IntTraits<T>::is_signed>());
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

template <typename T>
constexpr T clamp(T v, T lo, T hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// a + b and a - b that stop at the limits of T instead of wrapping.
template <typename T>
inline T sat_add(T a, T b) {
  return detail::sat_add_impl(a, b, detail::BoolTag<(bool)detail::IntTraits<T>::is_signed>());
}
template <typename T>
inline T sat_sub(T a, T b) {
  return detail::sat_sub_impl(a, b, detail::BoolTag<(bool)detail::IntTraits<T>::is_signed>());
}

// Linear mapping of x from [in_lo, in_hi] to [out_lo, out_hi], rounded to the
// nearest integer and clamped to the input range, like Arduino's map() done
// properly. Either range may run backwards. If in_lo == in_hi the result is
// out_lo. Each range's span must fit in an int32 (it is not checked).
//
//   map_range(512, 0, 1023, 0, 3300)  -> 1652   (ADC counts to millivolts)
//   map_range(25, 0, 100, 255, 0)     -> 191    (reversed output)
inline int32_t map_range(int32_t x, int32_t in_lo, int32_t in_hi, int32_t out_lo,
                         int32_t out_hi) {
  if (in_lo == in_hi) return out_lo;
  const int32_t lo = in_lo < in_hi ? in_lo : in_hi;
  const int32_t hi = in_lo < in_hi ? in_hi : in_lo;
  x = x < lo ? lo : (x > hi ? hi : x);
  int64_t num = (int64_t)(x - (int64_t)in_lo) * ((int64_t)out_hi - (int64_t)out_lo);
  int64_t den = (int64_t)in_hi - (int64_t)in_lo;
  if (den < 0) {
    num = -num;
    den = -den;
  }
  return (int32_t)(out_lo + detail::div_round<int64_t>(num, den));
}

// Integer square root: the largest r with r*r <= n.
inline uint32_t isqrt(uint32_t n) {
  uint32_t r = 0;
  uint32_t bit = 1u << 30;
  while (bit > n) bit >>= 2;
  while (bit != 0) {
    if (n >= r + bit) {
      n -= r + bit;
      r = (r >> 1) + bit;
    } else {
      r >>= 1;
    }
    bit >>= 2;
  }
  return r;
}

inline uint32_t isqrt64(uint64_t n) {
  uint64_t r = 0;
  uint64_t bit = (uint64_t)1 << 62;
  while (bit > n) bit >>= 2;
  while (bit != 0) {
    if (n >= r + bit) {
      n -= r + bit;
      r = (r >> 1) + bit;
    } else {
      r >>= 1;
    }
    bit >>= 2;
  }
  return (uint32_t)r;
}

// ---------------------------------------------------------------------------
// Q<F>: signed fixed point with F fraction bits in an int32.
//
// All arithmetic saturates instead of wrapping, multiplication and division
// are rounded to nearest, and division by zero gives the largest (or
// smallest) value instead of crashing. Resolution is 2^-F; range is about
// +/- 2^(31-F).
// ---------------------------------------------------------------------------
template <unsigned FracBits>
class Q {
  static_assert(FracBits >= 1 && FracBits <= 30, "FracBits must be 1..30");

 public:
  constexpr Q() : raw_(0) {}

  // The underlying integer: value * 2^FracBits.
  static constexpr Q from_raw(int32_t raw) { return Q(raw, 0); }

  // For constants: Q<16>::from_double(0.75). Evaluated at compile time when
  // used in a constant; avoid with runtime floating-point values.
  static constexpr Q from_double(double d) {
    return Q((int32_t)(d * (double)(1u << FracBits) + (d >= 0 ? 0.5 : -0.5)), 0);
  }

  static Q from_int(int32_t i) { return Q(saturate((int64_t)i * one()), 0); }

  // num/den, rounded. den == 0 saturates.
  static Q from_ratio(int32_t num, int32_t den) {
    return Q(divide((int64_t)num * one(), (int64_t)den), 0);
  }

  static constexpr Q max_value() { return Q(0x7FFFFFFF, 0); }
  static constexpr Q min_value() { return Q((int32_t)(-0x7FFFFFFF - 1), 0); }

  constexpr int32_t raw() const { return raw_; }
  int32_t to_int() const { return (int32_t)detail::div_round<int64_t>(raw_, one()); }  // rounded
  double to_double() const { return (double)raw_ / (double)(1u << FracBits); }

  Q operator+(Q o) const { return Q(saturate((int64_t)raw_ + o.raw_), 0); }
  Q operator-(Q o) const { return Q(saturate((int64_t)raw_ - o.raw_), 0); }
  Q operator-() const { return Q(saturate(-(int64_t)raw_), 0); }
  Q operator*(Q o) const {
    return Q(saturate(detail::div_round<int64_t>((int64_t)raw_ * o.raw_, one())), 0);
  }
  Q operator/(Q o) const { return Q(divide((int64_t)raw_ * one(), (int64_t)o.raw_), 0); }
  Q& operator+=(Q o) { return *this = *this + o; }
  Q& operator-=(Q o) { return *this = *this - o; }
  Q& operator*=(Q o) { return *this = *this * o; }
  Q& operator/=(Q o) { return *this = *this / o; }

  bool operator==(Q o) const { return raw_ == o.raw_; }
  bool operator!=(Q o) const { return raw_ != o.raw_; }
  bool operator<(Q o) const { return raw_ < o.raw_; }
  bool operator>(Q o) const { return raw_ > o.raw_; }
  bool operator<=(Q o) const { return raw_ <= o.raw_; }
  bool operator>=(Q o) const { return raw_ >= o.raw_; }

  Q abs() const { return raw_ < 0 ? -*this : *this; }

 private:
  constexpr Q(int32_t raw, int) : raw_(raw) {}

  static int64_t one() { return (int64_t)1 << FracBits; }

  static int32_t saturate(int64_t v) {
    return v > 0x7FFFFFFFLL ? (int32_t)0x7FFFFFFF
                            : (v < -0x7FFFFFFFLL - 1 ? (int32_t)(-0x7FFFFFFF - 1) : (int32_t)v);
  }

  // num / den rounded to nearest, saturating; den == 0 gives +/-max (0 if num is 0).
  static int32_t divide(int64_t num, int64_t den) {
    if (den == 0) return num > 0 ? 0x7FFFFFFF : (num < 0 ? (int32_t)(-0x7FFFFFFF - 1) : 0);
    if (den < 0) {
      num = -num;
      den = -den;
    }
    return saturate(detail::div_round<int64_t>(num, den));
  }

  int32_t raw_;
};

}  // namespace mcu

#endif  // MCU_FIXED_HPP
