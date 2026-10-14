// crc.hpp - table-free or table-assisted CRC-8/16/32, microcontroller agnostic.
//
// - Header-only, C++11, no heap, no exceptions, no RTTI, no <type_traits>,
//   no platform headers (just <stddef.h> and <stdint.h>).
// - Any CRC of width 8, 16 or 32 whose input and output reflection match
//   (which covers nearly every CRC in common use) can be described by the
//   usual catalogue parameters: polynomial, initial value, reflected, xor-out.
// - Default algorithm is bitwise: no table, a few bytes of code, slowest.
//   Each CRC also has a `::Fast` variant that uses a 16-entry nibble table
//   (16 * sizeof(value) bytes of constant data, about 4x faster than bitwise).
//   On AVR constant data lives in RAM unless placed in PROGMEM, so prefer the
//   default bitwise variant there.
//
// Common interface (shared with the other checksums in checksum.hpp):
//   Crc16Modbus c;              // starts ready to use
//   c.update(byte);             // feed one byte
//   c.update(ptr, len);         // feed a buffer
//   uint16_t v = c.value();     // result so far (may keep updating afterwards)
//   c.reset();                  // start again
//   Crc16Modbus::compute(ptr, len);   // one-shot
//   c.resume(previous_value);   // continue from a value that was returned earlier
//
// Parameter sets follow the Rocksoft model / reveng catalogue. Check value is
// the CRC of the ASCII string "123456789".

#ifndef MCU_CRC_HPP
#define MCU_CRC_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

namespace detail {

// Bit-reverses the low `bits` bits of v. constexpr in C++11 style (single return).
template <typename T>
constexpr T crc_reflect(T v, unsigned bits) {
  return bits == 0 ? (T)0
                   : (T)((T)((T)(v & 1) << (bits - 1)) | crc_reflect<T>((T)(v >> 1), bits - 1));
}

// n shift-and-xor steps of the reflected algorithm.
template <typename T>
constexpr T crc_step_reflected(T v, T rpoly, unsigned n) {
  return n == 0 ? v
                : crc_step_reflected<T>(
                      (T)((v & 1) ? (T)((T)(v >> 1) ^ rpoly) : (T)(v >> 1)), rpoly, n - 1);
}

// n shift-and-xor steps of the normal (MSB-first) algorithm.
template <typename T>
constexpr T crc_step_normal(T v, T poly, T topbit, unsigned n) {
  return n == 0 ? v
                : crc_step_normal<T>(
                      (T)((v & topbit) ? (T)((T)(v << 1) ^ poly) : (T)(v << 1)), poly, topbit,
                      n - 1);
}

template <bool B> struct CrcTag {};

}  // namespace detail

// T       : uint8_t, uint16_t or uint32_t (sets the CRC width)
// Poly    : polynomial in normal (non-reflected) form, e.g. 0x1021
// Init    : initial register value (catalogue form)
// Reflect : true for LSB-first (reflected) CRCs; input and output reflection
//           are both taken to be this value
// XorOut  : value xored into the final result
// Table   : use the 16-entry nibble table instead of the bitwise loop
template <typename T, T Poly, T Init, bool Reflect, T XorOut, bool Table = false>
class Crc {
  static_assert((T)~(T)0 > (T)0, "T must be an unsigned integer type");
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                "CRC width must be 8, 16 or 32 bits");

 public:
  typedef T value_type;
  // Same CRC, nibble-table algorithm.
  typedef Crc<T, Poly, Init, Reflect, XorOut, true> Fast;

  Crc() : state_(initial()) {}

  void reset() { state_ = initial(); }

  void update(uint8_t byte) { state_ = step(state_, byte, detail::CrcTag<Table>()); }

  void update(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    while (len--) update(*p++);
  }

  T value() const { return (T)(state_ ^ XorOut); }

  // Continue from a value previously returned by value() (for example a CRC
  // received at the end of a packet or stored in flash).
  void resume(T previous_value) { state_ = (T)(previous_value ^ XorOut); }

  static T compute(const void* data, size_t len) {
    Crc c;
    c.update(data, len);
    return c.value();
  }

 private:
  static constexpr unsigned kBits = (unsigned)(sizeof(T) * 8);

  static constexpr T rpoly() { return detail::crc_reflect<T>(Poly, kBits); }
  static constexpr T topbit() { return (T)((T)1 << (kBits - 1)); }
  static constexpr T initial() { return Reflect ? detail::crc_reflect<T>(Init, kBits) : Init; }

  // ---- bitwise ----
  static T step(T s, uint8_t byte, detail::CrcTag<false>) {
    if (Reflect) {
      s = (T)(s ^ byte);
      for (int i = 0; i < 8; ++i) s = (T)((s & 1) ? (T)((T)(s >> 1) ^ rpoly()) : (T)(s >> 1));
    } else {
      s = (T)(s ^ (T)((T)byte << (kBits - 8)));
      for (int i = 0; i < 8; ++i) s = (T)((s & topbit()) ? (T)((T)(s << 1) ^ Poly) : (T)(s << 1));
    }
    return s;
  }

  // ---- nibble table ----
  static constexpr T entry(unsigned i) {
    return Reflect ? detail::crc_step_reflected<T>((T)i, rpoly(), 4)
                   : detail::crc_step_normal<T>((T)((T)i << (kBits - 4)), Poly, topbit(), 4);
  }

  // entry() is constexpr, so this table is constant-initialized: it needs no
  // runtime setup and no thread-safe-static guard, and can live in flash.
  static const T* table() {
    static const T t[16] = {
        entry(0), entry(1), entry(2),  entry(3),  entry(4),  entry(5),  entry(6),  entry(7),
        entry(8), entry(9), entry(10), entry(11), entry(12), entry(13), entry(14), entry(15)};
    return t;
  }

  static T step(T s, uint8_t byte, detail::CrcTag<true>) {
    const T* t = table();
    if (Reflect) {
      s = (T)(s ^ byte);
      s = (T)(t[s & 0xF] ^ (T)(s >> 4));
      s = (T)(t[s & 0xF] ^ (T)(s >> 4));
    } else {
      s = (T)(s ^ (T)((T)byte << (kBits - 8)));
      s = (T)((T)(s << 4) ^ t[(T)(s >> (kBits - 4))]);
      s = (T)((T)(s << 4) ^ t[(T)(s >> (kBits - 4))]);
    }
    return s;
  }

  T state_;
};

// ---------------------------------------------------------------------------
// Presets.   name                        poly        init        refl  xorout     check
// ---------------------------------------------------------------------------

// CRC-8
typedef Crc<uint8_t, 0x07, 0x00, false, 0x00> Crc8;          // CRC-8/SMBUS            0xF4
typedef Crc<uint8_t, 0x31, 0x00, true, 0x00> Crc8Maxim;      // CRC-8/MAXIM-DOW (1-Wire) 0xA1
typedef Crc<uint8_t, 0x1D, 0xFF, false, 0xFF> Crc8SaeJ1850;  // CRC-8/SAE-J1850        0x4B

// CRC-16
typedef Crc<uint16_t, 0x1021, 0x0000, false, 0x0000> Crc16Xmodem;       // 0x31C3
typedef Crc<uint16_t, 0x1021, 0xFFFF, false, 0x0000> Crc16CcittFalse;   // 0x29B1 (IBM-3740)
typedef Crc<uint16_t, 0x1021, 0x0000, true, 0x0000> Crc16Kermit;        // 0x2189
typedef Crc<uint16_t, 0x1021, 0xFFFF, true, 0xFFFF> Crc16X25;           // 0x906E (HDLC)
typedef Crc<uint16_t, 0x8005, 0x0000, true, 0x0000> Crc16Arc;           // 0xBB3D (IBM)
typedef Crc<uint16_t, 0x8005, 0xFFFF, true, 0x0000> Crc16Modbus;        // 0x4B37
typedef Crc<uint16_t, 0x8005, 0xFFFF, true, 0xFFFF> Crc16Usb;           // 0xB4C8

// CRC-32
typedef Crc<uint32_t, 0x04C11DB7UL, 0xFFFFFFFFUL, true, 0xFFFFFFFFUL> Crc32;       // 0xCBF43926 (zip, png, ethernet)
typedef Crc<uint32_t, 0x1EDC6F41UL, 0xFFFFFFFFUL, true, 0xFFFFFFFFUL> Crc32c;      // 0xE3069283 (Castagnoli)
typedef Crc<uint32_t, 0x04C11DB7UL, 0xFFFFFFFFUL, false, 0x00000000UL> Crc32Mpeg2; // 0x0376E6E7
typedef Crc<uint32_t, 0x04C11DB7UL, 0xFFFFFFFFUL, false, 0xFFFFFFFFUL> Crc32Bzip2; // 0xFC891918

}  // namespace mcu

#endif  // MCU_CRC_HPP
