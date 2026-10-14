// output.hpp - text output helpers, microcontroller agnostic.
//
// Output is a tiny abstract sink with a handful of print helpers on top, so
// command handlers can print text and numbers without printf (which is large
// on small MCUs) and without caring where the bytes go.
//
//   out.print("temp = ");
//   out.print_fixed(2315, 2);        // 23.15
//   out.println(" C");
//
// write() is best effort and never blocks: it returns how many bytes were
// accepted, and the helpers ignore a short write. The console's output queue
// counts what it has to drop, see console.hpp.
//
// Line ends are "\r\n", which is what serial terminals expect.
//
// Header-only, C++11, no heap, no <string.h>, no printf.

#ifndef MCU_OUTPUT_HPP
#define MCU_OUTPUT_HPP

#include <stddef.h>
#include <stdint.h>

#include "hal/stream.hpp"

namespace mcu {

class Output {
 public:
  // Accepts up to `len` bytes, returns how many were taken.
  virtual size_t write(const uint8_t* src, size_t len) = 0;

  // ---- helpers (not virtual) ----

  void put(char c) {
    const uint8_t b = (uint8_t)c;
    write(&b, 1);
  }

  void print(const char* s) {
    if (s == nullptr) return;
    size_t n = 0;
    while (s[n] != '\0') ++n;
    if (n != 0) write(reinterpret_cast<const uint8_t*>(s), n);
  }

  void println() { print("\r\n"); }
  void println(const char* s) {
    print(s);
    println();
  }

  // Unsigned number in base 2..16 (anything else prints base 10), upper-case
  // digits, zero padded on the left to at least `min_digits`.
  void print_uint(uint32_t v, unsigned base = 10, unsigned min_digits = 1) {
    char tmp[32];
    unsigned n = 0;
    if (base < 2 || base > 16) base = 10;
    do {
      const unsigned d = (unsigned)(v % base);
      tmp[n++] = (char)(d < 10 ? ('0' + d) : ('A' + (d - 10)));
      v /= base;
    } while (v != 0 && n < sizeof tmp);
    while (n < min_digits && n < sizeof tmp) tmp[n++] = '0';
    while (n != 0) put(tmp[--n]);
  }

  void print_int(int32_t v) {
    if (v < 0) {
      put('-');
      print_uint((uint32_t)(-(v + 1)) + 1u);
    } else {
      print_uint((uint32_t)v);
    }
  }

  // Fixed-width upper-case hex without a prefix: print_hex(0xAB, 4) -> "00AB".
  void print_hex(uint32_t v, unsigned digits = 2) { print_uint(v, 16, digits); }

  // A value held as an integer count of 10^-decimals units, shown as a
  // decimal: print_fixed(-5, 2) -> "-0.05", print_fixed(31415, 3) -> "31.415".
  // This is the output counterpart of parse_scaled(), and needs no floating point.
  void print_fixed(int32_t scaled, unsigned decimals) {
    if (decimals > 9) decimals = 9;
    uint32_t mag = scaled < 0 ? (uint32_t)(-(scaled + 1)) + 1u : (uint32_t)scaled;
    if (scaled < 0) put('-');
    uint32_t div = 1;
    for (unsigned i = 0; i < decimals; ++i) div *= 10;
    print_uint(mag / div);
    if (decimals != 0) {
      put('.');
      print_uint(mag % div, 10, decimals);
    }
  }

  // Bytes as upper-case hex pairs with no separator: {0xDE,0xAD} -> "DEAD".
  void print_bytes_hex(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; ++i) print_hex(p[i], 2);
  }

 protected:
  Output() {}
  ~Output() {}
};

// Sends straight to a Stream, best effort: whatever the stream cannot take
// right now is lost. For simple cases; the console uses a queue instead.
class StreamOutput final : public Output {
 public:
  explicit StreamOutput(Stream& s) : s_(s) {}
  size_t write(const uint8_t* src, size_t len) override { return s_.write(src, len); }

 private:
  Stream& s_;
};

// Collects output into a fixed buffer as a NUL-terminated string. Handy for
// building a line before sending it, and for tests.
template <size_t N>
class BufferOutput final : public Output {
  static_assert(N >= 2, "buffer must hold at least one character");

 public:
  BufferOutput() : n_(0), over_(false) { buf_[0] = '\0'; }

  size_t write(const uint8_t* src, size_t len) override {
    const size_t room = N - 1 - n_;
    const size_t k = len < room ? len : room;
    for (size_t i = 0; i < k; ++i) buf_[n_++] = (char)src[i];
    buf_[n_] = '\0';
    if (k < len) over_ = true;
    return k;
  }

  const char* c_str() const { return buf_; }
  size_t size() const { return n_; }
  bool overflowed() const { return over_; }
  void clear() {
    n_ = 0;
    over_ = false;
    buf_[0] = '\0';
  }

 private:
  char buf_[N];
  size_t n_;
  bool over_;
};

}  // namespace mcu

#endif  // MCU_OUTPUT_HPP
