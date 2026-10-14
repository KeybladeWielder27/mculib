// codec.hpp - COBS and SLIP byte-stuffing codecs, microcontroller agnostic.
//
// Both codecs present the same small interface, so framer.hpp can use either
// one by swapping a template argument:
//
//   struct Codec {
//     static uint8_t delimiter();                  // frame separator on the wire
//     static bool    allows_empty();               // can a zero-length frame exist?
//     static size_t  max_encoded_size(size_t n);   // worst case for n bytes, incl. final delimiter
//     template <typename Sink> static bool encode(const Segments&, Sink&);
//     class Decoder;                               // byte-at-a-time, see below
//   };
//
// Header-only, C++11, no heap, no exceptions, no RTTI.
//
// COBS (Consistent Overhead Byte Stuffing): frames are delimited by 0x00,
//   which never appears inside a frame. Overhead is 1 byte per 254, worst
//   case. Resynchronises at the next 0x00 whatever garbage came before.
// SLIP (RFC 1055): frames are delimited by 0xC0. Bytes 0xC0 and 0xDB inside
//   the data are escaped, so worst case overhead is 2x, typical overhead is
//   tiny for data without those values. A zero-length frame cannot be
//   represented (it looks like two delimiters in a row, which means "idle").

#ifndef MCU_CODEC_HPP
#define MCU_CODEC_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

// Two byte ranges that are logically one sequence (payload followed by its
// check bytes), so encoding needs no intermediate copy.
struct Segments {
  const uint8_t* a;
  size_t an;
  const uint8_t* b;
  size_t bn;

  size_t size() const { return an + bn; }
  uint8_t at(size_t i) const { return i < an ? a[i] : b[i - an]; }
};

// What a Decoder reports after each wire byte.
enum class DecodeStep : uint8_t {
  kMore,         // frame still in progress
  kEmpty,        // a delimiter with nothing before it (idle line); ignore
  kComplete,     // a whole frame is in the output buffer
  kBadEncoding,  // malformed frame (truncated, bad escape); output is invalid
  kOverflow      // frame did not fit the output buffer; output is invalid
};

namespace detail {
inline void decoder_put(uint8_t v, uint8_t* buf, size_t cap, size_t& len, bool& over) {
  if (over) return;
  if (len >= cap) {
    over = true;
  } else {
    buf[len++] = v;
  }
}
}  // namespace detail

// ---------------------------------------------------------------------------
// COBS
// ---------------------------------------------------------------------------
struct Cobs {
  static constexpr uint8_t delimiter() { return 0x00; }
  static constexpr bool allows_empty() { return true; }
  // n + one code byte per started 254-byte block + final delimiter.
  static constexpr size_t max_encoded_size(size_t n) { return n + n / 254 + 2; }

  // Writes the encoded frame followed by the 0x00 delimiter to sink.put().
  // Returns false as soon as the sink refuses a byte.
  template <typename Sink>
  static bool encode(const Segments& s, Sink& sink) {
    const size_t total = s.size();
    size_t pos = 0;
    for (;;) {
      size_t k = 0;  // run of non-zero bytes starting at pos, at most 254
      while (k < 254 && pos + k < total && s.at(pos + k) != 0) ++k;
      if (!sink.put((uint8_t)(k + 1))) return false;
      for (size_t i = 0; i < k; ++i)
        if (!sink.put(s.at(pos + i))) return false;
      pos += k;
      if (k == 254) {            // full block: no zero is implied after it
        if (pos == total) break;
        continue;
      }
      if (pos == total) break;   // last block
      ++pos;                     // swallow the zero that ended this block
    }
    return sink.put(0x00);
  }

  class Decoder {
   public:
    Decoder() { reset(); }

    void reset() {
      left_ = 0;
      prev_ = 0;
      first_ = true;
      over_ = false;
      any_ = false;
    }

    // Feeds one wire byte. Decoded bytes are appended to buf[len++].
    DecodeStep feed(uint8_t b, uint8_t* buf, size_t cap, size_t& len) {
      if (b == 0x00) {
        DecodeStep r;
        if (!any_) {
          r = DecodeStep::kEmpty;
        } else if (left_ != 0) {
          r = DecodeStep::kBadEncoding;  // frame ended in the middle of a block
        } else if (over_) {
          r = DecodeStep::kOverflow;
        } else {
          r = DecodeStep::kComplete;
        }
        reset();
        return r;
      }
      any_ = true;
      if (left_ == 0) {
        // A code byte. The previous block (if any, and unless it was a full
        // 254-byte one) was followed by a zero in the original data.
        if (!first_ && prev_ != 0xFF) detail::decoder_put(0x00, buf, cap, len, over_);
        first_ = false;
        prev_ = b;
        left_ = (uint8_t)(b - 1);
      } else {
        detail::decoder_put(b, buf, cap, len, over_);
        left_ = (uint8_t)(left_ - 1);
      }
      return DecodeStep::kMore;
    }

   private:
    uint8_t left_;   // data bytes still to come in the current block
    uint8_t prev_;   // code byte of the current block
    bool first_;
    bool over_;
    bool any_;
  };
};

// ---------------------------------------------------------------------------
// SLIP
// ---------------------------------------------------------------------------
struct Slip {
  enum : uint8_t { kEnd = 0xC0, kEsc = 0xDB, kEscEnd = 0xDC, kEscEsc = 0xDD };

  static constexpr uint8_t delimiter() { return 0xC0; }
  static constexpr bool allows_empty() { return false; }
  // Every byte may need two bytes, plus the final delimiter.
  static constexpr size_t max_encoded_size(size_t n) { return 2 * n + 1; }

  template <typename Sink>
  static bool encode(const Segments& s, Sink& sink) {
    const size_t total = s.size();
    for (size_t i = 0; i < total; ++i) {
      const uint8_t v = s.at(i);
      if (v == (uint8_t)kEnd) {
        if (!sink.put((uint8_t)kEsc) || !sink.put((uint8_t)kEscEnd)) return false;
      } else if (v == (uint8_t)kEsc) {
        if (!sink.put((uint8_t)kEsc) || !sink.put((uint8_t)kEscEsc)) return false;
      } else {
        if (!sink.put(v)) return false;
      }
    }
    return sink.put((uint8_t)kEnd);
  }

  class Decoder {
   public:
    Decoder() { reset(); }

    void reset() {
      esc_ = false;
      bad_ = false;
      over_ = false;
      any_ = false;
    }

    DecodeStep feed(uint8_t b, uint8_t* buf, size_t cap, size_t& len) {
      if (b == (uint8_t)kEnd) {
        DecodeStep r;
        if (!any_) {
          r = DecodeStep::kEmpty;
        } else if (bad_ || esc_) {
          r = DecodeStep::kBadEncoding;  // bad escape, or END right after ESC
        } else if (over_) {
          r = DecodeStep::kOverflow;
        } else {
          r = DecodeStep::kComplete;
        }
        reset();
        return r;
      }
      any_ = true;
      if (bad_) return DecodeStep::kMore;  // swallow the rest of a bad frame
      if (esc_) {
        esc_ = false;
        if (b == (uint8_t)kEscEnd) {
          detail::decoder_put((uint8_t)kEnd, buf, cap, len, over_);
        } else if (b == (uint8_t)kEscEsc) {
          detail::decoder_put((uint8_t)kEsc, buf, cap, len, over_);
        } else {
          bad_ = true;  // ESC must be followed by ESC_END or ESC_ESC
        }
      } else if (b == (uint8_t)kEsc) {
        esc_ = true;
      } else {
        detail::decoder_put(b, buf, cap, len, over_);
      }
      return DecodeStep::kMore;
    }

   private:
    bool esc_;
    bool bad_;
    bool over_;
    bool any_;
  };
};

}  // namespace mcu

#endif  // MCU_CODEC_HPP
