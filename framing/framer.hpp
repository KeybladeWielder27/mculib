// framer.hpp - packet framing over a byte stream: COBS or SLIP, with an optional
// CRC / checksum, microcontroller agnostic.
//
//   typedef mcu::FrameEncoder<mcu::Cobs, mcu::Crc16Modbus, 64, true> Enc;
//   typedef mcu::FrameDecoder<mcu::Cobs, mcu::Crc16Modbus, 64, true> Dec;
//
// The template arguments are the same for both sides:
//   Codec       mcu::Cobs or mcu::Slip (codec.hpp). Swap one word to change framing.
//   Check       any checksum class with the common interface: a CRC from
//               crc.hpp, Fletcher16/32 / Xor8 / Sum8 / Lrc8 from checksum.hpp,
//               or mcu::NoCheck for none. Its bytes are appended to the payload
//               before framing.
//   MaxPayload  largest payload, in bytes, excluding the check.
//   LittleEndian  byte order of the check on the wire (false = most significant
//               byte first; Modbus CRC-16 uses true).
//
// Sending (no buffer needed, bytes go straight to the stream):
//   Enc::send(port, payload, len);          // false if it does not fit right now
//
// Receiving:
//   Dec dec;
//   for (;;) {
//     mcu::FrameResult r = dec.poll(port);  // reads until a frame or error, or the port is empty
//     if (r == mcu::FrameResult::kNoFrame) break;
//     if (r == mcu::FrameResult::kFrame) handle(dec.data(), dec.size());
//   }
//
// Header-only, C++11, no heap, no exceptions, no RTTI. Needs hal/stream.hpp.

#ifndef MCU_FRAMER_HPP
#define MCU_FRAMER_HPP

#include <stddef.h>
#include <stdint.h>

#include "framing/codec.hpp"
#include "hal/stream.hpp"

namespace mcu {

// Use as the Check argument for frames with no integrity check.
class NoCheck {
 public:
  typedef uint8_t value_type;
  void reset() {}
  void update(uint8_t) {}
  void update(const void*, size_t) {}
  uint8_t value() const { return 0; }
};

// Number of check bytes a Check type adds to every frame.
template <typename C>
struct CheckSize {
  enum { value = sizeof(typename C::value_type) };
};
template <>
struct CheckSize<NoCheck> {
  enum { value = 0 };
};

enum class FrameResult : uint8_t {
  kNoFrame,      // nothing complete yet (feed or poll more)
  kFrame,        // a valid frame: data() / size() hold the payload
  kBadCheck,     // frame decoded but failed its check (or was too short to hold one)
  kBadEncoding,  // malformed COBS/SLIP data; the frame is dropped
  kOverflow      // frame longer than MaxPayload; dropped
};

struct FrameStats {
  uint32_t ok;
  uint32_t bad_check;
  uint32_t bad_encoding;
  uint32_t overflow;
};

namespace detail {

// Writes `nbytes` low bytes of v to out, most significant first or last.
template <typename V>
inline void store_check(V v, size_t nbytes, bool little_endian, uint8_t* out) {
  const uint32_t w = (uint32_t)v;
  for (size_t i = 0; i < nbytes; ++i) {
    const size_t shift = 8 * (little_endian ? i : (nbytes - 1 - i));
    out[i] = (uint8_t)(w >> shift);
  }
}

// Sinks: where an encoder puts its bytes.
class CountSink {
 public:
  CountSink() : n_(0) {}
  bool put(uint8_t) { ++n_; return true; }
  size_t count() const { return n_; }

 private:
  size_t n_;
};

class BufferSink {
 public:
  BufferSink(uint8_t* out, size_t cap) : out_(out), cap_(cap), n_(0) {}
  bool put(uint8_t b) {
    if (n_ >= cap_) return false;
    out_[n_++] = b;
    return true;
  }
  size_t count() const { return n_; }

 private:
  uint8_t* out_;
  size_t cap_;
  size_t n_;
};

// Batches bytes into a small local buffer before calling Stream::write.
class StreamSink {
 public:
  explicit StreamSink(Stream& s) : s_(s), n_(0), ok_(true) {}
  bool put(uint8_t b) {
    if (!ok_) return false;
    if (n_ == sizeof buf_ && !flush()) return false;
    buf_[n_++] = b;
    return true;
  }
  bool flush() {
    if (n_ != 0) {
      if (s_.write(buf_, n_) != n_) ok_ = false;
      n_ = 0;
    }
    return ok_;
  }

 private:
  Stream& s_;
  uint8_t buf_[32];
  size_t n_;
  bool ok_;
};

}  // namespace detail

// ---------------------------------------------------------------------------
// Encoder (stateless: every function is static)
// ---------------------------------------------------------------------------
template <typename Codec, typename Check, size_t MaxPayload, bool LittleEndian = false>
class FrameEncoder {
 public:
  static constexpr size_t kCheckBytes = (size_t)CheckSize<Check>::value;

  // Upper bound on the size of any encoded frame, including the optional
  // leading delimiter. Use it to size a buffer for encode().
  static constexpr size_t max_frame_size() {
    return 1 + Codec::max_encoded_size(MaxPayload + (size_t)CheckSize<Check>::value);
  }

  // Exact size of the encoded frame for this payload, or 0 if it cannot be
  // sent (payload longer than MaxPayload, or an empty SLIP frame).
  static size_t frame_size(const uint8_t* payload, size_t len, bool leading_delimiter = false) {
    if (!sendable(len)) return 0;
    detail::CountSink sink;
    emit(payload, len, leading_delimiter, sink);
    return sink.count();
  }

  // Encodes into out[0..cap). Returns the frame size, or 0 if the payload is
  // not sendable or the buffer is too small.
  static size_t encode(const uint8_t* payload, size_t len, uint8_t* out, size_t cap,
                       bool leading_delimiter = false) {
    if (!sendable(len)) return 0;
    detail::BufferSink sink(out, cap);
    return emit(payload, len, leading_delimiter, sink) ? sink.count() : 0;
  }

  // Encodes straight into the stream, using no buffer of its own. Returns
  // false, having written nothing, if the stream cannot take the whole frame
  // right now (check Stream::write_space) or the payload is not sendable.
  //
  // `leading_delimiter` sends a delimiter first. It costs one byte and makes
  // the receiver discard any line noise that arrived before the frame, which
  // matters mainly for SLIP after a cold start or a noisy link.
  static bool send(Stream& s, const uint8_t* payload, size_t len,
                   bool leading_delimiter = false) {
    const size_t need = frame_size(payload, len, leading_delimiter);
    if (need == 0 || s.write_space() < need) return false;
    detail::StreamSink sink(s);
    const bool ok = emit(payload, len, leading_delimiter, sink);
    return sink.flush() && ok;
  }

 private:
  static bool sendable(size_t len) {
    if (len > MaxPayload) return false;
    return len + kCheckBytes != 0 || Codec::allows_empty();
  }

  template <typename Sink>
  static bool emit(const uint8_t* payload, size_t len, bool leading, Sink& sink) {
    uint8_t chk[4] = {0, 0, 0, 0};
    if (kCheckBytes != 0) {
      Check c;
      c.update(payload, len);
      detail::store_check(c.value(), kCheckBytes, LittleEndian, chk);
    }
    if (leading && !sink.put(Codec::delimiter())) return false;
    const Segments seg = {payload, len, chk, kCheckBytes};
    return Codec::encode(seg, sink);
  }
};

// ---------------------------------------------------------------------------
// Decoder. Holds MaxPayload + check bytes of receive buffer.
// ---------------------------------------------------------------------------
template <typename Codec, typename Check, size_t MaxPayload, bool LittleEndian = false>
class FrameDecoder {
  static_assert(MaxPayload >= 1, "MaxPayload must be at least 1");

 public:
  static constexpr size_t kCheckBytes = (size_t)CheckSize<Check>::value;

  FrameDecoder() { reset(); }

  void reset() {
    dec_.reset();
    len_ = 0;
    size_ = 0;
    ready_ = false;
  }

  // Feeds one wire byte. kNoFrame means "keep going". Anything else is an
  // event: kFrame (payload via data()/size(), valid until the next push or
  // poll) or one of the errors, which have already been counted in stats().
  FrameResult push(uint8_t b) {
    if (ready_) {  // the previous frame has been reported; start fresh
      len_ = 0;
      size_ = 0;
      ready_ = false;
    }
    const DecodeStep st = dec_.feed(b, buf_, sizeof buf_, len_);
    switch (st) {
      case DecodeStep::kMore:
      case DecodeStep::kEmpty:
        return FrameResult::kNoFrame;
      case DecodeStep::kBadEncoding:
        ready_ = true;
        ++stats_.bad_encoding;
        return FrameResult::kBadEncoding;
      case DecodeStep::kOverflow:
        ready_ = true;
        ++stats_.overflow;
        return FrameResult::kOverflow;
      case DecodeStep::kComplete:
        break;
    }
    ready_ = true;
    if (len_ < kCheckBytes || !check_ok()) {
      ++stats_.bad_check;
      return FrameResult::kBadCheck;
    }
    size_ = len_ - kCheckBytes;
    ++stats_.ok;
    return FrameResult::kFrame;
  }

  // Reads bytes from the stream one at a time until a frame or an error
  // completes, or the stream runs dry. Returns at the first event, leaving the
  // rest in the stream, so call it in a loop until it returns kNoFrame.
  FrameResult poll(Stream& s) {
    uint8_t b;
    while (s.read_byte(b)) {
      const FrameResult r = push(b);
      if (r != FrameResult::kNoFrame) return r;
    }
    return FrameResult::kNoFrame;
  }

  // The payload of the last kFrame result (check bytes removed).
  const uint8_t* data() const { return buf_; }
  size_t size() const { return size_; }

  const FrameStats& stats() const { return stats_; }
  void clear_stats() {
    stats_.ok = 0;
    stats_.bad_check = 0;
    stats_.bad_encoding = 0;
    stats_.overflow = 0;
  }

 private:
  bool check_ok() const {
    if (kCheckBytes == 0) return true;
    const size_t plen = len_ - kCheckBytes;
    Check c;
    c.update(buf_, plen);
    uint8_t expect[4] = {0, 0, 0, 0};
    detail::store_check(c.value(), kCheckBytes, LittleEndian, expect);
    for (size_t i = 0; i < kCheckBytes; ++i)
      if (buf_[plen + i] != expect[i]) return false;
    return true;
  }

  typename Codec::Decoder dec_;
  uint8_t buf_[MaxPayload + (size_t)CheckSize<Check>::value];
  size_t len_;
  size_t size_;
  bool ready_;
  FrameStats stats_ = {0, 0, 0, 0};
};

}  // namespace mcu

#endif  // MCU_FRAMER_HPP
