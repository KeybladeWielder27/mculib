// xbee_api.hpp - XBee API-mode (AP=1 and AP=2) frames, microcontroller agnostic.
//
// Three layers, use as much as you need:
//
//   1. Frame codec: wrap frame data in 0x7E / length / checksum (with the
//      AP=2 escaping), and a parser that finds frames in a byte stream and
//      recovers from noise.
//   2. Typed frames: build and parse the common frames (local and remote AT
//      commands, transmit request, receive packet, transmit status, modem
//      status) without hand-counting offsets.
//   3. XBeeApi: a small class that ties both to an mcu::Stream.
//
//   mcu::XBeeApi<> api(uart, mcu::XBeeApiMode::kApi2);
//
//   uint8_t id = api.send_at("ID");                       // query: reply is a 0x88 frame
//   api.send_tx(0x0013A20041ABCDEFull, payload, n);        // radio-to-radio data
//
//   if (api.poll()) {                                      // call often; never blocks
//     switch (api.frame_type()) {
//       case mcu::xbee_frame::kRxPacket: {
//         mcu::XBeeRxPacket rx;
//         if (mcu::xbee_parse_rx(api.frame(), api.frame_length(), rx)) use(rx.data, rx.data_len);
//         break; }
//       case mcu::xbee_frame::kAtResponse: ...
//     }
//   }
//
// The frame layouts follow Digi's XBee3 documentation. The frame format
// (delimiter, length, checksum, escaping) is the same for every XBee; the
// typed frames below are the ones shared by XBee3 DigiMesh and 802.15.4.
// Also covered: the explicit addressing frames 0x11/0x91 (Zigbee endpoints,
// cluster and profile) and the 802.15.4 frames 0x00/0x01/0x80/0x81. Anything
// else still passes through the codec; build the frame data yourself.
//
// Header-only, C++11, no heap.

#ifndef MCU_XBEE_API_HPP
#define MCU_XBEE_API_HPP

#include <stddef.h>
#include <stdint.h>

#include "hal/stream.hpp"

namespace mcu {

enum class XBeeApiMode : uint8_t {
  kApi1 = 1,  // AP=1: plain frames
  kApi2 = 2,  // AP=2: 0x7E, 0x7D, 0x11 and 0x13 inside a frame are escaped
};

namespace xbee_frame {
constexpr uint8_t kStart = 0x7E;
constexpr uint8_t kEscape = 0x7D;
constexpr uint8_t kXon = 0x11;
constexpr uint8_t kXoff = 0x13;

// Frame types (first byte of the frame data).
constexpr uint8_t kAtCommand = 0x08;
constexpr uint8_t kAtCommandQueued = 0x09;
constexpr uint8_t kTx64 = 0x00;             // 802.15.4 (deprecated by Digi in favour of 0x10)
constexpr uint8_t kTx16 = 0x01;             // 802.15.4 (deprecated)
constexpr uint8_t kTxRequest = 0x10;
constexpr uint8_t kExplicitTx = 0x11;
constexpr uint8_t kRemoteAtRequest = 0x17;
constexpr uint8_t kAtResponse = 0x88;
constexpr uint8_t kTxStatus = 0x89;         // 802.15.4 / DigiMesh legacy
constexpr uint8_t kModemStatus = 0x8A;
constexpr uint8_t kTxStatusExtended = 0x8B;
constexpr uint8_t kRx64 = 0x80;             // 802.15.4
constexpr uint8_t kRx16 = 0x81;             // 802.15.4
constexpr uint8_t kRxPacket = 0x90;
constexpr uint8_t kExplicitRx = 0x91;
constexpr uint8_t kRemoteAtResponse = 0x97;

// AT command status (0x88 and 0x97).
constexpr uint8_t kAtOk = 0;
constexpr uint8_t kAtError = 1;
constexpr uint8_t kAtInvalidCommand = 2;
constexpr uint8_t kAtInvalidParameter = 3;
constexpr uint8_t kAtTxFailure = 4;

// Remote AT command options (0x17).
constexpr uint8_t kRemoteDisableAck = 0x01;
constexpr uint8_t kRemoteApplyChanges = 0x02;

// Transmit options (0x10).
constexpr uint8_t kTxDisableAck = 0x01;
constexpr uint8_t kTxDisableRouteDiscovery = 0x02;

// Transmit options for the 802.15.4 frames 0x00 / 0x01.
constexpr uint8_t kTx802DisableAck = 0x01;
constexpr uint8_t kTx802BroadcastPan = 0x04;

// Serial-data defaults for explicit frames (0x11 / 0x91) between XBees.
constexpr uint8_t kSerialEndpoint = 0xE8;
constexpr uint16_t kSerialCluster = 0x0011;
constexpr uint16_t kSerialProfile = 0xC105;
}  // namespace xbee_frame

// ---------------------------------------------------------------------------
// Layer 1a: encoding
// ---------------------------------------------------------------------------

// 0xFF minus the 8-bit sum of the frame data.
inline uint8_t xbee_checksum(const uint8_t* data, size_t n) {
  uint8_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum = (uint8_t)(sum + data[i]);
  return (uint8_t)(0xFFu - sum);
}

namespace detail {
inline bool xbee_needs_escape(uint8_t b) {
  return b == xbee_frame::kStart || b == xbee_frame::kEscape || b == xbee_frame::kXon ||
         b == xbee_frame::kXoff;
}
}  // namespace detail

// Longest possible encoding of `data_len` bytes of frame data.
constexpr size_t xbee_max_encoded_size(XBeeApiMode mode, size_t data_len) {
  return mode == XBeeApiMode::kApi2 ? 1 + 2 * (data_len + 3) : data_len + 4;
}

// Exact size of the encoded frame, or 0 if `len` is 0 or over 65535.
inline size_t xbee_encoded_size(XBeeApiMode mode, const uint8_t* data, size_t len) {
  if (len == 0 || len > 0xFFFF) return 0;
  size_t n = len + 4;
  if (mode == XBeeApiMode::kApi2) {
    if (detail::xbee_needs_escape((uint8_t)(len >> 8))) ++n;
    if (detail::xbee_needs_escape((uint8_t)(len & 0xFF))) ++n;
    for (size_t i = 0; i < len; ++i)
      if (detail::xbee_needs_escape(data[i])) ++n;
    if (detail::xbee_needs_escape(xbee_checksum(data, len))) ++n;
  }
  return n;
}

// Encodes one frame into `out`. Returns the bytes written, or 0 if the data is
// empty or over 65535 bytes, or `cap` is too small.
inline size_t xbee_encode(XBeeApiMode mode, const uint8_t* data, size_t len, uint8_t* out,
                          size_t cap) {
  const size_t need = xbee_encoded_size(mode, data, len);
  if (need == 0 || need > cap) return 0;
  size_t o = 0;
  out[o++] = xbee_frame::kStart;
  const bool esc = mode == XBeeApiMode::kApi2;
  const uint8_t hdr[2] = {(uint8_t)(len >> 8), (uint8_t)(len & 0xFF)};
  const uint8_t cks = xbee_checksum(data, len);
  for (size_t i = 0; i < 2 + len + 1; ++i) {
    const uint8_t b = i < 2 ? hdr[i] : (i < 2 + len ? data[i - 2] : cks);
    if (esc && detail::xbee_needs_escape(b)) {
      out[o++] = xbee_frame::kEscape;
      out[o++] = (uint8_t)(b ^ 0x20u);
    } else {
      out[o++] = b;
    }
  }
  return o;
}

enum class XBeeSendResult : uint8_t {
  kSent,
  kNoSpace,  // the stream cannot take the whole frame right now; nothing was written
  kTooLong,  // empty, or over 65535 bytes
  kPartial,  // the stream took only part of it although it reported room; the frame is broken
};

// Writes one frame to `stream` all or nothing, without a buffer for the whole
// frame: it checks write_space() first, then streams the bytes out in small
// pieces. Retry on kNoSpace.
inline XBeeSendResult xbee_send(Stream& stream, XBeeApiMode mode, const uint8_t* data,
                                size_t len) {
  const size_t total = xbee_encoded_size(mode, data, len);
  if (total == 0) return XBeeSendResult::kTooLong;
  const size_t space = stream.write_space();
  if (space != kUnboundedSpace && space < total) return XBeeSendResult::kNoSpace;

  uint8_t chunk[32];
  size_t n = 0;
  bool ok = true;
  const bool esc = mode == XBeeApiMode::kApi2;
  const uint8_t hdr[2] = {(uint8_t)(len >> 8), (uint8_t)(len & 0xFF)};
  const uint8_t cks = xbee_checksum(data, len);

  struct Flush {
    static bool out(Stream& s, const uint8_t* p, size_t k) { return s.write(p, k) == k; }
  };
  chunk[n++] = xbee_frame::kStart;
  for (size_t i = 0; i < 2 + len + 1 && ok; ++i) {
    const uint8_t b = i < 2 ? hdr[i] : (i < 2 + len ? data[i - 2] : cks);
    if (n + 2 > sizeof chunk) {
      ok = Flush::out(stream, chunk, n);
      n = 0;
    }
    if (esc && detail::xbee_needs_escape(b)) {
      chunk[n++] = xbee_frame::kEscape;
      chunk[n++] = (uint8_t)(b ^ 0x20u);
    } else {
      chunk[n++] = b;
    }
  }
  if (ok && n != 0) ok = Flush::out(stream, chunk, n);
  return ok ? XBeeSendResult::kSent : XBeeSendResult::kPartial;
}

// ---------------------------------------------------------------------------
// Layer 1b: parsing
// ---------------------------------------------------------------------------
struct XBeeParserStats {
  uint32_t frames;        // good frames delivered
  uint32_t bad_checksum;  // frames dropped for a wrong checksum
  uint32_t bad_length;    // frames dropped for a length of 0 or over MaxFrame
  uint32_t resyncs;       // frames abandoned because a new start delimiter showed up
  uint32_t skipped;       // bytes ignored while looking for a start delimiter (AP=1)
};

// Finds frames in a byte stream. MaxFrame is the longest frame data (type byte
// included) it will accept; longer frames are dropped. The XBee3's largest
// frames are a little over 256 bytes, so 128 is plenty unless you move big
// payloads (see the radio's NP setting).
//
// In AP=2 an unescaped 0x7E always starts a new frame, so a truncated frame
// costs nothing. In AP=1 a 0x7E can legitimately be data, so after a bad frame
// the parser looks inside the bytes it just read for a real start delimiter
// and carries on from there; a stray 0x7E in noise therefore cannot swallow
// the real frame that follows it.
template <size_t MaxFrame = 128>
class XBeeApiParser {
  static_assert(MaxFrame >= 1 && MaxFrame <= 0xFFFF, "MaxFrame must be 1..65535");

 public:
  explicit XBeeApiParser(XBeeApiMode mode = XBeeApiMode::kApi2) : mode_(mode) { reset(); }

  void set_mode(XBeeApiMode mode) {
    mode_ = mode;
    reset();
  }
  XBeeApiMode mode() const { return mode_; }

  // Reads from `s` until a frame is complete or no more bytes are waiting.
  // Returns true when a frame is ready; it stays valid until the next call to
  // poll() or feed().
  bool poll(Stream& s) {
    ready_ = false;
    drain_replay();
    if (ready_) return true;
    uint8_t b;
    while (s.read_byte(b)) {
      step(b);
      drain_replay();
      if (ready_) return true;
    }
    return false;
  }

  // Same, from a buffer. Returns how many bytes it used; if frame_ready() is
  // true afterwards, handle the frame and call again with the rest.
  size_t feed(const uint8_t* p, size_t n) {
    ready_ = false;
    drain_replay();
    if (ready_) return 0;
    size_t i = 0;
    while (i < n) {
      step(p[i++]);
      drain_replay();
      if (ready_) break;
    }
    return i;
  }

  bool frame_ready() const { return ready_; }
  const uint8_t* frame() const { return raw_ + 2; }  // frame data: type byte first
  size_t frame_length() const { return ready_ ? len_ : 0; }
  uint8_t frame_type() const { return ready_ ? raw_[2] : 0; }

  const XBeeParserStats& stats() const { return stats_; }

  void reset() {
    state_ = kIdle;
    esc_ = false;
    ready_ = false;
    w_ = rbeg_ = rend_ = 0;
    len_ = got_ = 0;
    stats_ = XBeeParserStats{0, 0, 0, 0, 0};
  }

 private:
  enum State : uint8_t { kIdle, kLenHi, kLenLo, kData, kChecksum };

  void begin_frame() {
    state_ = kLenHi;
    esc_ = false;
    w_ = 0;
  }

  // Consumes bytes saved by a rescan (AP=1) until a frame is ready or none are left.
  void drain_replay() {
    while (!ready_ && rbeg_ < rend_) step(raw_[rbeg_++]);
  }

  void step(uint8_t b) {
    if (mode_ == XBeeApiMode::kApi2) {
      if (b == xbee_frame::kStart) {  // unescaped: always a new frame
        if (state_ != kIdle) ++stats_.resyncs;
        begin_frame();
        return;
      }
      if (state_ == kIdle) return;
      if (esc_) {
        b = (uint8_t)(b ^ 0x20u);
        esc_ = false;
      } else if (b == xbee_frame::kEscape) {
        esc_ = true;
        return;
      }
    } else if (state_ == kIdle) {
      if (b == xbee_frame::kStart) {
        begin_frame();
      } else {
        ++stats_.skipped;
      }
      return;
    }

    switch (state_) {
      case kLenHi:
        raw_[w_++] = b;
        len_ = (size_t)b << 8;
        state_ = kLenLo;
        break;
      case kLenLo:
        raw_[w_++] = b;
        len_ |= b;
        if (len_ == 0 || len_ > MaxFrame) {
          ++stats_.bad_length;
          fail();
        } else {
          got_ = 0;
          state_ = kData;
        }
        break;
      case kData:
        raw_[w_++] = b;
        if (++got_ == len_) state_ = kChecksum;
        break;
      case kChecksum:
        raw_[w_++] = b;
        if (xbee_checksum(raw_ + 2, len_) == b) {
          ++stats_.frames;
          ready_ = true;
          state_ = kIdle;
        } else {
          ++stats_.bad_checksum;
          fail();
        }
        break;
      default: break;
    }
  }

  // A frame went wrong. AP=2: wait for the next 0x7E. AP=1: look for a 0x7E
  // among the bytes just read (they may hold the start of a real frame) and
  // re-parse from there.
  void fail() {
    state_ = kIdle;
    if (mode_ == XBeeApiMode::kApi2) return;
    const size_t rem = rend_ - rbeg_;
    for (size_t k = 0; k < rem; ++k) raw_[w_ + k] = raw_[rbeg_ + k];  // w_ <= rbeg_: safe
    const size_t n = w_ + rem;
    size_t p = 0;
    while (p < n && raw_[p] != xbee_frame::kStart) ++p;
    if (p >= n) {
      w_ = rbeg_ = rend_ = 0;
      return;
    }
    ++stats_.resyncs;
    rbeg_ = p + 1;
    rend_ = n;
    begin_frame();
  }

  XBeeApiMode mode_;
  State state_;
  bool esc_;
  bool ready_;
  size_t w_;     // bytes stored in raw_ for the current frame (length, data, checksum)
  size_t rbeg_;  // pending replay bytes live at raw_[rbeg_, rend_)
  size_t rend_;
  size_t len_;
  size_t got_;
  uint8_t raw_[MaxFrame + 3];
  XBeeParserStats stats_;
};

// ---------------------------------------------------------------------------
// Layer 2: typed frames
// ---------------------------------------------------------------------------
namespace detail {
inline void put_be16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}
inline void put_be64(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; ++i) p[i] = (uint8_t)(v >> (56 - 8 * i));
}
inline uint16_t get_be16(const uint8_t* p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }
inline uint64_t get_be64(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
  return v;
}
}  // namespace detail

// ---- builders: write frame data (no delimiter, length or checksum) ----
// All return the frame data length, or 0 if it does not fit in `cap`.

// 0x08 AT command (0x09 if `queued`: applied only on ATAC). `cmd` is the two
// letters ("ID"); with no parameter the frame queries the value. Parameters
// are raw bytes: ATID 0x7FFF is {0x7F, 0xFF}. Frame id 0 asks for no response.
inline size_t xbee_build_at(uint8_t* out, size_t cap, uint8_t frame_id, const char* cmd,
                            const uint8_t* param = 0, size_t param_len = 0,
                            bool queued = false) {
  const size_t n = 4 + param_len;
  if (cmd == 0 || cmd[0] == 0 || cmd[1] == 0 || (param_len != 0 && param == 0) || n > cap)
    return 0;
  out[0] = queued ? xbee_frame::kAtCommandQueued : xbee_frame::kAtCommand;
  out[1] = frame_id;
  out[2] = (uint8_t)cmd[0];
  out[3] = (uint8_t)cmd[1];
  for (size_t i = 0; i < param_len; ++i) out[4 + i] = param[i];
  return n;
}

// 0x17 remote AT command. `options` e.g. xbee_frame::kRemoteApplyChanges.
inline size_t xbee_build_remote_at(uint8_t* out, size_t cap, uint8_t frame_id, uint64_t dest64,
                                   const char* cmd, const uint8_t* param = 0,
                                   size_t param_len = 0,
                                   uint8_t options = xbee_frame::kRemoteApplyChanges,
                                   uint16_t dest16 = 0xFFFE) {
  const size_t n = 15 + param_len;
  if (cmd == 0 || cmd[0] == 0 || cmd[1] == 0 || (param_len != 0 && param == 0) || n > cap)
    return 0;
  out[0] = xbee_frame::kRemoteAtRequest;
  out[1] = frame_id;
  detail::put_be64(out + 2, dest64);
  detail::put_be16(out + 10, dest16);
  out[12] = options;
  out[13] = (uint8_t)cmd[0];
  out[14] = (uint8_t)cmd[1];
  for (size_t i = 0; i < param_len; ++i) out[15 + i] = param[i];
  return n;
}

// 0x10 transmit request to a 64-bit address (0x000000000000FFFF broadcasts).
// radius 0 uses the radio's NH; options 0 uses its TO. Frame id 0 asks for no
// transmit status.
inline size_t xbee_build_tx(uint8_t* out, size_t cap, uint8_t frame_id, uint64_t dest64,
                            const uint8_t* payload, size_t payload_len, uint8_t options = 0,
                            uint8_t radius = 0, uint16_t reserved = 0xFFFE) {
  const size_t n = 14 + payload_len;
  if ((payload_len != 0 && payload == 0) || n > cap) return 0;
  out[0] = xbee_frame::kTxRequest;
  out[1] = frame_id;
  detail::put_be64(out + 2, dest64);
  detail::put_be16(out + 10, reserved);
  out[12] = radius;
  out[13] = options;
  for (size_t i = 0; i < payload_len; ++i) out[14 + i] = payload[i];
  return n;
}

// 0x11 explicit addressing transmit request (Zigbee, and DigiMesh/802.15.4 when
// you need endpoints, cluster or profile). For plain serial data between XBees
// use xbee_frame::kSerialEndpoint / kSerialCluster / kSerialProfile.
inline size_t xbee_build_explicit_tx(uint8_t* out, size_t cap, uint8_t frame_id, uint64_t dest64,
                                     uint8_t src_endpoint, uint8_t dst_endpoint, uint16_t cluster,
                                     uint16_t profile, const uint8_t* payload, size_t payload_len,
                                     uint8_t options = 0, uint8_t radius = 0,
                                     uint16_t reserved = 0xFFFE) {
  const size_t n = 20 + payload_len;
  if ((payload_len != 0 && payload == 0) || n > cap) return 0;
  out[0] = xbee_frame::kExplicitTx;
  out[1] = frame_id;
  detail::put_be64(out + 2, dest64);
  detail::put_be16(out + 10, reserved);
  out[12] = src_endpoint;
  out[13] = dst_endpoint;
  detail::put_be16(out + 14, cluster);
  detail::put_be16(out + 16, profile);
  out[18] = radius;
  out[19] = options;
  for (size_t i = 0; i < payload_len; ++i) out[20 + i] = payload[i];
  return n;
}

// 0x00 802.15.4 transmit to a 64-bit address (0x000000000000FFFF broadcasts).
inline size_t xbee_build_tx64(uint8_t* out, size_t cap, uint8_t frame_id, uint64_t dest64,
                              const uint8_t* payload, size_t payload_len, uint8_t options = 0) {
  const size_t n = 11 + payload_len;
  if ((payload_len != 0 && payload == 0) || n > cap) return 0;
  out[0] = xbee_frame::kTx64;
  out[1] = frame_id;
  detail::put_be64(out + 2, dest64);
  out[10] = options;
  for (size_t i = 0; i < payload_len; ++i) out[11 + i] = payload[i];
  return n;
}

// 0x01 802.15.4 transmit to a 16-bit address (0xFFFF broadcasts).
inline size_t xbee_build_tx16(uint8_t* out, size_t cap, uint8_t frame_id, uint16_t dest16,
                              const uint8_t* payload, size_t payload_len, uint8_t options = 0) {
  const size_t n = 5 + payload_len;
  if ((payload_len != 0 && payload == 0) || n > cap) return 0;
  out[0] = xbee_frame::kTx16;
  out[1] = frame_id;
  detail::put_be16(out + 2, dest16);
  out[4] = options;
  for (size_t i = 0; i < payload_len; ++i) out[5 + i] = payload[i];
  return n;
}

// ---- parsers: views into the frame data, which must outlive them ----

struct XBeeAtResponse {  // 0x88 (local) answer to an AT command
  uint8_t frame_id;
  char cmd[2];
  uint8_t status;  // xbee_frame::kAtOk ...
  const uint8_t* data;
  size_t data_len;  // the value of a query; 0 after a set
};

struct XBeeRemoteAtResponse {  // 0x97
  uint8_t frame_id;
  uint64_t src64;
  uint16_t src16;
  char cmd[2];
  uint8_t status;
  const uint8_t* data;
  size_t data_len;
};

struct XBeeRxPacket {  // 0x90
  uint64_t src64;
  uint16_t src16;
  uint8_t options;  // 0x01 acknowledged, 0x02 broadcast, top two bits: delivery method
  const uint8_t* data;
  size_t data_len;
};

struct XBeeTxStatus {  // 0x89 or 0x8B
  uint8_t frame_id;
  uint8_t delivery;   // 0 = success
  bool extended;      // true for 0x8B (the fields below are valid)
  uint16_t addr16;
  uint8_t retries;
  uint8_t discovery;  // 0 none, 2 route discovery
};

inline bool xbee_parse_at_response(const uint8_t* f, size_t n, XBeeAtResponse& out) {
  if (f == 0 || n < 5 || f[0] != xbee_frame::kAtResponse) return false;
  out.frame_id = f[1];
  out.cmd[0] = (char)f[2];
  out.cmd[1] = (char)f[3];
  out.status = f[4];
  out.data = f + 5;
  out.data_len = n - 5;
  return true;
}

inline bool xbee_parse_remote_at_response(const uint8_t* f, size_t n, XBeeRemoteAtResponse& out) {
  if (f == 0 || n < 15 || f[0] != xbee_frame::kRemoteAtResponse) return false;
  out.frame_id = f[1];
  out.src64 = detail::get_be64(f + 2);
  out.src16 = detail::get_be16(f + 10);
  out.cmd[0] = (char)f[12];
  out.cmd[1] = (char)f[13];
  out.status = f[14];
  out.data = f + 15;
  out.data_len = n - 15;
  return true;
}

inline bool xbee_parse_rx(const uint8_t* f, size_t n, XBeeRxPacket& out) {
  if (f == 0 || n < 12 || f[0] != xbee_frame::kRxPacket) return false;
  out.src64 = detail::get_be64(f + 1);
  out.src16 = detail::get_be16(f + 9);
  out.options = f[11];
  out.data = f + 12;
  out.data_len = n - 12;
  return true;
}

inline bool xbee_parse_tx_status(const uint8_t* f, size_t n, XBeeTxStatus& out) {
  if (f == 0 || n < 3) return false;
  if (f[0] == xbee_frame::kTxStatus) {
    out.frame_id = f[1];
    out.delivery = f[2];
    out.extended = false;
    out.addr16 = 0;
    out.retries = 0;
    out.discovery = 0;
    return true;
  }
  if (f[0] == xbee_frame::kTxStatusExtended && n >= 7) {
    out.frame_id = f[1];
    out.addr16 = detail::get_be16(f + 2);
    out.retries = f[4];
    out.delivery = f[5];
    out.discovery = f[6];
    out.extended = true;
    return true;
  }
  return false;
}

struct XBeeExplicitRx {  // 0x91
  uint64_t src64;
  uint16_t src16;
  uint8_t src_endpoint;
  uint8_t dst_endpoint;
  uint16_t cluster;
  uint16_t profile;
  uint8_t options;
  const uint8_t* data;
  size_t data_len;
};

struct XBeeRx802 {  // 0x80 (64-bit source) or 0x81 (16-bit source)
  bool addr64_valid;  // true for 0x80: src64 is valid; false for 0x81: src16 is valid
  uint64_t src64;
  uint16_t src16;
  uint8_t rssi;     // magnitude of the dBm reading: 0x28 means -40 dBm
  uint8_t options;  // 0x02 broadcast, 0x04 broadcast across all PANs
  const uint8_t* data;
  size_t data_len;
};

inline bool xbee_parse_explicit_rx(const uint8_t* f, size_t n, XBeeExplicitRx& out) {
  if (f == 0 || n < 18 || f[0] != xbee_frame::kExplicitRx) return false;
  out.src64 = detail::get_be64(f + 1);
  out.src16 = detail::get_be16(f + 9);
  out.src_endpoint = f[11];
  out.dst_endpoint = f[12];
  out.cluster = detail::get_be16(f + 13);
  out.profile = detail::get_be16(f + 15);
  out.options = f[17];
  out.data = f + 18;
  out.data_len = n - 18;
  return true;
}

inline bool xbee_parse_rx802(const uint8_t* f, size_t n, XBeeRx802& out) {
  if (f == 0 || n < 1) return false;
  if (f[0] == xbee_frame::kRx64 && n >= 11) {
    out.addr64_valid = true;
    out.src64 = detail::get_be64(f + 1);
    out.src16 = 0;
    out.rssi = f[9];
    out.options = f[10];
    out.data = f + 11;
    out.data_len = n - 11;
    return true;
  }
  if (f[0] == xbee_frame::kRx16 && n >= 5) {
    out.addr64_valid = false;
    out.src64 = 0;
    out.src16 = detail::get_be16(f + 1);
    out.rssi = f[3];
    out.options = f[4];
    out.data = f + 5;
    out.data_len = n - 5;
    return true;
  }
  return false;
}

// 0x8A modem status: 0 power-up, 1 watchdog reset, 2 joined, 3 left, ...
inline bool xbee_parse_modem_status(const uint8_t* f, size_t n, uint8_t& status) {
  if (f == 0 || n < 2 || f[0] != xbee_frame::kModemStatus) return false;
  status = f[1];
  return true;
}

// ---------------------------------------------------------------------------
// Layer 3: XBeeApi, the codec bound to a Stream
// ---------------------------------------------------------------------------
// MaxFrame: longest received frame data. MaxOut: longest frame data to send.
template <size_t MaxFrame = 128, size_t MaxOut = 96>
class XBeeApi {
 public:
  XBeeApi(Stream& stream, XBeeApiMode mode = XBeeApiMode::kApi2)
      : s_(stream), mode_(mode), parser_(mode), next_id_(1), last_result_(XBeeSendResult::kSent) {}

  void set_mode(XBeeApiMode mode) {
    mode_ = mode;
    parser_.set_mode(mode);
  }

  // ---- receiving ----

  // True when a frame is ready (valid until the next poll()).
  bool poll() { return parser_.poll(s_); }
  const uint8_t* frame() const { return parser_.frame(); }
  size_t frame_length() const { return parser_.frame_length(); }
  uint8_t frame_type() const { return parser_.frame_type(); }
  const XBeeParserStats& stats() const { return parser_.stats(); }

  // ---- sending (each returns the frame id used, or 0 if nothing was sent) ----

  // 0x08 AT command. See xbee_build_at() for the parameter format.
  uint8_t send_at(const char* cmd, const uint8_t* param = 0, size_t param_len = 0,
                  bool queued = false) {
    const uint8_t id = take_id();
    uint8_t buf[MaxOut];
    const size_t n = xbee_build_at(buf, sizeof buf, id, cmd, param, param_len, queued);
    return finish(buf, n, id);
  }

  // 0x17 remote AT command.
  uint8_t send_remote_at(uint64_t dest64, const char* cmd, const uint8_t* param = 0,
                         size_t param_len = 0,
                         uint8_t options = xbee_frame::kRemoteApplyChanges) {
    const uint8_t id = take_id();
    uint8_t buf[MaxOut];
    const size_t n =
        xbee_build_remote_at(buf, sizeof buf, id, dest64, cmd, param, param_len, options);
    return finish(buf, n, id);
  }

  // 0x10 transmit request. Pass `want_status` = false to suppress the transmit
  // status frame (frame id 0); the return value is then 0 on failure and 1 on success.
  uint8_t send_tx(uint64_t dest64, const uint8_t* payload, size_t len, uint8_t options = 0,
                  uint8_t radius = 0, bool want_status = true) {
    const uint8_t id = want_status ? take_id() : 0;
    uint8_t buf[MaxOut];
    const size_t n = xbee_build_tx(buf, sizeof buf, id, dest64, payload, len, options, radius);
    const uint8_t r = finish(buf, n, id);
    return (r == 0 && !want_status && last_result_ == XBeeSendResult::kSent) ? 1 : r;
  }

  // 0x11 explicit addressing transmit (see xbee_build_explicit_tx).
  uint8_t send_explicit(uint64_t dest64, uint8_t src_endpoint, uint8_t dst_endpoint,
                        uint16_t cluster, uint16_t profile, const uint8_t* payload, size_t len,
                        uint8_t options = 0, uint8_t radius = 0) {
    const uint8_t id = take_id();
    uint8_t buf[MaxOut];
    const size_t n = xbee_build_explicit_tx(buf, sizeof buf, id, dest64, src_endpoint, dst_endpoint,
                                            cluster, profile, payload, len, options, radius);
    return finish(buf, n, id);
  }

  // 802.15.4 transmit to a 64-bit (0x00) or 16-bit (0x01) address.
  uint8_t send_tx64(uint64_t dest64, const uint8_t* payload, size_t len, uint8_t options = 0) {
    const uint8_t id = take_id();
    uint8_t buf[MaxOut];
    const size_t n = xbee_build_tx64(buf, sizeof buf, id, dest64, payload, len, options);
    return finish(buf, n, id);
  }
  uint8_t send_tx16(uint16_t dest16, const uint8_t* payload, size_t len, uint8_t options = 0) {
    const uint8_t id = take_id();
    uint8_t buf[MaxOut];
    const size_t n = xbee_build_tx16(buf, sizeof buf, id, dest16, payload, len, options);
    return finish(buf, n, id);
  }

  // Sends any prepared frame data. True if the whole frame went out.
  bool send_frame(const uint8_t* data, size_t len) {
    last_result_ = xbee_send(s_, mode_, data, len);
    return last_result_ == XBeeSendResult::kSent;
  }

  // Why the last send failed (kNoSpace means try again later).
  XBeeSendResult last_send_result() const { return last_result_; }

 private:
  // Frame ids cycle 1..255; 0 is reserved for "no response wanted".
  uint8_t take_id() {
    const uint8_t id = next_id_;
    next_id_ = (uint8_t)(next_id_ == 255 ? 1 : next_id_ + 1);
    return id;
  }

  uint8_t finish(const uint8_t* buf, size_t n, uint8_t id) {
    if (n == 0) {
      last_result_ = XBeeSendResult::kTooLong;
      return 0;
    }
    return send_frame(buf, n) ? id : 0;
  }

  Stream& s_;
  XBeeApiMode mode_;
  XBeeApiParser<MaxFrame> parser_;
  uint8_t next_id_;
  XBeeSendResult last_result_;
};

}  // namespace mcu

#endif  // MCU_XBEE_API_HPP
