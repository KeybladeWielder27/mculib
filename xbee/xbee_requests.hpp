// xbee_requests.hpp - request/response tracking for XBee API mode, microcontroller agnostic.
//
// XBeeApi (xbee_api.hpp) sends and receives frames. This layer adds what a
// program usually wants on top: "send this AT command and tell me what the
// radio answered", "send this data and tell me whether it was delivered", with
// timeouts and retries, several requests in flight at once, and no blocking.
//
//   mcu::XBeeRequests<> xb(uart, mcu::XBeeApiMode::kApi2);
//
//   mcu::XBeeReqHandle h = xb.at("ID", 0, 0, millis());        // query ATID
//   ...
//   mcu::XBeeReqHandle t = xb.tx(0x0013A20041ABCDEFull, data, n, millis());
//
//   void loop() {
//     if (xb.poll(millis())) {                                   // call every pass
//       // a frame that is not an answer to a request: received data, modem status, ...
//       if (xb.frame_type() == mcu::xbee_frame::kRxPacket) { ... xbee_parse_rx(xb.frame(), xb.frame_length(), rx) ... }
//     }
//     const mcu::XBeeReqInfo* r = xb.info(h);                    // 0 if the handle is stale
//     if (r && r->result != mcu::XBeeReqResult::kPending) {
//       uint64_t id; if (r->result == mcu::XBeeReqResult::kOk && mcu::xbee_resp_u64(*r, id)) ...
//       xb.release(h);                                           // frees the slot
//       h = 0;
//     }
//   }
//
// Or give a request a callback and forget the handle (the slot is freed for you):
//
//   mcu::XBeeReqOptions o = mcu::XBeeReqOptions::at();
//   o.on_done = my_callback; o.ctx = &something;      // or o.auto_release = true to just discard the result
//   xb.at("NI", 0, 0, millis(), &o);
//
// How requests are matched
//   * Every request gets a frame id (1..255, not in use by another request).
//     0x88 answers local AT commands, 0x97 remote AT commands (and must come
//     from the radio that was addressed, unless it was a broadcast), 0x89 / 0x8B
//     transmit status. Anything else is handed back by poll().
//   * Answers that match nothing (late, duplicated, from a retry that raced the
//     first answer) are dropped and counted in stats().stale.
//
// Timeouts and retries
//   * `timeout_ms` is the wait for an answer after each send. When it runs out
//     the frame is sent again (same frame id) up to `retries` times, then the
//     request ends with kTimeout. AT commands are safe to repeat. Retrying a
//     transmit request can deliver the data twice if only the status was lost,
//     so transmit requests default to no retries.
//   * If the serial link cannot take the frame yet (output buffer full), it is
//     tried again on later polls, in submission order, for up to
//     `send_timeout_ms`; then the request ends with kSendFailed.
//
// Time is always passed in (wrap-safe 32-bit milliseconds). Not interrupt
// safe; one context at a time. Header-only, C++11, no heap.

#ifndef MCU_XBEE_REQUESTS_HPP
#define MCU_XBEE_REQUESTS_HPP

#include <stddef.h>
#include <stdint.h>

#include "hal/clock.hpp"
#include "xbee/xbee_api.hpp"

namespace mcu {

typedef uint16_t XBeeReqHandle;  // 0 means "no request"

enum class XBeeReqKind : uint8_t { kAt, kRemoteAt, kTx };

enum class XBeeReqResult : uint8_t {
  kPending,      // still waiting
  kOk,           // answered, and the radio reported success (status 0)
  kRadioError,   // answered with an error: see `status` (AT status, or transmit delivery status)
  kTimeout,      // no answer after all attempts
  kSendFailed,   // the frame could not be written to the serial link in time
};

enum class XBeeReqError : uint8_t { kNone, kNoSlot, kBadArg, kTooBig };

struct XBeeReqInfo {
  XBeeReqResult result;
  XBeeReqKind kind;
  uint8_t frame_id;    // 0 for requests that ask for no answer
  uint8_t status;      // AT status (0 ok, 1 error, 2 invalid command, 3 invalid parameter, 4 tx failure)
                       // or transmit delivery status (0 success); see Digi's tables
  uint8_t attempts;    // times the frame was sent
  // AT answers: the value, e.g. {0x7F, 0xFF} for ATID 0x7FFF, or the text of ATNI.
  const uint8_t* data;
  size_t data_len;
  bool data_truncated;  // the answer was longer than MaxData and was cut
  // Transmit status 0x8B only:
  bool tx_extended;
  uint8_t tx_retries;   // MAC retries used
  uint8_t discovery;    // 0 none, 2 route discovery
  uint16_t addr16;
};

class XBeeReqOptions {
 public:
  typedef void (*Callback)(void* ctx, XBeeReqHandle handle, const XBeeReqInfo& info);

  uint32_t timeout_ms;       // wait for an answer after each send
  uint8_t retries;           // extra sends after a timeout
  uint32_t send_timeout_ms;  // how long to keep trying to put the frame on the serial link (0 = timeout_ms)
  bool want_response;        // false: frame id 0, the radio sends no answer; the request ends once sent
  Callback on_done;          // called from poll() when the request ends; the slot is then freed
  void* ctx;
  bool auto_release;         // free the slot when the request ends, even without a callback

  // Local and remote AT commands: 1 s, two retries.
  static XBeeReqOptions at() { return XBeeReqOptions(1000, 2); }
  // Remote AT commands travel over the air: 4 s, one retry.
  static XBeeReqOptions remote_at() { return XBeeReqOptions(4000, 1); }
  // Transmit requests: 5 s, no retries.
  static XBeeReqOptions tx() { return XBeeReqOptions(5000, 0); }

  XBeeReqOptions() : timeout_ms(1000), retries(0), send_timeout_ms(0), want_response(true), on_done(0), ctx(0), auto_release(false) {}

 private:
  XBeeReqOptions(uint32_t timeout, uint8_t retry)
      : timeout_ms(timeout), retries(retry), send_timeout_ms(0), want_response(true), on_done(0), ctx(0), auto_release(false) {}
};

struct XBeeReqStats {
  uint32_t submitted;
  uint32_t ok;
  uint32_t radio_errors;
  uint32_t timeouts;
  uint32_t send_failures;
  uint32_t retries;  // extra sends after a timeout
  uint32_t stale;    // answers that matched no request
};

// The AT answer's data as a big-endian number (1..8 bytes), as most numeric settings are.
inline bool xbee_resp_u64(const XBeeReqInfo& info, uint64_t& out) {
  if (info.data == 0 || info.data_truncated || info.data_len == 0 || info.data_len > 8) return false;
  uint64_t v = 0;
  for (size_t i = 0; i < info.data_len; ++i) v = (v << 8) | info.data[i];
  out = v;
  return true;
}

// Slots: requests in flight at once (1..255). MaxData: longest AT answer kept.
// MaxOut: longest frame data sent; MaxFrame: longest frame data received.
template <size_t Slots = 4, size_t MaxData = 32, size_t MaxOut = 96, size_t MaxFrame = 128>
class XBeeRequests {
  static_assert(Slots >= 1 && Slots <= 255, "Slots must be 1..255");

 public:
  explicit XBeeRequests(Stream& stream, XBeeApiMode mode = XBeeApiMode::kApi2)
      : api_(stream, mode), next_id_(1), seq_(0), ready_(false), err_(XBeeReqError::kNone) {
    stats_ = XBeeReqStats{0, 0, 0, 0, 0, 0, 0};
    for (size_t i = 0; i < Slots; ++i) {
      s_[i].state = kFree;
      s_[i].gen = 0;
    }
  }

  // The underlying codec, for sending frames this layer does not track.
  XBeeApi<MaxFrame, MaxOut>& api() { return api_; }
  void set_mode(XBeeApiMode mode) { api_.set_mode(mode); }

  // ---- submitting (each returns a handle, or 0: see submit_error()) ----

  // Local AT command. With no parameter it queries the setting. Parameters are
  // raw bytes ({0x7F, 0xFF} for ATID 0x7FFF). `queued` uses frame 0x09 (applied by ATAC).
  XBeeReqHandle at(const char* cmd, const uint8_t* param, size_t param_len, uint32_t now,
                   const XBeeReqOptions* opts = 0, bool queued = false) {
    const XBeeReqOptions o = opts ? *opts : XBeeReqOptions::at();
    Slot* s = begin_submit(XBeeReqKind::kAt, o, 0);
    if (!s) return 0;
    s->len = xbee_build_at(s->frame, sizeof s->frame, s->id, cmd, param, param_len, queued);
    return finish_submit(s, o, now, 4 + param_len);
  }

  // AT command on another radio, by 64-bit address.
  XBeeReqHandle remote_at(uint64_t dest64, const char* cmd, const uint8_t* param, size_t param_len,
                          uint32_t now, const XBeeReqOptions* opts = 0,
                          uint8_t options = xbee_frame::kRemoteApplyChanges) {
    const XBeeReqOptions o = opts ? *opts : XBeeReqOptions::remote_at();
    Slot* s = begin_submit(XBeeReqKind::kRemoteAt, o, dest64);
    if (!s) return 0;
    s->len = xbee_build_remote_at(s->frame, sizeof s->frame, s->id, dest64, cmd, param, param_len,
                                  options);
    return finish_submit(s, o, now, 15 + param_len);
  }

  // Transmit request to a 64-bit address (0x000000000000FFFF broadcasts). The
  // answer is the transmit status: kOk means delivered (or sent, for broadcasts).
  XBeeReqHandle tx(uint64_t dest64, const uint8_t* payload, size_t len, uint32_t now,
                   const XBeeReqOptions* opts = 0, uint8_t tx_options = 0, uint8_t radius = 0) {
    const XBeeReqOptions o = opts ? *opts : XBeeReqOptions::tx();
    Slot* s = begin_submit(XBeeReqKind::kTx, o, dest64);
    if (!s) return 0;
    s->len = xbee_build_tx(s->frame, sizeof s->frame, s->id, dest64, payload, len, tx_options, radius);
    return finish_submit(s, o, now, 14 + len);
  }

  XBeeReqError submit_error() const { return err_; }

  // ---- running ----

  // Call every pass of the main loop. Reads the serial link, completes requests
  // that were answered, handles timeouts and retries, and writes frames that
  // were waiting. Returns true when a frame arrived that is NOT an answer to a
  // request; it is valid (frame(), frame_length(), frame_type()) until the next poll().
  bool poll(uint32_t now) {
    ready_ = false;
    while (api_.poll()) {
      if (!dispatch()) {
        ready_ = true;
        break;
      }
    }
    expire(now);
    send_pending(now);
    return ready_;
  }

  const uint8_t* frame() const { return api_.frame(); }
  size_t frame_length() const { return ready_ ? api_.frame_length() : 0; }
  uint8_t frame_type() const { return ready_ ? api_.frame_type() : 0; }

  // ---- results ----

  // The request's state, or 0 if the handle is stale (released or never issued).
  const XBeeReqInfo* info(XBeeReqHandle h) const {
    const Slot* s = lookup(h);
    return s ? &s->info : 0;
  }
  bool done(XBeeReqHandle h) const {
    const Slot* s = lookup(h);
    return s && s->state == kDone;
  }
  // Frees the slot. Works on a finished request and on one still waiting (a late
  // answer is then dropped). Does nothing for a stale handle.
  void release(XBeeReqHandle h) {
    Slot* s = lookup_mut(h);
    if (s) s->state = kFree;
  }

  size_t free_slots() const {
    size_t n = 0;
    for (size_t i = 0; i < Slots; ++i) n += s_[i].state == kFree ? 1 : 0;
    return n;
  }
  // Requests still waiting to be sent or answered.
  size_t pending() const {
    size_t n = 0;
    for (size_t i = 0; i < Slots; ++i) n += (s_[i].state == kSending || s_[i].state == kWaiting) ? 1 : 0;
    return n;
  }
  const XBeeReqStats& stats() const { return stats_; }
  const XBeeParserStats& parser_stats() const { return api_.stats(); }

 private:
  enum State : uint8_t { kFree, kSending, kWaiting, kDone };

  struct Slot {
    State state;
    uint8_t gen;
    uint8_t id;
    uint8_t retries_left;
    bool want_response;
    XBeeReqKind kind;
    uint64_t dest64;
    size_t len;
    uint32_t seq;        // submission order, for sending in order
    uint32_t deadline;   // kSending: give up sending; kWaiting: answer due
    uint32_t timeout_ms;
    uint32_t send_timeout_ms;
    XBeeReqOptions::Callback on_done;
    void* ctx;
    bool auto_release;
    XBeeReqInfo info;
    uint8_t frame[MaxOut];
    uint8_t data[MaxData];
  };

  static XBeeReqHandle make_handle(size_t idx, uint8_t gen) {
    return (XBeeReqHandle)(((unsigned)gen << 8) | (unsigned)(idx + 1));
  }
  const Slot* lookup(XBeeReqHandle h) const {
    const size_t idx = (size_t)(h & 0xFF);
    if (idx == 0 || idx > Slots) return 0;
    const Slot& s = s_[idx - 1];
    return (s.state != kFree && s.gen == (uint8_t)(h >> 8)) ? &s : 0;
  }
  Slot* lookup_mut(XBeeReqHandle h) { return const_cast<Slot*>(lookup(h)); }

  bool id_in_use(uint8_t id) const {
    for (size_t i = 0; i < Slots; ++i)
      if ((s_[i].state == kSending || s_[i].state == kWaiting) && s_[i].want_response && s_[i].id == id)
        return true;
    return false;
  }

  Slot* begin_submit(XBeeReqKind kind, const XBeeReqOptions& o, uint64_t dest64) {
    err_ = XBeeReqError::kNone;
    Slot* s = 0;
    for (size_t i = 0; i < Slots && !s; ++i)
      if (s_[i].state == kFree) s = &s_[i];
    if (!s) {
      err_ = XBeeReqError::kNoSlot;
      return 0;
    }
    uint8_t id = 0;
    if (o.want_response) {
      for (int tries = 0; tries < 256; ++tries) {
        const uint8_t cand = next_id_;
        next_id_ = (uint8_t)(next_id_ == 255 ? 1 : next_id_ + 1);
        if (!id_in_use(cand)) {
          id = cand;
          break;
        }
      }
      if (id == 0) {  // cannot happen with Slots <= 255, but never hand out id 0 by accident
        err_ = XBeeReqError::kNoSlot;
        return 0;
      }
    }
    s->id = id;
    s->kind = kind;
    s->dest64 = dest64;
    s->want_response = o.want_response;
    return s;
  }

  XBeeReqHandle finish_submit(Slot* s, const XBeeReqOptions& o, uint32_t now, size_t need) {
    if (s->len == 0) {  // a builder refused: frame too big for MaxOut, bad command, or null pointer
      err_ = need > MaxOut ? XBeeReqError::kTooBig : XBeeReqError::kBadArg;
      return 0;
    }
    s->gen = (uint8_t)(s->gen == 255 ? 1 : s->gen + 1);
    s->state = kSending;
    s->retries_left = o.retries;
    s->timeout_ms = o.timeout_ms;
    s->send_timeout_ms = o.send_timeout_ms ? o.send_timeout_ms : o.timeout_ms;
    s->deadline = now + s->send_timeout_ms;
    s->seq = seq_++;
    s->on_done = o.on_done;
    s->ctx = o.ctx;
    s->auto_release = o.auto_release;
    XBeeReqInfo& in = s->info;
    in.result = XBeeReqResult::kPending;
    in.kind = s->kind;
    in.frame_id = s->id;
    in.status = 0;
    in.attempts = 0;
    in.data = s->data;
    in.data_len = 0;
    in.data_truncated = false;
    in.tx_extended = false;
    in.tx_retries = 0;
    in.discovery = 0;
    in.addr16 = 0;
    ++stats_.submitted;
    const XBeeReqHandle h = make_handle((size_t)(s - s_), s->gen);
    send_pending(now);  // try to send right away, in order
    // The request may already be over (fire and forget, or a send failure).
    return h;
  }

  // ---- incoming frames ----

  Slot* find_waiting(XBeeReqKind kind, uint8_t id) {
    if (id == 0) return 0;
    for (size_t i = 0; i < Slots; ++i) {
      Slot& s = s_[i];
      if ((s.state == kWaiting || (s.state == kSending && s.info.attempts > 0)) && s.want_response &&
          s.kind == kind && s.id == id)
        return &s;
    }
    return 0;
  }

  // Handles the frame api_ just produced. True if it answered a request (or was a stale answer).
  bool dispatch() {
    const uint8_t* f = api_.frame();
    const size_t n = api_.frame_length();
    switch (api_.frame_type()) {
      case xbee_frame::kAtResponse: {
        XBeeAtResponse r = XBeeAtResponse();
        if (!xbee_parse_at_response(f, n, r)) return false;
        Slot* s = find_waiting(XBeeReqKind::kAt, r.frame_id);
        if (!s) {
          ++stats_.stale;
          return true;
        }
        s->info.status = r.status;
        keep_data(*s, r.data, r.data_len);
        finish(*s, r.status == xbee_frame::kAtOk ? XBeeReqResult::kOk : XBeeReqResult::kRadioError);
        return true;
      }
      case xbee_frame::kRemoteAtResponse: {
        XBeeRemoteAtResponse r = XBeeRemoteAtResponse();
        if (!xbee_parse_remote_at_response(f, n, r)) return false;
        Slot* s = find_waiting(XBeeReqKind::kRemoteAt, r.frame_id);
        if (!s || (s->dest64 != kBroadcast64 && s->dest64 != r.src64)) {
          ++stats_.stale;
          return true;
        }
        s->info.status = r.status;
        keep_data(*s, r.data, r.data_len);
        finish(*s, r.status == xbee_frame::kAtOk ? XBeeReqResult::kOk : XBeeReqResult::kRadioError);
        return true;
      }
      case xbee_frame::kTxStatus:
      case xbee_frame::kTxStatusExtended: {
        XBeeTxStatus t = XBeeTxStatus();
        if (!xbee_parse_tx_status(f, n, t)) return false;
        Slot* s = find_waiting(XBeeReqKind::kTx, t.frame_id);
        if (!s) {
          ++stats_.stale;
          return true;
        }
        s->info.status = t.delivery;
        s->info.tx_extended = t.extended;
        s->info.tx_retries = t.retries;
        s->info.discovery = t.discovery;
        s->info.addr16 = t.addr16;
        finish(*s, t.delivery == 0 ? XBeeReqResult::kOk : XBeeReqResult::kRadioError);
        return true;
      }
      default: return false;
    }
  }

  void keep_data(Slot& s, const uint8_t* d, size_t n) {
    const size_t k = n < MaxData ? n : MaxData;
    for (size_t i = 0; i < k; ++i) s.data[i] = d[i];
    s.info.data_len = k;
    s.info.data_truncated = n > MaxData;
  }

  // ---- timers and sending ----

  void expire(uint32_t now) {
    for (size_t i = 0; i < Slots; ++i) {
      Slot& s = s_[i];
      if (s.state != kWaiting || !time_reached(now, s.deadline)) continue;
      if (s.retries_left > 0) {
        --s.retries_left;
        ++stats_.retries;
        s.state = kSending;
        s.deadline = now + s.send_timeout_ms;
        s.seq = seq_++;
      } else {
        finish(s, XBeeReqResult::kTimeout);
      }
    }
  }

  // Writes waiting frames in submission order. Stops at the first one the link cannot take
  // yet, so a small frame never overtakes a bigger one.
  void send_pending(uint32_t now) {
    for (;;) {
      Slot* first = 0;
      for (size_t i = 0; i < Slots; ++i) {
        Slot& s = s_[i];
        if (s.state != kSending) continue;
        if (!first || (int32_t)(s.seq - first->seq) < 0) first = &s;
      }
      if (!first) return;
      Slot& s = *first;
      if (api_.send_frame(s.frame, s.len)) {
        ++s.info.attempts;
        if (!s.want_response) {
          finish(s, XBeeReqResult::kOk);
        } else {
          s.state = kWaiting;
          s.deadline = now + s.timeout_ms;
        }
        continue;
      }
      if (api_.last_send_result() == XBeeSendResult::kNoSpace) {
        if (time_reached(now, s.deadline)) {
          finish(s, XBeeReqResult::kSendFailed);
          continue;  // others behind it may still go
        }
        return;  // wait for room, keep the order
      }
      finish(s, XBeeReqResult::kSendFailed);  // kPartial / kTooLong: the frame is lost
    }
  }

  void finish(Slot& s, XBeeReqResult r) {
    s.info.result = r;
    s.state = kDone;
    switch (r) {
      case XBeeReqResult::kOk: ++stats_.ok; break;
      case XBeeReqResult::kRadioError: ++stats_.radio_errors; break;
      case XBeeReqResult::kTimeout: ++stats_.timeouts; break;
      case XBeeReqResult::kSendFailed: ++stats_.send_failures; break;
      default: break;
    }
    const uint8_t gen = s.gen;
    if (s.on_done) s.on_done(s.ctx, make_handle((size_t)(&s - s_), gen), s.info);
    // free the slot unless the callback already released (and possibly reused) it
    if ((s.on_done || s.auto_release) && s.state == kDone && s.gen == gen) s.state = kFree;
  }

  static constexpr uint64_t kBroadcast64 = 0x000000000000FFFFull;

  XBeeApi<MaxFrame, MaxOut> api_;
  uint8_t next_id_;
  uint32_t seq_;
  bool ready_;
  XBeeReqError err_;
  XBeeReqStats stats_;
  Slot s_[Slots];
};

}  // namespace mcu

#endif  // MCU_XBEE_REQUESTS_HPP
