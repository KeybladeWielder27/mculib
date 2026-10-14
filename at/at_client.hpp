// at_client.hpp - non-blocking AT command engine, microcontroller agnostic.
//
// Talks to anything that takes text commands and answers with text lines:
// XBee radios in command mode, ESP32/ESP8266 AT firmware, cellular modems,
// BLE modules, GPS receivers. It knows nothing about any particular device;
// device drivers (such as xbee/) are built on top of it.
//
//   mcu::AtClient<> at(uart);                       // uart is any mcu::Stream
//
//   at.command("AT+GMR", now);                      // send, wait for OK / ERROR
//   ...
//   switch (at.poll(clock.millis())) {              // call often; never blocks
//     case mcu::AtStatus::kBusy:    break;          // still waiting
//     case mcu::AtStatus::kOk:      use(at.response()); break;
//     case mcu::AtStatus::kError:   report(at.final_line()); break;
//     case mcu::AtStatus::kTimeout: retry(); break;
//     default: break;
//   }
//
// How a command finishes depends on what the device sends back (AtExpect):
//
//   kFinal   info lines, then "OK" or "ERROR" (also "+CME ERROR: n" and
//            "+CMS ERROR: n"). Most modems, and XBee commands that set
//            something. Info lines are collected in response().
//   kLines   exactly n lines are the answer (default 1). XBee queries:
//            "ATID" answers "7FFF" with no OK after it. "ERROR" still fails.
//   kNone    nothing comes back: finished as soon as the text is sent.
//
// Lines may end in CR, LF or CR LF; empty lines are skipped. A line equal to
// the command just sent is taken as an echo and skipped (set_echo_filter).
// Lines that arrive while no command is waiting for an answer are passed to
// the unsolicited-line handler, if one is set ("RING", "+IPD", ...).
//
// raw() sends text without a line terminator, after first keeping the line
// quiet for a given time. That is the escape sequence of transparent-mode
// radios and modems: one second of silence, "+++", then "OK". Because data
// from the far end may still be arriving just before the OK, in raw() mode
// the final result is matched at the END of a line ("hello worldOK").
//
// Not interrupt safe: use it from one context. Header-only, C++11, no heap.

#ifndef MCU_AT_CLIENT_HPP
#define MCU_AT_CLIENT_HPP

#include <stddef.h>
#include <stdint.h>

#include "hal/clock.hpp"
#include "hal/stream.hpp"

namespace mcu {

enum class AtStatus : uint8_t {
  kIdle,     // nothing started yet (or cancelled)
  kBusy,     // waiting to send, sending, or waiting for the answer
  kOk,       // finished successfully
  kError,    // the device answered ERROR (see final_line())
  kTimeout,  // no complete answer in time
};

enum class AtExpect : uint8_t { kFinal, kLines, kNone };

// CmdN:  longest command text plus terminator and NUL.
// LineN: longest received line kept whole (longer lines are cut, see
//        line_overflows()).
// RespN: room for all info lines of one answer, joined with '\n'.
template <size_t CmdN = 64, size_t LineN = 64, size_t RespN = 128>
class AtClient {
  static_assert(CmdN >= 4, "CmdN too small");
  static_assert(LineN >= 8, "LineN too small");
  static_assert(RespN >= 2, "RespN too small");

 public:
  typedef void (*LineHandler)(void* ctx, const char* line);

  static constexpr uint32_t kDefaultTimeoutMs = 1000;

  // `terminator` is what command() appends: '\r' for nearly every device.
  explicit AtClient(Stream& stream, char terminator = '\r')
      : s_(stream),
        term_(terminator),
        echo_filter_(true),
        urc_(0),
        urc_ctx_(0),
        info_(0),
        info_ctx_(0) {
    reset_state();
  }

  // ---- starting a command ----

  // Sends `text` plus the terminator and waits for the answer. Returns false
  // (and changes nothing) if a command is still running or the text does not
  // fit in CmdN. `lines` is only used with AtExpect::kLines.
  bool command(const char* text, uint32_t now, uint32_t timeout_ms = kDefaultTimeoutMs,
               AtExpect expect = AtExpect::kFinal, uint8_t lines = 1) {
    return begin(text, true, now, 0, timeout_ms, expect, lines);
  }

  // Waits until `quiet_ms` have passed (send nothing else meanwhile), then
  // sends `text` as is, with no terminator, all in one write if the stream
  // reports enough space. The answer is waited for as with command(), but a
  // final "OK"/"ERROR" is recognised at the end of a line. The timeout is
  // counted from when the text has been sent.
  bool raw(const char* text, uint32_t now, uint32_t quiet_ms, uint32_t timeout_ms,
           AtExpect expect = AtExpect::kFinal, uint8_t lines = 1) {
    return begin(text, false, now, quiet_ms, timeout_ms, expect, lines);
  }

  // Abandons the running command. Its answer, if it still arrives, is
  // treated as unsolicited lines.
  void cancel() {
    if (state_ != kIdle) {
      state_ = kIdle;
      status_ = AtStatus::kIdle;
    }
  }

  // Drops everything waiting in the receive buffer and any partial line.
  // Useful before taking over a port that was carrying data.
  void discard_input() {
    uint8_t buf[16];
    while (s_.read(buf, sizeof buf) != 0) {
    }
    len_ = 0;
    line_cut_ = false;
  }

  // ---- running ----

  // Moves bytes in both directions and checks the timeout. Call it often,
  // also when idle, so unsolicited lines are delivered. Returns status():
  // a finished result stays until the next command is started.
  AtStatus poll(uint32_t now) {
    pump_tx(now);
    uint8_t buf[16];
    size_t n;
    while ((n = s_.read(buf, sizeof buf)) != 0) {
      for (size_t i = 0; i < n; ++i) on_byte((char)buf[i]);
    }
    if (state_ == kSending || state_ == kWaiting) {
      if (time_reached(now, deadline_)) finish(AtStatus::kTimeout, "");
    }
    return status_;
  }

  AtStatus status() const { return status_; }
  bool busy() const { return state_ != kIdle; }

  // ---- results (valid once a command has finished) ----

  // Info lines of the answer joined with '\n' (for kLines: the answer lines).
  const char* response() const { return resp_; }
  size_t response_length() const { return resp_len_; }
  bool response_truncated() const { return resp_cut_; }  // some text did not fit in RespN
  uint8_t line_count() const { return lines_got_; }      // info / answer lines received
  // The line that ended the command: "OK", "ERROR", "+CME ERROR: 10", ...
  // Empty after a timeout, kNone, or a kLines command that succeeded.
  const char* final_line() const { return final_; }

  // ---- configuration ----

  // Skip a received line identical to the command just sent (default on).
  void set_echo_filter(bool on) { echo_filter_ = on; }

  // Called for every complete line received while no command waits for an
  // answer. The line is only valid during the call.
  void set_unsolicited_handler(LineHandler fn, void* ctx = 0) {
    urc_ = fn;
    urc_ctx_ = ctx;
  }

  // Called for every info / answer line of a running command, in addition to
  // collecting it in response(). Handy for long listings that do not fit.
  void set_info_handler(LineHandler fn, void* ctx = 0) {
    info_ = fn;
    info_ctx_ = ctx;
  }

  // Number of received lines that were longer than LineN-1 and were cut.
  uint32_t line_overflows() const { return overflows_; }

 private:
  enum State : uint8_t { kIdle, kQuiet, kSending, kWaiting };
  static constexpr size_t kFinalN = 24;

  void reset_state() {
    state_ = kIdle;
    status_ = AtStatus::kIdle;
    cmd_[0] = 0;
    cmd_text_len_ = cmd_len_ = sent_ = 0;
    len_ = 0;
    line_cut_ = false;
    resp_[0] = 0;
    resp_len_ = 0;
    resp_cut_ = false;
    final_[0] = 0;
    lines_got_ = lines_want_ = 0;
    overflows_ = 0;
    start_ = deadline_ = quiet_ = timeout_ = 0;
    expect_ = AtExpect::kFinal;
    raw_ = false;
  }

  bool begin(const char* text, bool terminate, uint32_t now, uint32_t quiet_ms,
             uint32_t timeout_ms, AtExpect expect, uint8_t lines) {
    if (state_ != kIdle || text == 0) return false;
    size_t n = 0;
    while (text[n] != 0) {
      if (n >= CmdN) return false;
      ++n;
    }
    if (n + (terminate ? 1u : 0u) + 1 > CmdN) return false;
    if (n == 0 && !terminate) return false;
    for (size_t i = 0; i < n; ++i) cmd_[i] = text[i];
    cmd_text_len_ = n;
    if (terminate) cmd_[n++] = term_;
    cmd_[n] = 0;
    cmd_len_ = n;
    sent_ = 0;

    resp_[0] = 0;
    resp_len_ = 0;
    resp_cut_ = false;
    final_[0] = 0;
    lines_got_ = 0;
    lines_want_ = (uint8_t)(lines == 0 ? 1 : lines);
    expect_ = expect;
    raw_ = !terminate;
    timeout_ = timeout_ms;
    quiet_ = quiet_ms;
    start_ = now;
    status_ = AtStatus::kBusy;
    state_ = quiet_ms != 0 ? kQuiet : kSending;
    deadline_ = (uint32_t)(now + timeout_ms);
    pump_tx(now);
    return true;
  }

  void pump_tx(uint32_t now) {
    if (state_ == kQuiet) {
      if (elapsed_ms(now, start_) < quiet_) return;
      state_ = kSending;
      deadline_ = (uint32_t)(now + timeout_);
    }
    if (state_ != kSending) return;
    const size_t left = cmd_len_ - sent_;
    if (raw_ && sent_ == 0) {
      // Send an escape sequence in one piece when the stream can say it fits.
      const size_t space = s_.write_space();
      if (space != kUnboundedSpace && space < left) return;
    }
    const size_t w = s_.write(reinterpret_cast<const uint8_t*>(cmd_) + sent_, left);
    if (w > 0) {
      sent_ += w;
      deadline_ = (uint32_t)(now + timeout_);  // progress: the clock starts again
    }
    if (sent_ == cmd_len_) {
      if (expect_ == AtExpect::kNone) {
        finish(AtStatus::kOk, "");
      } else {
        state_ = kWaiting;
        deadline_ = (uint32_t)(now + timeout_);
      }
    }
  }

  void on_byte(char c) {
    if (c == '\r' || c == '\n') {
      end_line();
      return;
    }
    if (len_ + 1 < LineN) {
      line_[len_++] = c;
      return;
    }
    if (!line_cut_) ++overflows_;
    line_cut_ = true;
    if (raw_ && state_ == kWaiting) {
      // Keep the END of an over-long line: the result is matched there.
      const size_t keep = (LineN - 1) / 2;
      for (size_t i = 0; i < keep; ++i) line_[i] = line_[len_ - keep + i];
      len_ = keep;
      line_[len_++] = c;
    }
  }

  void end_line() {
    const size_t n = len_;
    const bool cut = line_cut_;
    len_ = 0;
    line_cut_ = false;
    if (n == 0) return;
    line_[n] = 0;

    if (state_ != kWaiting) {
      if (urc_ != 0) urc_(urc_ctx_, line_);
      return;
    }
    if (echo_filter_ && !cut && !raw_ && same(line_, n, cmd_, cmd_text_len_)) return;

    const bool ok = match_ok(line_, n);
    const bool err = !ok && match_error(line_, n);
    if (expect_ == AtExpect::kFinal) {
      if (ok) {
        finish(AtStatus::kOk, line_);
        return;
      }
      if (err) {
        finish(AtStatus::kError, line_);
        return;
      }
      add_response(line_, n);
      return;
    }
    // kLines
    if (err) {
      finish(AtStatus::kError, line_);
      return;
    }
    add_response(line_, n);
    if (lines_got_ >= lines_want_) finish(AtStatus::kOk, "");
  }

  void add_response(const char* line, size_t n) {
    if (lines_got_ != 0xFF) ++lines_got_;
    if (info_ != 0) info_(info_ctx_, line);
    if (resp_len_ != 0) put_resp('\n');
    for (size_t i = 0; i < n; ++i) put_resp(line[i]);
  }

  void put_resp(char c) {
    if (resp_len_ + 1 < RespN) {
      resp_[resp_len_++] = c;
      resp_[resp_len_] = 0;
    } else {
      resp_cut_ = true;
    }
  }

  void finish(AtStatus st, const char* final_line) {
    size_t i = 0;
    for (; final_line[i] != 0 && i + 1 < kFinalN; ++i) final_[i] = final_line[i];
    final_[i] = 0;
    status_ = st;
    state_ = kIdle;
  }

  static bool same(const char* a, size_t an, const char* b, size_t bn) {
    if (an != bn) return false;
    for (size_t i = 0; i < an; ++i)
      if (a[i] != b[i]) return false;
    return true;
  }
  static size_t length(const char* s) {
    size_t n = 0;
    while (s[n] != 0) ++n;
    return n;
  }
  static bool starts_with(const char* s, size_t n, const char* p) {
    const size_t pn = length(p);
    return n >= pn && same(s, pn, p, pn);
  }
  static bool ends_with(const char* s, size_t n, const char* p) {
    const size_t pn = length(p);
    return n >= pn && same(s + (n - pn), pn, p, pn);
  }

  bool match_ok(const char* s, size_t n) const {
    return raw_ ? ends_with(s, n, "OK") : same(s, n, "OK", 2);
  }
  bool match_error(const char* s, size_t n) const {
    if (raw_) return ends_with(s, n, "ERROR");
    return same(s, n, "ERROR", 5) || starts_with(s, n, "+CME ERROR") ||
           starts_with(s, n, "+CMS ERROR");
  }

  Stream& s_;
  char term_;
  bool echo_filter_;
  LineHandler urc_;
  void* urc_ctx_;
  LineHandler info_;
  void* info_ctx_;

  State state_;
  AtStatus status_;
  AtExpect expect_;
  bool raw_;

  char cmd_[CmdN];
  size_t cmd_text_len_;  // without the terminator
  size_t cmd_len_;       // with it
  size_t sent_;

  char line_[LineN];
  size_t len_;
  bool line_cut_;
  uint32_t overflows_;

  char resp_[RespN];
  size_t resp_len_;
  bool resp_cut_;
  char final_[kFinalN];
  uint8_t lines_got_;
  uint8_t lines_want_;

  uint32_t start_;
  uint32_t deadline_;
  uint32_t quiet_;
  uint32_t timeout_;
};

}  // namespace mcu

#endif  // MCU_AT_CLIENT_HPP
