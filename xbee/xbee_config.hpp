// xbee_config.hpp - configure an XBee3 through its AT command mode.
// Non-blocking, microcontroller agnostic, built on at/at_client.hpp.
//
// Describe the settings you want as a constant table, hand it over, and poll:
//
//   static const mcu::XBeeSetting kProfile[] = {
//       mcu::xb_num("ID", 0x7FFF),        // network id
//       mcu::xb_num("CH", 0x0C),          // channel
//       mcu::xb_num("PL", 4),             // power level
//       mcu::xb_text("NI", "Base"),       // node identifier
//       mcu::xb_num("BD", 7),             // 115200 baud
//   };
//
//   mcu::ArduinoStream<HardwareSerial> uart(Serial1);
//   mcu::ArduinoBaudControl<HardwareSerial> baud(Serial1);
//   mcu::XBeeConfigurator<> xb(uart, &baud);
//
//   xb.begin_profile(kProfile, sizeof kProfile / sizeof kProfile[0], millis());
//   ...
//   switch (xb.poll(millis())) {                       // call often; never blocks
//     case mcu::XBeeStatus::kDone:   /* xb.changed_count() settings were different */ break;
//     case mcu::XBeeStatus::kFailed: /* xb.error(), xb.failed_setting() */ break;
//     default: break;
//   }
//
// What a profile does, in order:
//   1. enters command mode (+++ with guard times), trying the baud rates in
//      options().baud_candidates if the radio's current rate is unknown
//   2. reads every setting and notes the ones that differ
//   3. writes only those, then ATAC to apply
//   4. if BD changed, retunes the host UART (needs a BaudControl)
//   5. reads the changed settings back to verify them
//   6. ATWR to save to flash, but only if something changed, so a radio that
//      is already configured is not worn by a flash write on every boot
//   7. ATCN to leave command mode
//
// Single operations (begin_get, begin_set, begin_command, ...) are available
// for everything else. Not interrupt safe. Header-only, C++11, no heap.

#ifndef MCU_XBEE_CONFIG_HPP
#define MCU_XBEE_CONFIG_HPP

#include <stddef.h>
#include <stdint.h>

#include "at/at_client.hpp"
#include "at/at_text.hpp"
#include "hal/baud.hpp"
#include "hal/clock.hpp"
#include "hal/stream.hpp"

namespace mcu {

// ---------------------------------------------------------------------------
// Settings tables
// ---------------------------------------------------------------------------
struct XBeeSetting {
  char c0, c1;        // the two-letter AT command, e.g. 'I','D'
  bool is_text;
  uint64_t num;       // value when !is_text (XBee values are hex of up to 64 bits)
  const char* text;   // value when is_text
};

// Numeric setting: xb_num("ID", 0x7FFF). The name must be a 2-letter literal.
constexpr XBeeSetting xb_num(const char (&name)[3], uint64_t value) {
  return XBeeSetting{name[0], name[1], false, value, 0};
}

// Text setting: xb_text("NI", "Base"). The text must stay valid while in use.
constexpr XBeeSetting xb_text(const char (&name)[3], const char* text) {
  return XBeeSetting{name[0], name[1], true, 0, text};
}

// The host baud rate for an XBee BD value: 0..10 are the standard rates
// (1200 ... 921600), 0x12C..0xEC400 are the rate itself. False if invalid.
inline bool xbee_bd_to_baud(uint64_t bd, uint32_t& baud) {
  static const uint32_t kStd[11] = {1200,  2400,  4800,   9600,   19200, 38400,
                                    57600, 115200, 230400, 460800, 921600};
  if (bd <= 10) {
    baud = kStd[bd];
    return true;
  }
  if (bd >= 0x12Cu && bd <= 0xEC400u) {
    baud = (uint32_t)bd;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Status and options
// ---------------------------------------------------------------------------
enum class XBeeStatus : uint8_t { kIdle, kBusy, kDone, kFailed };

enum class XBeeError : uint8_t {
  kNone,
  kNoResponse,     // never got OK after +++ (wrong baud, not an XBee, wrong guard time)
  kTimeout,        // a command was not answered in time
  kCommandError,   // the radio answered ERROR (bad name or out-of-range value)
  kBadReply,       // an answer was not a hex number
  kVerifyFailed,   // a setting read back different from what was written
  kBaudChange,     // the host UART refused the new baud rate
};

struct XBeeOptions {
  const char* escape;                 // command-mode sequence (CC): "+++"
  uint32_t guard_ms;                  // GT: silence around the escape (default 1000)
  uint32_t reply_ms;                  // wait for an answer to a normal command
  uint32_t slow_ms;                   // wait for ATAC / ATWR, which can take a while
  uint32_t baud_settle_ms;            // pause around a host baud change
  uint32_t command_mode_ms;           // CT: radio leaves command mode after this idle time
  uint8_t enter_attempts;             // tries per baud rate
  bool enter;                         // profile: enter command mode first if not believed in it
  bool verify;                        // profile: read changed settings back
  bool write;                         // profile: ATWR if anything changed
  bool exit;                          // profile: ATCN at the end
  const uint32_t* baud_candidates;    // host baud rates to try when entering (0 = current only)
  uint8_t baud_count;

  XBeeOptions()
      : escape("+++"),
        guard_ms(1000),
        reply_ms(1000),
        slow_ms(3000),
        baud_settle_ms(100),
        command_mode_ms(10000),
        enter_attempts(2),
        enter(true),
        verify(true),
        write(true),
        exit(true),
        baud_candidates(0),
        baud_count(0) {}
};

// MaxSettings: longest profile. CmdN/LineN/RespN: AtClient buffers; the
// defaults fit names, hex values and an NI of up to 20 characters.
template <size_t MaxSettings = 24, size_t CmdN = 40, size_t LineN = 40, size_t RespN = 40>
class XBeeConfigurator {
  static_assert(MaxSettings >= 1 && MaxSettings <= 255, "MaxSettings must be 1..255");
  static_assert(CmdN >= 12, "CmdN too small");

 public:
  static constexpr size_t kTextMax = RespN - 1;

  explicit XBeeConfigurator(Stream& stream, BaudControl* baud = 0)
      : at_(stream), baud_(baud) {
    reset_state();
  }

  XBeeOptions& options() { return opts_; }
  const XBeeOptions& options() const { return opts_; }

  // ---- starting jobs (each returns false if busy or the arguments are bad) ----

  // Brings the radio into command mode (see options for guard time and bauds).
  bool begin_enter(uint32_t now) {
    if (busy()) return false;
    start_job(kJobEnter);
    start_enter(now);
    return true;
  }

  // Reads one setting. The result is value() (numeric) or text().
  bool begin_get(const char* name, uint32_t now) { return begin_single(kSingleGet, name, 0, 0, now); }
  bool begin_get_text(const char* name, uint32_t now) {
    return begin_single(kSingleGetText, name, 0, 0, now);
  }
  // Writes one setting (not applied until ATAC / ATCN, not saved until ATWR).
  bool begin_set(const char* name, uint64_t value, uint32_t now) {
    return begin_single(kSingleSet, name, value, 0, now);
  }
  bool begin_set_text(const char* name, const char* text, uint32_t now) {
    return text != 0 && begin_single(kSingleSetText, name, 0, text, now);
  }
  // A command with no value and an OK answer: "AC" apply, "WR" save, "CN" exit
  // command mode, "RE" restore defaults, "FR" software reset.
  bool begin_command(const char* name, uint32_t now) {
    return begin_single(kSingleCmd, name, 0, 0, now);
  }

  // Applies a whole profile (see the top of this file). `items` must stay valid
  // until the job ends.
  bool begin_profile(const XBeeSetting* items, size_t count, uint32_t now) {
    if (busy() || items == 0 || count == 0 || count > MaxSettings) return false;
    for (size_t i = 0; i < count; ++i) {
      const XBeeSetting& s = items[i];
      if (!alnum(s.c0) || !alnum(s.c1)) return false;
      if (s.is_text) {
        if (s.text == 0 || !valid_text(s.text)) return false;
      } else if (s.c0 == 'B' && s.c1 == 'D') {
        uint32_t b;
        if (baud_ == 0 || !xbee_bd_to_baud(s.num, b)) return false;  // needs a BaudControl
      }
    }
    start_job(kJobProfile);
    items_ = items;
    n_ = count;
    if (opts_.enter && !in_command_mode(now)) {
      start_enter(now);
    } else {
      start_reads(now);
    }
    return true;
  }

  // ---- running ----

  XBeeStatus poll(uint32_t now) {
    const AtStatus st = at_.poll(now);
    if (status_ != XBeeStatus::kBusy) return status_;
    if (phase_ == kSettle) {
      if (time_reached(now, until_)) after_settle(now);
      return status_;
    }
    if (st != AtStatus::kBusy) on_command_done(st, now);
    return status_;
  }

  XBeeStatus status() const { return status_; }
  bool busy() const { return status_ == XBeeStatus::kBusy; }
  XBeeError error() const { return error_; }

  // ---- results ----

  // For a failed profile: index in the table of the setting that failed
  // (-1 when the failure was not tied to one).
  int failed_setting() const { return failed_; }
  size_t changed_count() const { return changed_count_; }  // settings that differed (profile)
  uint64_t value() const { return value_; }                // result of begin_get
  const char* text() const { return text_; }               // result of begin_get_text
  // Host baud rate at which the radio answered (0 if candidates were not used or no job ran).
  uint32_t detected_baud() const { return detected_baud_; }
  // Last AT status, for diagnostics.
  AtStatus last_at_status() const { return last_at_; }
  const char* last_final_line() const { return at_.final_line(); }

  // True if the radio is believed to be in command mode: we entered it and
  // fewer than command_mode_ms (CT) have passed since the last command.
  bool in_command_mode(uint32_t now) const {
    return in_cmd_ && elapsed_ms(now, activity_) < opts_.command_mode_ms;
  }

  // Unsolicited lines from the radio (e.g. while not configuring).
  void set_unsolicited_handler(typename AtClient<CmdN, LineN, RespN>::LineHandler fn,
                               void* ctx = 0) {
    at_.set_unsolicited_handler(fn, ctx);
  }

 private:
  enum Job : uint8_t { kJobNone, kJobEnter, kJobSingle, kJobProfile };
  enum Single : uint8_t { kSingleGet, kSingleGetText, kSingleSet, kSingleSetText, kSingleCmd };
  enum Phase : uint8_t {
    kIdle,
    kEnterWait,
    kSingleWait,
    kRead,
    kSet,
    kApply,
    kSettle,
    kVerify,
    kWrite,
    kExit,
  };

  typedef TextBuilder<CmdN> Cmd;

  static bool alnum(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
  }
  static bool valid_text(const char* t) {
    size_t n = 0;
    for (; t[n] != 0; ++n) {
      const char c = t[n];
      if (c < 0x20 || c > 0x7E || c == ',') return false;  // ',' chains commands
    }
    return n + 5 <= CmdN;  // "AT" + name + text + CR + NUL
  }
  static bool streq(const char* a, const char* b) {
    while (*a != 0 && *a == *b) {
      ++a;
      ++b;
    }
    return *a == *b;
  }

  void reset_state() {
    status_ = XBeeStatus::kIdle;
    error_ = XBeeError::kNone;
    job_ = kJobNone;
    phase_ = kIdle;
    single_ = kSingleCmd;
    items_ = 0;
    n_ = idx_ = 0;
    failed_ = -1;
    changed_count_ = 0;
    for (size_t i = 0; i < sizeof changed_ / sizeof changed_[0]; ++i) changed_[i] = 0;
    bd_changed_ = false;
    new_baud_ = 0;
    value_ = 0;
    text_[0] = 0;
    detected_baud_ = 0;
    in_cmd_ = false;
    activity_ = 0;
    until_ = 0;
    attempt_ = 0;
    cand_ = 0;
    last_at_ = AtStatus::kIdle;
    sname_[0] = sname_[1] = 0;
  }

  void start_job(Job j) {
    const bool keep_cmd = in_cmd_;
    const uint32_t keep_act = activity_;
    const uint32_t keep_baud = detected_baud_;
    reset_state();
    in_cmd_ = keep_cmd;
    activity_ = keep_act;
    detected_baud_ = keep_baud;
    job_ = j;
    status_ = XBeeStatus::kBusy;
  }

  void finish_ok() {
    status_ = XBeeStatus::kDone;
    phase_ = kIdle;
  }

  void fail(XBeeError e, int setting = -1) {
    error_ = e;
    failed_ = setting;
    status_ = XBeeStatus::kFailed;
    phase_ = kIdle;
    at_.cancel();
    if (e == XBeeError::kNoResponse || e == XBeeError::kTimeout) in_cmd_ = false;
  }

  bool send(const Cmd& c, uint32_t now, uint32_t timeout, AtExpect ex) {
    if (!c.ok() || !at_.command(c.c_str(), now, timeout, ex)) {
      fail(XBeeError::kBadReply);  // cannot happen after validation; defensive
      return false;
    }
    return true;
  }

  static void put_name(Cmd& c, char a, char b) { c.add("AT").add(a).add(b); }

  // ---- entering command mode ----

  void start_enter(uint32_t now) {
    phase_ = kEnterWait;
    attempt_ = 0;
    cand_ = 0;
    try_enter(now);
  }

  void try_enter(uint32_t now) {
    if (opts_.baud_count != 0 && baud_ != 0 && opts_.baud_candidates != 0) {
      if (!baud_->set_baud(opts_.baud_candidates[cand_])) {
        next_enter_attempt(now, true);
        return;
      }
    }
    at_.discard_input();
    if (!at_.raw(opts_.escape, now, opts_.guard_ms, opts_.guard_ms + opts_.reply_ms))
      fail(XBeeError::kNoResponse);
  }

  void next_enter_attempt(uint32_t now, bool skip_baud) {
    if (!skip_baud) ++attempt_;
    if (!skip_baud && attempt_ < opts_.enter_attempts) {
      try_enter(now);
      return;
    }
    if (opts_.baud_count != 0 && (uint8_t)(cand_ + 1) < opts_.baud_count) {
      ++cand_;
      attempt_ = 0;
      try_enter(now);
      return;
    }
    fail(XBeeError::kNoResponse);
  }

  // ---- single operations ----

  bool begin_single(Single kind, const char* name, uint64_t value, const char* text,
                    uint32_t now) {
    if (busy() || name == 0 || !alnum(name[0]) || !alnum(name[1]) || name[2] != 0) return false;
    if (kind == kSingleSetText && !valid_text(text)) return false;
    start_job(kJobSingle);
    single_ = kind;
    sname_[0] = name[0];
    sname_[1] = name[1];
    phase_ = kSingleWait;
    Cmd c;
    put_name(c, name[0], name[1]);
    AtExpect ex = AtExpect::kFinal;
    uint32_t to = opts_.reply_ms;
    if (kind == kSingleGet || kind == kSingleGetText) {
      ex = AtExpect::kLines;
    } else if (kind == kSingleSet) {
      c.add_hex(value);
    } else if (kind == kSingleSetText) {
      c.add(text);
    } else if ((name[0] == 'A' && name[1] == 'C') || (name[0] == 'W' && name[1] == 'R')) {
      to = opts_.slow_ms;
    }
    send(c, now, to, ex);
    return true;
  }

  // ---- profile pieces ----

  bool is_changed(size_t i) const { return (changed_[i >> 5] >> (i & 31)) & 1u; }
  void mark_changed(size_t i) { changed_[i >> 5] |= (uint32_t)1 << (i & 31); }

  size_t next_changed(size_t from) const {
    while (from < n_ && !is_changed(from)) ++from;
    return from;
  }

  void query(const XBeeSetting& s, uint32_t now) {
    Cmd c;
    put_name(c, s.c0, s.c1);
    send(c, now, opts_.reply_ms, AtExpect::kLines);
  }

  void start_reads(uint32_t now) {
    phase_ = kRead;
    idx_ = 0;
    query(items_[0], now);
  }

  void send_set(size_t i, uint32_t now) {
    const XBeeSetting& s = items_[i];
    Cmd c;
    put_name(c, s.c0, s.c1);
    if (s.is_text) {
      c.add(s.text);
    } else {
      c.add_hex(s.num);
    }
    send(c, now, opts_.reply_ms, AtExpect::kFinal);
  }

  void start_apply(uint32_t now) {
    phase_ = kApply;
    Cmd c;
    c.add("ATAC");
    if (bd_changed_) {
      // The radio switches baud rate as it applies; its OK may be lost or
      // garbled, so do not wait for one. The host retunes and verifies instead.
      send(c, now, opts_.reply_ms, AtExpect::kNone);
    } else {
      send(c, now, opts_.slow_ms, AtExpect::kFinal);
    }
  }

  void start_verify(uint32_t now) {
    if (!opts_.verify) {
      start_write(now);
      return;
    }
    phase_ = kVerify;
    idx_ = next_changed(0);
    query(items_[idx_], now);
  }

  void start_write(uint32_t now) {
    if (changed_count_ != 0 && opts_.write) {
      phase_ = kWrite;
      Cmd c;
      c.add("ATWR");
      send(c, now, opts_.slow_ms, AtExpect::kFinal);
    } else {
      start_exit(now);
    }
  }

  void start_exit(uint32_t now) {
    if (opts_.exit) {
      phase_ = kExit;
      Cmd c;
      c.add("ATCN");
      send(c, now, opts_.reply_ms, AtExpect::kFinal);
    } else {
      finish_ok();
    }
  }

  void after_settle(uint32_t now) {
    if (!baud_->set_baud(new_baud_)) {
      fail(XBeeError::kBaudChange);
      return;
    }
    at_.discard_input();
    detected_baud_ = new_baud_;
    start_verify(now);
  }

  // Compares an answer with the wanted value. False = could not parse.
  bool answer_matches(const XBeeSetting& s, bool& same) const {
    const char* r = at_.response();
    if (s.is_text) {
      same = streq(r, s.text);
      return true;
    }
    uint64_t v;
    if (!parse_hex_u64(r, v)) return false;
    same = (v == s.num);
    return true;
  }

  void on_command_done(AtStatus st, uint32_t now) {
    last_at_ = st;
    if (st == AtStatus::kError) {
      fail(XBeeError::kCommandError, phase_is_item() ? (int)idx_ : -1);
      return;
    }
    if (st == AtStatus::kTimeout) {
      if (phase_ == kEnterWait) {
        next_enter_attempt(now, false);
      } else {
        fail(XBeeError::kTimeout, phase_is_item() ? (int)idx_ : -1);
      }
      return;
    }
    // kOk
    if (phase_ == kEnterWait) {
      in_cmd_ = true;
      activity_ = now;
      detected_baud_ = (opts_.baud_count != 0 && opts_.baud_candidates != 0)
                           ? opts_.baud_candidates[cand_]
                           : 0;
      if (job_ == kJobEnter) {
        finish_ok();
      } else {
        start_reads(now);
      }
      return;
    }
    in_cmd_ = true;
    activity_ = now;
    switch (phase_) {
      case kSingleWait: finish_single(); break;
      case kRead: {
        bool same = false;
        if (!answer_matches(items_[idx_], same)) {
          fail(XBeeError::kBadReply, (int)idx_);
          return;
        }
        if (!same) {
          mark_changed(idx_);
          ++changed_count_;
          if (!items_[idx_].is_text && items_[idx_].c0 == 'B' && items_[idx_].c1 == 'D') {
            bd_changed_ = true;
            xbee_bd_to_baud(items_[idx_].num, new_baud_);  // validated in begin_profile
          }
        }
        if (++idx_ < n_) {
          query(items_[idx_], now);
        } else if (changed_count_ == 0) {
          start_exit(now);  // nothing differs: no apply, no flash write
        } else {
          phase_ = kSet;
          idx_ = next_changed(0);
          send_set(idx_, now);
        }
        break;
      }
      case kSet: {
        idx_ = next_changed(idx_ + 1);
        if (idx_ < n_) {
          send_set(idx_, now);
        } else {
          start_apply(now);
        }
        break;
      }
      case kApply:
        if (bd_changed_) {
          phase_ = kSettle;
          until_ = (uint32_t)(now + opts_.baud_settle_ms);
        } else {
          start_verify(now);
        }
        break;
      case kVerify: {
        bool same = false;
        if (!answer_matches(items_[idx_], same)) {
          fail(XBeeError::kBadReply, (int)idx_);
          return;
        }
        const XBeeSetting& s = items_[idx_];
        const bool is_bd = !s.is_text && s.c0 == 'B' && s.c1 == 'D';
        // A custom BD reads back as the rate the hardware can actually do.
        if (!same && !(is_bd && s.num > 10)) {
          fail(XBeeError::kVerifyFailed, (int)idx_);
          return;
        }
        idx_ = next_changed(idx_ + 1);
        if (idx_ < n_) {
          query(items_[idx_], now);
        } else {
          start_write(now);
        }
        break;
      }
      case kWrite: start_exit(now); break;
      case kExit:
        in_cmd_ = false;
        finish_ok();
        break;
      default: break;
    }
  }

  bool phase_is_item() const {
    return job_ == kJobProfile &&
           (phase_ == kRead || phase_ == kSet || phase_ == kVerify) && idx_ < n_;
  }

  void finish_single() {
    if (single_ == kSingleGet) {
      uint64_t v;
      if (!parse_hex_u64(at_.response(), v)) {
        fail(XBeeError::kBadReply);
        return;
      }
      value_ = v;
    } else if (single_ == kSingleGetText) {
      const char* r = at_.response();
      size_t i = 0;
      for (; r[i] != 0 && i < kTextMax; ++i) text_[i] = r[i];
      text_[i] = 0;
    } else if (single_ == kSingleCmd && sname_[0] == 'C' && sname_[1] == 'N') {
      in_cmd_ = false;
    }
    finish_ok();
  }

  AtClient<CmdN, LineN, RespN> at_;
  BaudControl* baud_;
  XBeeOptions opts_;

  XBeeStatus status_;
  XBeeError error_;
  Job job_;
  Phase phase_;
  Single single_;

  const XBeeSetting* items_;
  size_t n_;
  size_t idx_;
  int failed_;
  size_t changed_count_;
  uint32_t changed_[(MaxSettings + 31) / 32];
  bool bd_changed_;
  uint32_t new_baud_;

  uint64_t value_;
  char text_[RespN];
  uint32_t detected_baud_;

  bool in_cmd_;
  uint32_t activity_;
  uint32_t until_;
  uint8_t attempt_;
  uint8_t cand_;
  AtStatus last_at_;

  char sname_[2];
};

}  // namespace mcu

#endif  // MCU_XBEE_CONFIG_HPP
