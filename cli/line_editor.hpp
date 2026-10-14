// line_editor.hpp - collects typed bytes into a command line, microcontroller agnostic.
//
// Feed it one received byte at a time. It handles what a serial terminal
// sends, and tells you when a line is ready:
//
//   mcu::LineEditor<64> ed(&echo_output);
//   uint8_t b;
//   while (port.read_byte(b)) {
//     if (ed.feed(b) == mcu::LineEvent::kLine) run(ed.line());
//   }
//
// Handled keys:
//   Enter (CR, LF or CR+LF, counted once)  finish the line
//   Backspace (0x08) and Delete (0x7F)     remove the last character
//   Ctrl-U                                 erase the whole line
//   Ctrl-C                                 cancel the line
//   ESC sequences (arrow keys, Home, ...)  swallowed, so they do not insert junk
//   other control characters               ignored
// Tab is kept as a character (the tokenizer treats it as a separator).
//
// Too-long lines: once the buffer is full, extra characters are refused (the
// terminal gets a bell) and counted. Backspace takes the refused characters
// back first, so the user can fix the line. Pressing Enter on a line that is
// still too long reports kTooLong and discards it, so a truncated command is
// never run by accident.
//
// Echo: if an Output is given, typed characters, backspaces and line ends are
// echoed to it. Pass nullptr when the terminal echoes locally or the link is
// not a human.
//
// Header-only, C++11, no heap. Needs output.hpp.

#ifndef MCU_LINE_EDITOR_HPP
#define MCU_LINE_EDITOR_HPP

#include <stddef.h>
#include <stdint.h>

#include "cli/output.hpp"

namespace mcu {

enum class LineEvent : uint8_t {
  kNone,       // still typing
  kLine,       // a complete line is in line()
  kCancelled,  // Ctrl-C
  kTooLong     // Enter pressed on a line that did not fit; it was discarded
};

// N is the buffer size including the terminating NUL, so the longest line is N-1 characters.
template <size_t N>
class LineEditor {
  static_assert(N >= 2, "buffer must hold at least one character");

 public:
  explicit LineEditor(Output* echo = nullptr) : echo_(echo) { reset(); }

  void set_echo(Output* echo) { echo_ = echo; }

  void reset() {
    len_ = 0;
    lost_ = 0;
    esc_ = 0;
    done_ = false;
    skip_lf_ = false;
    buf_[0] = '\0';
  }

  LineEvent feed(uint8_t b) {
    if (done_) {  // the previous line was reported; start a fresh one
      len_ = 0;
      lost_ = 0;
      done_ = false;
    }

    // Escape sequences: ESC [ params final, or ESC O final.
    if (esc_ == 1) {
      if (b < 0x20) {
        esc_ = 0;  // a control key right after ESC: treat ESC as a lone key press
      } else {
        esc_ = (b == '[' || b == 'O') ? 2 : 0;
        return LineEvent::kNone;
      }
    } else if (esc_ == 2) {
      if (b >= 0x40 && b <= 0x7E) {
        esc_ = 0;
        return LineEvent::kNone;
      }
      if (b >= 0x20 && b <= 0x3F) return LineEvent::kNone;
      esc_ = 0;  // interrupted by a control character: handle it normally
    }

    if (b == '\n' && skip_lf_) {  // the LF of a CR+LF pair
      skip_lf_ = false;
      return LineEvent::kNone;
    }
    skip_lf_ = false;

    switch (b) {
      case '\r':
        skip_lf_ = true;
        return finish();
      case '\n':
        return finish();
      case 0x08:
      case 0x7F:
        if (lost_ != 0) {
          --lost_;
        } else if (len_ != 0) {
          --len_;
          echo("\b \b");
        }
        return LineEvent::kNone;
      case 0x03:  // Ctrl-C
        echo("^C\r\n");
        done_ = true;
        return LineEvent::kCancelled;
      case 0x15:  // Ctrl-U
        while (len_ != 0) {
          --len_;
          echo("\b \b");
        }
        lost_ = 0;
        return LineEvent::kNone;
      case 0x1B:
        esc_ = 1;
        return LineEvent::kNone;
      default:
        break;
    }

    if (b < 0x20 && b != '\t') return LineEvent::kNone;  // other control characters
    if (len_ + 1 < N) {
      buf_[len_++] = (char)b;
      if (echo_ != nullptr) echo_->put((char)b);
    } else {
      ++lost_;
      echo("\a");
    }
    return LineEvent::kNone;
  }

  // The finished line, NUL terminated and editable (the tokenizer works in
  // place). Valid after kLine, until the next feed().
  char* line() { return buf_; }
  size_t length() const { return len_; }

 private:
  LineEvent finish() {
    echo("\r\n");
    done_ = true;
    if (lost_ != 0) return LineEvent::kTooLong;
    buf_[len_] = '\0';
    return LineEvent::kLine;
  }

  void echo(const char* s) {
    if (echo_ != nullptr) echo_->print(s);
  }

  Output* echo_;
  char buf_[N];
  size_t len_;
  size_t lost_;
  uint8_t esc_;
  bool done_;
  bool skip_lf_;
};

}  // namespace mcu

#endif  // MCU_LINE_EDITOR_HPP
