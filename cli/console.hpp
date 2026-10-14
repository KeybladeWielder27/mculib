// console.hpp - a serial command console: line editing, tokenizing, command
// table, built-in help, and queued non-blocking output. Microcontroller
// agnostic: it talks to any mcu::Stream.
//
//   static int cmd_add(mcu::CommandContext& c) {
//     int32_t a, b;
//     if (!c.get_int(1, a) || !c.get_int(2, b)) return mcu::kCmdReported;
//     c.out.print_int(a + b);
//     c.out.println();
//     return mcu::kCmdOk;
//   }
//
//   static const mcu::Command kCommands[] = {
//     // name   handler  help text          usage    min max args
//     {"add",  cmd_add, "add two numbers", "<a> <b>", 2, 2},
//     {"echo", cmd_echo, "print text",     "[text]",  0, mcu::kAnyArgs},
//   };
//
//   mcu::Console<64, 8, 256> console(port, kCommands, sizeof kCommands / sizeof kCommands[0]);
//   console.begin();                 // prints the prompt
//   for (;;) { console.poll(); ... } // call from your main loop
//
// Template arguments: longest line, most arguments (including the command
// name), and size of the output queue in bytes (a power of two).
//
// How it behaves:
//   * poll() never blocks. Each call sends queued output as far as the stream
//     accepts it, reads input, and runs at most one command.
//   * Back-pressure: while queued output is still waiting for the stream, no
//     new input is read. A slow link slows the console down instead of losing
//     text, and a long reply cannot be overtaken by the next command.
//   * Output a handler produces goes into the queue. If a single command
//     prints more than the queue holds while the stream is full, the excess is
//     dropped and counted (dropped_output()). Size the queue for your longest
//     reply.
//   * "help" lists the commands, "help <name>" shows one. A command in your
//     table named "help" replaces the built-in.
//
// Handler return values: kCmdOk (0) = fine, kCmdUsage = print the usage line,
// kCmdReported = failed and the handler already said why, anything else =
// "error <code>" is printed.
//
// Command tables are plain const arrays. On AVR they occupy RAM unless you
// place them in flash yourself.
//
// Header-only, C++11, no heap. Needs hal/stream.hpp and ring_buffer/.

#ifndef MCU_CONSOLE_HPP
#define MCU_CONSOLE_HPP

#include <stddef.h>
#include <stdint.h>

#include "cli/line_editor.hpp"
#include "cli/output.hpp"
#include "cli/parse.hpp"
#include "cli/tokenizer.hpp"
#include "hal/stream.hpp"
#include "ring_buffer/ring_buffer.hpp"

namespace mcu {

constexpr int kCmdOk = 0;
constexpr int kCmdUsage = 1;      // console prints "usage: name <usage>"
constexpr int kCmdReported = -1;  // handler already printed the problem
constexpr uint8_t kAnyArgs = 255;

struct CommandContext {
  size_t argc;   // number of entries in argv, including the command name
  char** argv;   // argv[0] is the command name, argv[argc] is nullptr
  Output& out;   // print results here
  void* user;    // the pointer given to the Console

  // Argument i as text, or "" if there is none.
  const char* arg(size_t i) const { return i < argc ? argv[i] : ""; }

  // Typed argument access. On a bad value these print "error: bad argument N:
  // text" and return false, so a handler can simply return kCmdReported.
  bool get_uint(size_t i, uint32_t& v, uint32_t max = 0xFFFFFFFFu) {
    if (parse_uint(arg(i), v, max)) return true;
    bad_arg(i);
    return false;
  }
  bool get_int(size_t i, int32_t& v, int32_t min = kInt32Min, int32_t max = kInt32Max) {
    if (parse_int(arg(i), v, min, max)) return true;
    bad_arg(i);
    return false;
  }
  bool get_bool(size_t i, bool& v) {
    if (parse_bool(arg(i), v)) return true;
    bad_arg(i);
    return false;
  }
  bool get_scaled(size_t i, int32_t& v, unsigned decimals) {
    if (parse_scaled(arg(i), v, decimals)) return true;
    bad_arg(i);
    return false;
  }

 private:
  void bad_arg(size_t i) {
    out.print("error: bad argument ");
    out.print_uint((uint32_t)i);
    out.print(": ");
    out.println(arg(i));
  }
};

typedef int (*CommandHandler)(CommandContext& ctx);

struct Command {
  const char* name;
  CommandHandler handler;
  const char* help;    // one line, shown by "help"
  const char* usage;   // arguments, e.g. "<addr> [count]"; may be nullptr
  uint8_t min_args;    // not counting the command name
  uint8_t max_args;    // or kAnyArgs
};

template <size_t MaxLine = 64, size_t MaxArgs = 8, size_t TxSize = 128>
class Console {
  static_assert(MaxArgs >= 1, "need room for at least the command name");

 public:
  Console(Stream& io, const Command* commands, size_t count, void* user = nullptr)
      : io_(io),
        cmds_(commands),
        count_(count),
        user_(user),
        tx_(io),
        editor_(&tx_),
        prompt_("> "),
        ignore_case_(false) {}

  Console(const Console&) = delete;
  Console& operator=(const Console&) = delete;

  void set_prompt(const char* prompt) { prompt_ = prompt; }
  void set_user(void* user) { user_ = user; }
  void set_echo(bool on) { editor_.set_echo(on ? static_cast<Output*>(&tx_) : nullptr); }
  void set_ignore_case(bool on) { ignore_case_ = on; }

  // Prints the first prompt.
  void begin() {
    tx_.print(prompt_);
    tx_.flush();
  }

  // Call often from the main loop.
  void poll() {
    tx_.flush();
    if (tx_.pending() != 0) return;  // stream is behind: wait before taking more input
    uint8_t b;
    while (io_.read_byte(b)) {
      const LineEvent ev = editor_.feed(b);
      if (ev == LineEvent::kNone) continue;
      handle(ev);
      break;  // one command per call keeps the main loop responsive
    }
    tx_.flush();
  }

  // For banners and messages not tied to a command. Mixes with command output
  // in order, but may interrupt a line the user is typing.
  Output& out() { return tx_; }

  // Bytes of output lost because the queue and the stream were both full.
  size_t dropped_output() const { return tx_.dropped(); }

 private:
  class TxBuffer final : public Output {
   public:
    explicit TxBuffer(Stream& io) : io_(io), dropped_(0) {}

    size_t write(const uint8_t* src, size_t len) override {
      size_t n = q_.write(src, len);
      if (n < len) {  // queue full: give the stream a chance to drain it
        flush();
        n += q_.write(src + n, len - n);
      }
      dropped_ += len - n;
      return n;
    }

    // Moves queued bytes to the stream, as many as it will take right now.
    void flush() {
      uint8_t chunk[16];
      while (!q_.empty()) {
        const size_t room = io_.write_space();
        if (room == 0) return;
        size_t n = q_.size();
        if (n > room) n = room;
        if (n > sizeof chunk) n = sizeof chunk;
        n = q_.read(chunk, n);
        const size_t w = io_.write(chunk, n);
        if (w < n) {  // the stream took less than it promised; those bytes are gone
          dropped_ += n - w;
          return;
        }
      }
    }

    size_t pending() const { return q_.size(); }
    size_t dropped() const { return dropped_; }

   private:
    Stream& io_;
    RingBuffer<uint8_t, TxSize> q_;
    size_t dropped_;
  };

  void handle(LineEvent ev) {
    if (ev == LineEvent::kLine) {
      run(editor_.line());
    } else if (ev == LineEvent::kTooLong) {
      tx_.println("error: line too long");
    }
    tx_.print(prompt_);
  }

  void run(char* line) {
    char* argv[MaxArgs + 1];
    size_t argc;
    const TokenizeStatus st = tokenize(line, argv, MaxArgs, argc);
    if (st == TokenizeStatus::kTooManyArgs) {
      tx_.println("error: too many arguments");
      return;
    }
    if (st == TokenizeStatus::kUnterminatedQuote) {
      tx_.println("error: unterminated quote");
      return;
    }
    if (argc == 0) return;

    const Command* cmd = find(argv[0]);
    if (cmd == nullptr) {
      if (names_equal(argv[0], "help")) {
        help(argc, argv);
      } else {
        tx_.print("unknown command: ");
        tx_.print(argv[0]);
        tx_.println(" (try 'help')");
      }
      return;
    }

    const size_t nargs = argc - 1;
    if (nargs < cmd->min_args || (cmd->max_args != kAnyArgs && nargs > cmd->max_args)) {
      print_usage(*cmd);
      return;
    }

    CommandContext ctx = {argc, argv, tx_, user_};
    const int rc = cmd->handler(ctx);
    if (rc == kCmdUsage) {
      print_usage(*cmd);
    } else if (rc != kCmdOk && rc != kCmdReported) {
      tx_.print("error ");
      tx_.print_int(rc);
      tx_.println();
    }
  }

  void help(size_t argc, char** argv) {
    if (argc >= 2) {
      const Command* cmd = find(argv[1]);
      if (cmd == nullptr) {
        tx_.print("unknown command: ");
        tx_.println(argv[1]);
        return;
      }
      tx_.print(cmd->name);
      if (cmd->usage != nullptr) {
        tx_.put(' ');
        tx_.print(cmd->usage);
      }
      tx_.println();
      if (cmd->help != nullptr) {
        tx_.print("  ");
        tx_.println(cmd->help);
      }
      return;
    }
    size_t width = 4;  // "help"
    for (size_t i = 0; i < count_; ++i) {
      const size_t n = length(cmds_[i].name);
      if (n > width) width = n;
    }
    for (size_t i = 0; i < count_; ++i) {
      tx_.print("  ");
      pad(cmds_[i].name, width + 2);
      tx_.println(cmds_[i].help);
    }
    tx_.print("  ");
    pad("help", width + 2);
    tx_.println("list commands, or 'help <command>'");
  }

  void print_usage(const Command& cmd) {
    tx_.print("usage: ");
    tx_.print(cmd.name);
    if (cmd.usage != nullptr) {
      tx_.put(' ');
      tx_.print(cmd.usage);
    }
    tx_.println();
  }

  void pad(const char* s, size_t width) {
    tx_.print(s);
    for (size_t n = length(s); n < width; ++n) tx_.put(' ');
  }

  static size_t length(const char* s) {
    size_t n = 0;
    while (s != nullptr && s[n] != '\0') ++n;
    return n;
  }

  bool names_equal(const char* a, const char* b) const {
    return ignore_case_ ? detail::iequals(a, b) : equals(a, b);
  }

  static bool equals(const char* a, const char* b) {
    for (;; ++a, ++b) {
      if (*a != *b) return false;
      if (*a == '\0') return true;
    }
  }

  const Command* find(const char* name) const {
    for (size_t i = 0; i < count_; ++i)
      if (names_equal(name, cmds_[i].name)) return &cmds_[i];
    return nullptr;
  }

  Stream& io_;
  const Command* cmds_;
  size_t count_;
  void* user_;
  TxBuffer tx_;
  LineEditor<MaxLine> editor_;
  const char* prompt_;
  bool ignore_case_;
};

}  // namespace mcu

#endif  // MCU_CONSOLE_HPP
