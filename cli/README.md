# cli

A serial command console for any microcontroller: line editing, quoting, a command table, built-in `help`,
strict number parsing, and output that never blocks. It talks to any `mcu::Stream`.

Header-only, C++11, no heap, no exceptions, no RTTI, no `printf`. Needs `hal/` and `ring_buffer/`.

| File | What it is |
|---|---|
| `console.hpp` | `Console`: the whole thing, plus `Command` and `CommandContext` |
| `line_editor.hpp` | `LineEditor`: bytes in, finished lines out (backspace, Ctrl-U/C, arrow-key filtering) |
| `tokenizer.hpp` | `tokenize()`: split a line into arguments in place, with quotes and escapes |
| `parse.hpp` | `parse_uint`, `parse_int`, `parse_bool`, `parse_scaled`, `parse_hex_bytes` |
| `output.hpp` | `Output` print helpers, `StreamOutput`, `BufferOutput` |

Each layer works on its own. Use just `parse.hpp` or `tokenizer.hpp` without the console if you like.

## Quick start

```cpp
#include "cli/console.hpp"

static int cmd_add(mcu::CommandContext& c) {
  int32_t a, b;
  if (!c.get_int(1, a) || !c.get_int(2, b)) return mcu::kCmdReported;   // already printed why
  c.out.print_int(a + b);
  c.out.println();
  return mcu::kCmdOk;
}

static int cmd_led(mcu::CommandContext& c) {
  bool on;
  if (!c.get_bool(1, on)) return mcu::kCmdReported;
  set_led(on);
  return mcu::kCmdOk;
}

static const mcu::Command kCommands[] = {
  // name   handler   help text          usage       min max args
  {"add",  cmd_add, "add two numbers", "<a> <b>",   2, 2},
  {"led",  cmd_led, "switch the LED",  "<on|off>",  1, 1},
};

mcu::ArduinoStream<HardwareSerial> port(Serial);
mcu::Console<64, 8, 256> console(port, kCommands, sizeof kCommands / sizeof kCommands[0]);

void setup() { Serial.begin(115200); console.begin(); }
void loop()  { console.poll(); /* ...the rest of your firmware... */ }
```

```
> help
  add   add two numbers
  led   switch the LED
  help  list commands, or 'help <command>'
> add 2 0x10
18
> led maybe
error: bad argument 1: maybe
> add 1
usage: add <a> <b>
```

Template arguments of `Console<MaxLine, MaxArgs, TxSize>`: longest line, most arguments (counting the command
name), and output queue size in bytes (a power of two).

## Commands

`Command` fields: `name`, `handler`, `help` (one line), `usage` (arguments text, may be `nullptr`),
`min_args` and `max_args` (not counting the name; `mcu::kAnyArgs` for no limit). The console checks the argument
count and prints the usage line for you.

Handlers get a `CommandContext`:

| Member | |
|---|---|
| `argc`, `argv` | `argv[0]` is the command name; `argv[argc]` is `nullptr` |
| `out` | print results here |
| `user` | the pointer you gave the console |
| `get_uint(i, v, max)`, `get_int(i, v, min, max)`, `get_bool(i, v)`, `get_scaled(i, v, decimals)` | typed arguments; on a bad value they print `error: bad argument N: text` and return `false` |

Return values: `kCmdOk` (0), `kCmdUsage` (console prints the usage line), `kCmdReported` (failed, handler already
said why), anything else prints `error <code>`.

Command tables are ordinary const arrays. On AVR they sit in RAM unless you place them in flash yourself.

## Output helpers

`put`, `print`, `println`, `print_uint(v, base, min_digits)`, `print_int`, `print_hex(v, digits)`,
`print_fixed(scaled, decimals)` and `print_bytes_hex`. No `printf`, so no floating-point or large-library cost.

`print_fixed` and `parse_scaled` work as a pair for decimal values without floats: `parse_scaled("3.30", v, 3)` gives
`3300`, and `print_fixed(3300, 3)` prints `3.300`.

## Parsing rules

All parsers are strict: no trailing junk, no empty input, overflow and range checked, output untouched on failure.

| | Accepts |
|---|---|
| `parse_uint` | decimal, `0x` hex, `0b` binary. `007` is 7. No sign |
| `parse_int` | optional `+` or `-`, then the same. Full int32 range, including `-2147483648` |
| `parse_bool` | `1 0 on off true false yes no y n`, any letter case |
| `parse_scaled` | `3.14`, `5`, `.25`, `-0.5` as an integer of 10^-decimals units. Extra digits beyond `decimals` are rejected, not rounded |
| `parse_hex_bytes` | an even number of hex digits, no prefix or separators |

## Tokenizing rules

Close to a Unix shell: space and tab separate arguments; `"double quotes"` group text and understand `\" \\ \n \t \r`;
`'single quotes'` group literally; a backslash outside quotes makes the next character literal (`a\ b` is one
argument); adjacent pieces join (`ab"c d"e` is `abc de`); `""` is an empty argument.

## Behavior worth knowing

- **Never blocks.** `poll()` sends queued output as far as the stream accepts it, reads input, and runs at most one
  command per call, so your main loop stays responsive.
- **Back-pressure.** While output is still waiting for the stream, the console reads no new input. A slow link slows
  the console down rather than losing text, and a long reply cannot be overtaken by the next command.
- **Output queue.** A command's output goes into a queue of `TxSize` bytes. If one command prints more than that
  while the stream is full, the excess is dropped and counted in `dropped_output()`. Size it for your longest reply.
  `help` with many commands is the usual biggest one.
- **Lines that are too long** are never run truncated: the terminal gets a bell for each refused character,
  Backspace takes them back, and pressing Enter on a line that is still too long prints `error: line too long`.
- **Arrow keys, Home, Delete** and other escape sequences are swallowed, so they do not insert junk. There is no
  history or cursor movement.
- `console.out()` prints banners and messages in order with command output, but may land in the middle of a line
  the user is typing.
- Options: `set_prompt()`, `set_echo(false)` for terminals that echo locally, `set_ignore_case(true)`, `set_user()`.
  A command named `help` in your table replaces the built-in.

## Test

From this folder:

```
g++ -std=c++11 -Wall -Wextra -Werror -I.. test_cli.cpp -o test_cli && ./test_cli
```

Covers the tokenizer (including random-input fuzzing), every parser at its limits, the output helpers, every key
the line editor handles, and the console end to end over a `LoopbackStream`: errors, quoting, help, case
folding, back-pressure with a stream that holds 16 bytes, output overflow accounting, and a stream that
accepts less than it promised.
