# at

A non-blocking AT command engine. It sends text commands to a serial device and collects the text
lines that come back. It knows nothing about any particular device, so the same engine drives XBee
radios in command mode, ESP32/ESP8266 AT firmware, cellular modems, BLE modules and GPS receivers.
Device drivers (such as the planned `xbee/`) are built on top of it.

Header-only, C++11, no heap, no exceptions, no RTTI. Needs `hal/stream.hpp` and `hal/clock.hpp`.

| File | What it is |
|---|---|
| `at_client.hpp` | `AtClient<CmdN, LineN, RespN>`: send a command, poll until OK / ERROR / answer / timeout |
| `at_text.hpp` | `TextBuilder<N>` to compose commands without printf; `parse_hex_u32/u64`, `parse_dec_u32` for answers |

## Quick start

```cpp
#include "at/at_client.hpp"
#include "hal/ports/arduino_stream.hpp"

mcu::ArduinoStream<HardwareSerial> uart(Serial1);
mcu::AtClient<> at(uart);

at.command("AT+GMR", millis());                 // returns at once

// in loop():
switch (at.poll(millis())) {
  case mcu::AtStatus::kBusy:    break;                              // still waiting
  case mcu::AtStatus::kOk:      show(at.response()); break;         // info lines, joined with '\n'
  case mcu::AtStatus::kError:   show(at.final_line()); break;       // "ERROR", "+CME ERROR: 10", ...
  case mcu::AtStatus::kTimeout: retry(); break;
  default: break;
}
```

`poll()` never blocks. Call it often, also when no command is running, so that unsolicited lines are
delivered. A finished result stays in `status()` until the next command starts.

## How a command finishes

Devices answer in different shapes, so each command says what to wait for:

| `AtExpect` | Finishes when | Typical use |
|---|---|---|
| `kFinal` (default) | a line `OK`, `ERROR`, `+CME ERROR: ...` or `+CMS ERROR: ...` arrives; the lines before it go into `response()` | modems; XBee commands that set something (`ATID7FFF` answers `OK`) |
| `kLines` | `n` answer lines have arrived (default 1); `ERROR` still fails | XBee queries: `ATID` answers `7FFF` with no OK after it |
| `kNone` | the text has been sent | commands that reboot the device or answer nothing |

```cpp
at.command("ATID", now, 1000, mcu::AtExpect::kLines);           // -> response() "7FFF"
at.command("ATID,CH,PL", now, 1000, mcu::AtExpect::kLines, 3);  // -> "7FFF\nC\n4"

uint32_t pan;
mcu::parse_hex_u32(at.response(), pan);                          // 0x7FFF
```

Lines may end in CR, LF or CR LF, and empty lines are skipped. A line identical to the command just
sent is taken as an echo and ignored (`set_echo_filter(false)` turns that off). `line_count()` says how
many info or answer lines arrived.

## Escape sequences: `raw()`

Transparent-mode radios and modems switch to command mode when they see a pause, `+++`, then another
pause. `raw()` handles that:

```cpp
at.raw("+++", now, 1100, 2000);   // 1.1 s of silence first, then "+++" with no CR,
                                  // then wait up to 2 s for "OK"
```

- Nothing is sent during the quiet time, so do not write to the stream from elsewhere meanwhile.
- The escape is written in one piece when the stream reports enough `write_space()`, so it is not
  split across a full transmit buffer.
- In `raw()` mode `OK` and `ERROR` are matched **at the end of a line**. Data from the far end can
  still be arriving right up to the OK with no line break in between (`hello worldOK`), and this
  still counts as OK. Over-long lines keep their last characters for the same reason.
- The timeout is counted from when the text has gone out, not from the call.

## Composing commands

`TextBuilder<N>` appends into a fixed buffer. If anything does not fit it is dropped and `ok()` turns
false, so one check after building is enough.

```cpp
mcu::TextBuilder<24> cmd;
cmd.add("ATCH").add_hex(0x0C, 2);       // "ATCH0C"
cmd.add(',').add("NI").add("Base");      // "ATCH0C,NIBase"
if (cmd.ok()) at.command(cmd.c_str(), now);
```

`add_hex(v, min_digits)` writes upper-case hex with no prefix, which is what AT devices expect.
`add_dec()` takes signed or unsigned values. The parsers are strict: `parse_hex_u32("7FFF")` succeeds,
while `"0x7FFF"`, `" 7FFF"` and `"ERROR"` fail and leave the output untouched.

## Other calls

| Call | Notes |
|---|---|
| `cancel()` | abandons the running command; a late answer then arrives as unsolicited lines |
| `discard_input()` | drops buffered input and any partial line. Use it before taking over a port that was carrying data, or leftover bytes will glue onto the first answer |
| `set_unsolicited_handler(fn, ctx)` | called for each line that arrives while no command waits for an answer (`RING`, `+IPD,...`) |
| `set_info_handler(fn, ctx)` | called for each info line of a running command; useful for listings longer than `RespN` |
| `response_truncated()`, `line_overflows()` | something did not fit in `RespN` or `LineN` |

## Sizing

`AtClient<CmdN = 64, LineN = 64, RespN = 128>` holds three buffers of those sizes plus about 50 bytes
of state, so about 350 bytes on a 32-bit MCU with the defaults. On a small AVR, an XBee driver is comfortable with
`AtClient<32, 32, 48>`. `final_line()` keeps the first 23 characters.

## Rules

- **One context.** Not interrupt safe; call it from the main loop or one task.
- **One command at a time.** `command()` and `raw()` return false while a command is running, and
  also if the text does not fit in `CmdN` (with its terminator).
- Timeouts restart whenever transmit makes progress, so a long command on a slow, nearly full port
  does not time out while it is still being sent.
- Time is the wrapping 32-bit millisecond counter; everything is wrap-safe.

## Test

From this folder:

```
g++ -std=c++11 -Wall -Wextra -Werror -I.. test_at.cpp -o test_at && ./test_at
```

The test drives the client against a simulated device over a `LoopbackStream` with adjustable wire
speed. It covers every answer shape, echoes, `+CME`/`+CMS` errors, timeouts (also across counter
wrap-around), partial writes through an 8-byte transmit buffer, escape-sequence quiet time and
end-of-line matching, unsolicited lines, buffer overflow, and the text helpers. It also runs 4,000
randomized answers with random line endings and chunking against an expected result, and a 300,000-step
random-garbage run for memory safety under ASAN/UBSAN.
