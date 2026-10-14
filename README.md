# mculib

A growing collection of microcontroller-agnostic C++ library files. The same code builds for AVR,
ARM, ESP32, RP2040 and a PC.

## Ground rules

- C++11, header-only where practical, so there is no build system to adopt.
- No heap, exceptions or RTTI. Buffers are sized by template parameters or supplied by the caller.
- No platform headers in the portable code. Hardware is reached through the small interfaces in
  `hal/`, and each platform gets a thin port class.
- Time is always passed in (or read through `mcu::Clock`), never fetched from a global.
- Every library comes with a README and a host-side test that runs on a PC.

## Libraries

| Folder | Contents |
|---|---|
| `ring_buffer/` | Fixed-capacity FIFO, lock-free for one producer plus one consumer (ISR and main loop) |
| `checksum/` | CRC-8/16/32 (14 presets, optional table speed-up), Fletcher-16/32, Xor8, Sum8, Lrc8 |
| `hal/` | `Stream`, `Clock`, `Timeout`, `BaudControl`, an in-memory `LoopbackStream`, Arduino port |
| `framing/` | COBS and SLIP packet framing with a pluggable CRC/checksum, over any `Stream` |
| `cli/` | Serial command console: line editor, tokenizer, strict number parsers, command table, help |
| `sched/` | Cooperative task scheduler (periodic and one-shot), plus Stopwatch, Throttle, Backoff, RetryTimer |
| `at/` | Non-blocking AT command engine (OK/ERROR, one-line answers, `+++` escape with guard time) and text helpers |
| `xbee/` | XBee3: configuration over AT command mode (settings profile, change only what differs, verify, save only if changed, baud detection) and the API-mode (`AP=1`/`2`) frame codec with typed frames |
| `filter/` | Moving average, EMA, median, slew limiter, Schmitt trigger, saturating math, `Q<F>` fixed point, button debounce with click / long-press events |

## Using it

Put this folder on your include path (`-I path/to/mculib`) and include headers by folder:

```cpp
#include "ring_buffer/ring_buffer.hpp"
#include "checksum/crc.hpp"
#include "hal/stream.hpp"
```

On Arduino or PlatformIO, copy the folders you need into the project's `lib/` or `src/` directory.
Note that `ring_buffer/` and `checksum/` tests include their own header directly, while `hal/`,
`framing/`, `cli/`, `sched/`, `filter/`, `at/` and `xbee/` tests use the root include path (`-I..`); each folder's README gives the exact compile line.
