# xbee

Two independent pieces for XBee3 radios:

- **`xbee_config.hpp`**: configure a radio through its AT command mode (transparent mode, `AP=0`).
- **`xbee_api.hpp`**: the API-mode frame codec (`AP=1` / `AP=2`): build and parse frames, talk to the
  radio in structured frames. See [API mode](#api-mode-xbee_apihpp) below.

## Configuring a radio (xbee_config.hpp)

Configure an XBee3 radio from firmware through its AT command mode. You describe the settings you
want as a constant table; the library enters command mode, changes only what differs, applies,
verifies, saves to flash only if something changed, and leaves command mode. Non-blocking, no heap,
any microcontroller.

Header-only, C++11, no exceptions, no RTTI. Built on `at/` and `hal/`.

| File | What it is |
|---|---|
| `xbee_config.hpp` | `XBeeConfigurator`: profile apply, single get/set/command, entering command mode, baud detection |
| `xbee_api.hpp` | API-mode frames: encoder/parser with `AP=2` escaping, typed frame helpers, `XBeeApi` stream wrapper |

## Quick start

```cpp
#include "xbee/xbee_config.hpp"
#include "hal/ports/arduino_stream.hpp"

static const mcu::XBeeSetting kProfile[] = {
    mcu::xb_num("ID", 0x7FFF),      // network id
    mcu::xb_num("CH", 0x0C),        // channel
    mcu::xb_num("PL", 4),           // power level
    mcu::xb_text("NI", "Base"),     // node identifier
    mcu::xb_num("BD", 7),           // 115200 baud
};

mcu::ArduinoStream<HardwareSerial>      uart(Serial1);
mcu::ArduinoBaudControl<HardwareSerial> baud(Serial1);     // needed only if the profile sets BD
mcu::XBeeConfigurator<>                 xb(uart, &baud);

void setup() {
  Serial1.begin(9600);                                      // the radio's current rate (default 9600)
  xb.begin_profile(kProfile, sizeof kProfile / sizeof kProfile[0], millis());
}

void loop() {
  switch (xb.poll(millis())) {                              // never blocks; call every pass
    case mcu::XBeeStatus::kDone:
      // xb.changed_count() settings were different and have been saved
      break;
    case mcu::XBeeStatus::kFailed:
      // xb.error(), xb.failed_setting() (index in kProfile, or -1)
      break;
    default: break;                                         // kBusy: keep polling
  }
}
```

Values are written as the hex numbers the radio uses (`xb_num("ID", 0x7FFF)` sends `ATID7FFF`) and may
be up to 64 bits (`ID` on Zigbee). Text values (`NI`) may contain printable ASCII except `,`, which the
radio uses to chain commands; the table is checked up front and `begin_profile()` returns false for a
bad table, an unsupported `BD` value, or if a job is already running.

## What a profile does

1. **Enter command mode**: `+++` with a guard time of silence before and after (1 s by default,
   matching the radio's `GT`). Skipped if the radio is believed to be in command mode already.
2. **Read** every setting in the table and note the ones that differ.
3. **Write** only those, in table order, then **`ATAC`** to apply them.
4. If `BD` changed, **retune the host UART** through the `BaudControl`, then discard line noise.
5. **Verify**: read the changed settings back. A mismatch fails the job (`kVerifyFailed`) and nothing is saved.
6. **`ATWR`** to save to flash, **only if something changed**. A radio that is already configured is
   read and left alone, so running the profile at every boot does not wear its flash.
7. **`ATCN`** to leave command mode.

The order matters for `BD`: the new rate is applied, verified, and only then saved, so a rate your
UART cannot hold never ends up in flash. If verification fails after a baud change the radio keeps the
new rate in RAM until its next power cycle and the host is left at the same rate.

## Unknown baud rate

If the radio might be at any rate, give it a list and it will find the one that answers:

```cpp
static const uint32_t kRates[] = {9600, 115200, 57600};
xb.options().baud_candidates = kRates;
xb.options().baud_count = 3;
...
xb.detected_baud();      // 115200, once the job is done
```

Each rate gets `enter_attempts` tries (default 2), and each try costs about 3 s (1 s of quiet plus up to
2 s for the `OK`), so a list of three rates with nothing connected takes about 18 s to fail with
`kNoResponse`. This needs a `BaudControl`.

## Single operations

```cpp
xb.begin_enter(now);                       // just get into command mode
xb.begin_get("ID", now);        ...        // then xb.value()  (numeric)
xb.begin_get_text("NI", now);   ...        // then xb.text()
xb.begin_set("CH", 0x10, now);             // set only: not applied until AC, not saved until WR
xb.begin_set_text("NI", "Rover", now);
xb.begin_command("AC", now);               // also "WR", "CN", "RE", "FR"
```

Each returns false if a job is running or the arguments are bad, then you `poll()` as usual. These do
not enter command mode for you; call `begin_enter()` first (or run a profile). Reading the serial
number is `begin_get("SH")` and `begin_get("SL")`.

## Errors

| `XBeeError` | Meaning |
|---|---|
| `kNoResponse` | no `OK` after `+++`: wrong baud rate, not an XBee, or wrong guard time |
| `kTimeout` | a command was not answered. The radio may have left command mode (see below) |
| `kCommandError` | the radio answered `ERROR`: unknown or read-only setting, or value out of range. `failed_setting()` says which |
| `kBadReply` | an answer was not the hex number expected |
| `kVerifyFailed` | the radio stored something other than what was written |
| `kBaudChange` | the host UART refused the new rate |

A failed job never saves anything to flash after the point of failure.

## Options

`xb.options()` returns an `XBeeOptions` with these members (defaults in brackets):

| Option | Meaning |
|---|---|
| `escape` ["+++"] | the command sequence, if you changed `CC` |
| `guard_ms` [1000] | silence around the escape, if you changed `GT` |
| `reply_ms` [1000], `slow_ms` [3000] | wait for a normal answer; for `ATAC` and `ATWR` |
| `command_mode_ms` [10000] | the radio's `CT`: it leaves command mode after this much idle time |
| `enter_attempts` [2] | tries per baud rate |
| `enter`, `verify`, `write`, `exit` [all true] | switch off the matching profile step |
| `baud_candidates`, `baud_count` [none] | rates to try when entering |
| `baud_settle_ms` [100] | pause around a host baud change |

**Command-mode timeout.** The radio drops out of command mode after `CT` of silence. The library tracks
this (`in_command_mode(now)`) and enters again when needed. If you raise `command_mode_ms` above the
radio's real `CT`, a command sent after the radio has timed out gets `kTimeout`; the next job then
enters again.

## Notes and limits

- **One context, one job at a time.** Not interrupt safe.
- **Do not write to the UART yourself while a job runs**, and expect the radio's serial data to be
  swallowed while it is in command mode.
- **Transparent mode only.** `XBeeConfigurator` drives the AT command-mode interface (`AP=0`). For
  `AP=1`/`2` use `xbee_api.hpp`, which can also send AT commands (including to remote radios) as frames.
- Tested against a simulated radio that follows the Digi documentation for XBee3 (guard times, `CT`,
  `ATBD` taking effect on `ATAC`, flash persistence), not yet against real hardware. Timings of
  `+++` and flash writes on your firmware version are worth a check on a real module.
- **RAM:** about 400 bytes on a 32-bit MCU with the defaults (`XBeeConfigurator<24, 40, 40, 40>`). For
  small AVR parts use e.g. `XBeeConfigurator<8, 32, 32, 32>`.

## API mode (xbee_api.hpp)

In API mode every exchange is a frame: `0x7E`, a 2-byte length, the frame data (type byte first), and a
checksum (`0xFF` minus the sum of the data). With `AP=2` the bytes `0x7E 0x7D 0x11 0x13` inside a frame
are sent as `0x7D` followed by the byte XOR `0x20` (this applies to the length and checksum too).
The library has three layers; use as much as you need.

```cpp
#include "xbee/xbee_api.hpp"

mcu::ArduinoStream<HardwareSerial> uart(Serial1);
mcu::XBeeApi<> api(uart, mcu::XBeeApiMode::kApi2);          // must match the radio's AP

void setup() {
  Serial1.begin(115200);
  uint8_t id = api.send_at("NI");                           // query; the answer is a 0x88 frame
  static const uint8_t hello[] = {'h', 'i'};
  api.send_tx(0x0013A20041ABCDEFull, hello, 2);             // data to another radio
}

void loop() {
  if (api.poll()) {                                         // never blocks; call every pass
    if (api.frame_type() == mcu::xbee_frame::kRxPacket) {
      mcu::XBeeRxPacket rx = mcu::XBeeRxPacket();
      if (mcu::xbee_parse_rx(api.frame(), api.frame_length(), rx)) {
        // rx.src64, rx.data, rx.data_len
      }
    } else if (api.frame_type() == mcu::xbee_frame::kAtResponse) {
      mcu::XBeeAtResponse ar = mcu::XBeeAtResponse();
      if (mcu::xbee_parse_at_response(api.frame(), api.frame_length(), ar) &&
          ar.status == mcu::xbee_frame::kAtOk) {
        // ar.cmd, ar.data, ar.data_len
      }
    }
  }
}
```

The frame returned by `frame()` stays valid until the next `poll()`.

**1. Codec.** `xbee_encode(mode, data, len, out, cap)` wraps frame data into a wire frame (returns the
bytes written, or 0 if empty, over 65535 bytes, or `cap` is too small); `xbee_encoded_size` and
`xbee_max_encoded_size` size the buffer. `xbee_send(stream, mode, data, len)` writes a frame all or
nothing without a buffer for it: it checks `write_space()` first and returns `kNoSpace` (nothing
written, retry later), `kTooLong`, `kPartial` (the stream accepted less than it promised) or `kSent`.
`XBeeApiParser<MaxFrame>` finds frames in a byte stream with `poll(stream)` or `feed(buf, n)`; it counts
bad checksums, bad lengths and resyncs in `stats()`.

**2. Typed frames.** Builders write frame data and return its length (0 if it does not fit):
`xbee_build_at` (0x08, or 0x09 queued), `xbee_build_remote_at` (0x17), `xbee_build_tx` (0x10),
`xbee_build_explicit_tx` (0x11: endpoints, cluster and profile, for Zigbee), and the 802.15.4 frames
`xbee_build_tx64` (0x00) and `xbee_build_tx16` (0x01).
Parsers return views into the frame: `xbee_parse_at_response` (0x88), `xbee_parse_remote_at_response`
(0x97), `xbee_parse_rx` (0x90), `xbee_parse_explicit_rx` (0x91), `xbee_parse_rx802` (0x80 and 0x81, with
RSSI as the magnitude of the dBm value), `xbee_parse_tx_status` (0x89 and 0x8B), `xbee_parse_modem_status`
(0x8A). `XBeeApi` has `send_explicit`, `send_tx64` and `send_tx16` for the new builders. Serial-data
defaults for the explicit frames are in `xbee_frame` (`kSerialEndpoint` 0xE8, `kSerialCluster` 0x0011,
`kSerialProfile` 0xC105).
They reject frames that are too short, so any frame from `poll()` can be passed straight in. AT
parameters are raw bytes: `ATID 0x7FFF` is `{0x7F, 0xFF}`.

**3. XBeeApi.** The parser and sender bound to a `Stream`. Each `send_*` returns the frame id it used
(they cycle 1..255; 0 means "no reply wanted" and is never issued) or 0 if nothing was sent;
`last_send_result()` says why. `send_tx(..., want_status=false)` sets frame id 0 so no transmit status
comes back (it then returns 1 on success).

**Frames the typed helpers do not cover** (Zigbee-specific frames such as 0x92/0x95, anything newer) still pass through the codec: build the frame data yourself, send it with `send_frame()`,
and read it from `frame()` / `frame_length()`.

**Noise and recovery.** In `AP=2` an unescaped `0x7E` always starts a new frame, so a truncated frame is
dropped and the next one is read normally. In `AP=1` a `0x7E` can legitimately appear inside data, so after
a frame with a bad length or checksum the parser rescans the bytes it just read for a real start
delimiter instead of discarding them; a stray `0x7E` in line noise cannot swallow the real frame behind
it. A noise `0x7E` that claims a long length is still waiting for that many bytes before it is judged, so
recovery happens once enough data has arrived.

**Notes.**

- `MaxFrame` (default 128) is the longest frame data accepted; longer frames are dropped and counted as
  `bad_length`. `XBeeApi<MaxFrame, MaxOut>` also bounds the largest frame it will build (`MaxOut`, default
  96). RAM is about `MaxFrame + 40` bytes for the parser plus `MaxOut` on the stack while building.
- The radio's `AP` setting and the host's mode must match. `AP=2` is more robust on noisy links.
- Layouts follow Digi's XBee3 documentation for the 64-bit transmit/receive, AT, remote AT, transmit
  status and modem status frames, and for the explicit and 802.15.4 frames (0x11, 0x91, 0x00, 0x01, 0x80, 0x81;
  Digi marks the last four deprecated in favour of 0x10/0x90). Not yet checked against real hardware.

## Test

API codec (from this folder):

```
g++ -std=c++11 -Wall -Wextra -Werror -I.. test_xbee_api.cpp -o test_xbee_api && ./test_xbee_api
```

It checks the documented example frames, escaping and sizes for both modes, the parser fed one byte at a
time and in random chunks (by `feed` and `poll`), length limits, bad checksums, API1 rescan recovery and
API2 resynchronisation, all-or-nothing sending against streams that lie about their space, every builder
and parser including truncated input, `XBeeApi` frame-id cycling and a simulated request/response
exchange, and randomized noise and truncation.

Configurator (from this folder):

```
g++ -std=c++11 -Wall -Wextra -Werror -I.. test_xbee.cpp -o test_xbee && ./test_xbee
```

The test runs the library against a simulated XBee3 on a loopback link: guard times and the `CT`
timeout, queries and sets, `ERROR` replies, read-only settings, a radio that stores a different value
than it was given, baud changes (with the radio's `OK` sent at the old rate, lost, or followed by line
noise), a radio at an unknown baud rate, no radio at all, 64-bit values, counter wrap-around, and 3,000
randomized profiles checked against the radio's RAM, its flash, and its state after a power cycle.
