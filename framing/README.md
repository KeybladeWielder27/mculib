# framing

Packet framing for serial links: **COBS** or **SLIP**, with an optional **CRC / checksum**, behind one
interface. Swap the codec by changing one word.

Header-only, C++11, no heap, no exceptions, no RTTI. Depends on `hal/stream.hpp` and, if you use a check,
`checksum/`.

| File | What it is |
|---|---|
| `codec.hpp` | `mcu::Cobs`, `mcu::Slip`: encode and byte-at-a-time decode |
| `framer.hpp` | `FrameEncoder`, `FrameDecoder`, `NoCheck`, `FrameResult`, `FrameStats` |

## Use

```cpp
#include "checksum/crc.hpp"
#include "framing/framer.hpp"

//                              codec      check             max payload, check byte order
typedef mcu::FrameEncoder<mcu::Cobs, mcu::Crc16Modbus, 64, true> Enc;
typedef mcu::FrameDecoder<mcu::Cobs, mcu::Crc16Modbus, 64, true> Dec;

Dec dec;

// send: goes straight to the stream, needs no buffer
if (!Enc::send(port, payload, len)) { /* does not fit in the TX buffer right now: retry later */ }

// receive: call from your main loop
for (;;) {
  mcu::FrameResult r = dec.poll(port);          // port is any mcu::Stream
  if (r == mcu::FrameResult::kNoFrame) break;   // nothing more right now
  if (r == mcu::FrameResult::kFrame) handle(dec.data(), dec.size());
  // other results are errors, already counted in dec.stats()
}
```

To use SLIP instead, replace `mcu::Cobs` with `mcu::Slip` in both typedefs. Nothing else changes.

Frame on the wire: `[encoded(payload + check)][delimiter]`.

## Template arguments

| Argument | Meaning |
|---|---|
| `Codec` | `mcu::Cobs` or `mcu::Slip` |
| `Check` | any CRC from `crc.hpp` (including `::Fast`), `Fletcher16/32`, `Xor8`, `Sum8`, `Lrc8`, or `mcu::NoCheck` |
| `MaxPayload` | largest payload in bytes, not counting the check |
| `LittleEndian` | check byte order on the wire, default `false` (most significant byte first). Modbus CRC-16 is `true` |

Both sides must use the same arguments.

## Encoder (all static)

| Call | Notes |
|---|---|
| `send(stream, p, len, leading=false)` | encodes directly into the stream. Returns `false`, having written nothing, if the stream has no room for the whole frame (it checks `write_space()`), or the payload is not sendable. Also returns `false` if the stream stops accepting bytes partway |
| `encode(p, len, out, cap, leading=false)` | into your own buffer; returns the frame size, or 0 |
| `frame_size(p, len, leading=false)` | exact encoded size, or 0 |
| `max_frame_size()` | compile-time upper bound, for sizing a buffer |

`leading` sends a delimiter before the frame. It costs one byte and makes the receiver throw away any
noise that arrived earlier, which mostly matters for SLIP after power-up or on a noisy link.

## Decoder

| Call | Notes |
|---|---|
| `push(byte)` | feed one wire byte; returns a `FrameResult` |
| `poll(stream)` | reads bytes until an event happens or the stream is empty. Returns at the first event, so call it in a loop |
| `data()`, `size()` | the payload after `kFrame` (check bytes removed). Valid until the next `push` or `poll` |
| `stats()` | counters: `ok`, `bad_check`, `bad_encoding`, `overflow` |
| `reset()`, `clear_stats()` | |

Results: `kNoFrame` (keep feeding), `kFrame`, `kBadCheck` (failed the check, or too short to hold one),
`kBadEncoding` (truncated frame or bad escape), `kOverflow` (longer than `MaxPayload`). A bad frame is
dropped and the decoder is ready for the next one immediately.

RAM: a buffer of `MaxPayload` plus the check bytes, plus a few dozen bytes of state (46 bytes on a 64-bit PC,
less on an MCU).

## COBS or SLIP?

| | COBS | SLIP |
|---|---|---|
| Delimiter | `0x00` | `0xC0` |
| Overhead | 1 byte per 254, worst case. Predictable | 0 for most data, up to 2x for data full of `0xC0`/`0xDB` |
| Resync after garbage | always at the next `0x00` | at the next `0xC0` |
| Zero-length frame | allowed | not representable (refused by the encoder), unless a check is present |
| Good for | binary data, fixed buffers, UART links | text-like data, compatibility with existing SLIP peers |

Choose COBS for binary payloads where worst-case size matters. Choose SLIP when a peer already speaks it.

## Notes

- The check is computed over the payload and sent as part of the frame, so it is protected by the same
  encoding as the data.
- A corrupted frame is rejected by the check, not repaired. Both codecs resynchronise at the next delimiter,
  so one bad frame never takes down the next.
- `poll()` reads one byte per call to `Stream::read`. That is fine for UART rates; for very high rates,
  feed whole buffers to `push()` yourself.

## Test

From this folder:

```
g++ -std=c++11 -Wall -Wextra -Werror -I.. test_framer.cpp -o test_framer && ./test_framer
```

The tests cover the published COBS test vectors and RFC 1055 SLIP cases, round trips for 22 codec-and-check
combinations at every payload length, the 254/255-byte COBS block boundaries, every single-bit corruption of a
frame, resync after garbage, overflow, check byte order, and sending over a `Stream` with back-pressure.
