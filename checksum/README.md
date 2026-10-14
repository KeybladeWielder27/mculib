# checksum

CRC-8/16/32, Fletcher-16/32 and simple XOR/sum checksums. Header-only, C++11, no heap,
no exceptions, no RTTI, no `<type_traits>`, no platform headers.

- `crc.hpp`: CRCs
- `checksum.hpp`: Fletcher-16, Fletcher-32, Xor8, Sum8, Lrc8

## One interface for everything

```cpp
#include "crc.hpp"
#include "checksum.hpp"

mcu::Crc16Modbus c;             // ready to use
c.update(byte);                 // one byte
c.update(buf, len);             // a buffer
uint16_t v = c.value();         // result so far; you can keep updating afterwards
c.reset();                      // start over
c.resume(v);                    // continue from a value returned earlier

uint32_t x = mcu::Crc32::compute(buf, len);   // one-shot
```

Every class has `value_type`, `reset`, `update` (both forms), `value` and a static `compute`,
so protocol code can be a template over the checksum type. `resume` is provided
everywhere except `Fletcher32` (its pending odd byte makes a single value ambiguous).

## CRC presets

Check value is the CRC of `"123456789"`; all are verified in the tests.

| Type | Catalogue name | Check |
|---|---|---|
| `Crc8` | CRC-8/SMBUS | 0xF4 |
| `Crc8Maxim` | CRC-8/MAXIM-DOW (1-Wire) | 0xA1 |
| `Crc8SaeJ1850` | CRC-8/SAE-J1850 | 0x4B |
| `Crc16Xmodem` | CRC-16/XMODEM | 0x31C3 |
| `Crc16CcittFalse` | CRC-16/IBM-3740 | 0x29B1 |
| `Crc16Kermit` | CRC-16/KERMIT | 0x2189 |
| `Crc16X25` | CRC-16/X-25 (HDLC) | 0x906E |
| `Crc16Arc` | CRC-16/ARC | 0xBB3D |
| `Crc16Modbus` | CRC-16/MODBUS | 0x4B37 |
| `Crc16Usb` | CRC-16/USB | 0xB4C8 |
| `Crc32` | CRC-32/ISO-HDLC (zip, png, ethernet) | 0xCBF43926 |
| `Crc32c` | CRC-32C (Castagnoli) | 0xE3069283 |
| `Crc32Mpeg2` | CRC-32/MPEG-2 | 0x0376E6E7 |
| `Crc32Bzip2` | CRC-32/BZIP2 | 0xFC891918 |

### Your own CRC

```cpp
//                     width    poly    init    reflect xorout
typedef mcu::Crc<uint16_t, 0x1021, 0x0000, false, 0x0000> MyCrc;
```

Width comes from the type (`uint8_t`, `uint16_t`, `uint32_t`). Parameters use the usual
catalogue convention (reveng / Rocksoft). Input and output reflection are assumed equal,
which covers nearly every CRC in use; odd widths like CRC-12 are not supported.

### Speed vs size

- Default: bitwise, no table, smallest code, slowest.
- `Name::Fast`: 16-entry nibble table (16 x sizeof(value) bytes), roughly 4x faster.
  The table is built at compile time, so there is no startup cost and no guard variable.
  On AVR, constant data occupies RAM unless moved to PROGMEM, so prefer the default there.

```cpp
uint32_t v = mcu::Crc32::Fast::compute(buf, len);   // same result, faster
```

## Fletcher and simple sums

| Type | Result | Notes |
|---|---|---|
| `Fletcher16` | 16 bit | bytes, mod 255. "abcde" -> 0xC8F0 |
| `Fletcher32` | 32 bit | little-endian 16-bit words, mod 65535, odd tail zero-padded. "abcde" -> 0xF04FC729 |
| `Xor8` | 8 bit | XOR of all bytes (NMEA style) |
| `Sum8` | 8 bit | sum mod 256 |
| `Lrc8` | 8 bit | two's complement of the sum (Modbus ASCII LRC, Intel HEX) |

The Fletcher sums use a conditional subtract instead of `%`, so there is no division on
small cores. They are weaker than a CRC: Fletcher-16 cannot tell 0x00 from 0xFF.

## Test

```
g++ -std=c++11 -Wall -Wextra -Werror -I. test_checksum.cpp -o test_checksum && ./test_checksum
```

The tests check every preset against its published check value and against an independent
textbook CRC implementation on random data, plus streaming at every split point, `resume`,
and the nibble-table variants.
