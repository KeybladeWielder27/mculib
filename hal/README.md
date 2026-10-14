# hal

The thin layer between the portable libraries and the hardware: a byte `Stream`, a millisecond
`Clock`, and a `Timeout`. Portable code only ever sees these, so moving to a new chip means
writing one small port class, not touching the libraries.

Header-only, C++11, no heap, no exceptions, no RTTI.

| File | What it is |
|---|---|
| `stream.hpp` | `mcu::Stream`: non-blocking `read`, `write`, `available`, `write_space` |
| `clock.hpp` | `mcu::Clock`, `FunctionClock`, `ManualClock`, `Timeout`, wrap-safe time helpers |
| `loopback_stream.hpp` | `LoopbackStream<RxN, TxN>`: in-memory stream for tests and simulation |
| `ports/arduino_stream.hpp` | `ArduinoStream<S>`: wraps `Serial`, `Serial1`, `SoftwareSerial`, USB CDC, ... |

## Stream

```cpp
size_t read(uint8_t* dst, size_t max);          // bytes actually read, may be 0
size_t write(const uint8_t* src, size_t len);   // bytes actually accepted, may be 0
size_t available() const;                       // readable now
size_t write_space() const;                     // writable now, or kUnboundedSpace if unknown
bool   read_byte(uint8_t& out);                 // helpers
bool   write_byte(uint8_t b);
```

Nothing blocks and partial transfers are normal. `write_space()` exists so a framer can check that a
whole frame fits before sending any of it, instead of leaving a half-sent frame on the wire.

Writing a port for a new platform is four small functions. For example, ESP-IDF UART:

```cpp
class EspUart final : public mcu::Stream {
  size_t read(uint8_t* d, size_t n) override { /* uart_read_bytes(..., 0 ticks) */ }
  size_t write(const uint8_t* s, size_t n) override { /* uart_tx_chip_fifo / ring */ }
  size_t available() const override { /* uart_get_buffered_data_len */ }
  size_t write_space() const override { /* free TX ring space */ }
};
```

### Arduino

```cpp
#include "hal/ports/arduino_stream.hpp"           // does not include Arduino.h

mcu::ArduinoStream<HardwareSerial> uart(Serial1);
```

By default writes are limited to `availableForWrite()` so they never block. On objects where that
method just returns 0, construct with `false` (`ArduinoStream<S> s(obj, false)`): writes then pass
straight through and may block, and `write_space()` reports `kUnboundedSpace`.

## Clock and Timeout

Libraries never call `millis()` directly. They take a `const mcu::Clock&`, or just a `uint32_t now`.

```cpp
mcu::FunctionClock clock(millis);            // Arduino
mcu::FunctionClock clock(HAL_GetTick);       // STM32 HAL
// ESP-IDF: static uint32_t ms() { return (uint32_t)(esp_timer_get_time() / 1000); }
//          mcu::FunctionClock clock(ms);

mcu::Timeout t;
t.start(clock.millis(), 500);
if (t.expired(clock.millis())) { /* ... */ }

mcu::Timeout tick;                            // periodic, drift-free
tick.start(clock.millis(), 1000);
if (tick.poll_periodic(clock.millis())) { /* once a second */ }
```

The counter is an unsigned 32-bit millisecond value that wraps every 49.7 days. All comparisons are
wrap-safe for intervals up to about 24.8 days. `ManualClock` lets a PC test move time by hand.

## LoopbackStream

```cpp
mcu::LoopbackStream<64, 64> port;
port.inject(bytes, n);          // pretend these arrived on the wire
code_under_test.poll(port);     // sees a normal mcu::Stream
size_t got = port.take(out, sizeof out);   // what it sent
```

## Design notes

- Abstract base classes (one vtable pointer per object) rather than templates, so a library compiles
  once and works with any port. The base destructors are protected and non-virtual: nothing is ever
  deleted through a base pointer, and this keeps `operator delete` out of small builds. Concrete
  classes are `final`.
- Include path: put the root of the collection (the folder containing `hal/`, `ring_buffer/`,
  `checksum/`) on the include path. Headers include each other as `"hal/stream.hpp"`.

## Test

From this folder:

```
g++ -std=c++11 -Wall -Wextra -Werror -I.. test_hal.cpp -o test_hal && ./test_hal
```
