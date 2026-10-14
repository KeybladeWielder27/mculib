# ring_buffer

Fixed-capacity FIFO ring buffer. Header-only, C++11, no heap, no exceptions,
no RTTI, no platform headers. Works the same on AVR, ARM, ESP32, RP2040, or a PC.

## Use

```cpp
#include "ring_buffer.hpp"

mcu::RingBuffer<uint8_t, 64> rx;   // capacity must be a power of two

// producer (e.g. UART RX interrupt)
rx.push(byte);                     // false if full (item dropped)

// consumer (e.g. main loop)
uint8_t b;
while (rx.pop(b)) { /* handle b */ }
```

## API

| Call | Side | Notes |
|---|---|---|
| `push(item)` | producer | returns `false` if full |
| `write(ptr, n)` | producer | writes as many as fit, returns count |
| `pop(out)` | consumer | returns `false` if empty |
| `peek(out)` | consumer | look at oldest item without removing it |
| `read(ptr, n)` | consumer | reads up to `n`, returns count |
| `discard(n)` | consumer | drops up to `n` oldest items |
| `clear()` | consumer | empties the buffer |
| `size()`, `free_space()`, `empty()`, `full()`, `capacity()` | either | |

## Thread / interrupt safety

Lock-free for **one producer and one consumer** (for example an ISR and the
main loop), as long as an index read or write is atomic on your target.

- The index type is picked automatically: 8-bit for capacity up to 128,
  16-bit up to 32768, otherwise 32-bit.
- On an 8-bit MCU, capacities of 128 or less are safe as is. For a larger
  capacity on an 8-bit core, wrap calls in a critical section.
- With several producers or several consumers, protect the buffer yourself.

## Design notes

- Indices run freely and are masked on access, so all N slots are usable and
  full/empty need no wasted slot or extra flag.
- `volatile` indices plus a compiler barrier make the data visible before the
  index moves. This is correct for single-core MCUs. A multi-core target
  (dual-core ESP32, RP2040 with both cores) would also need hardware memory
  barriers, which is not covered here.

## Test

```
g++ -std=c++11 -Wall -Wextra -Werror -I. test_ring_buffer.cpp -o test_ring_buffer && ./test_ring_buffer
```
