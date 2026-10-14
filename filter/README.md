# filter

Integer signal conditioning for firmware: smoothing filters, saturating math, fixed-point numbers and
switch debouncing. No floating point, no heap, no platform headers. Time is passed in, so any clock works
and tests can drive it by hand.

Header-only, C++11, no exceptions, no RTTI. `debounce.hpp` needs `hal/clock.hpp`.

| File | What it is |
|---|---|
| `fixed.hpp` | `sat_add`, `sat_sub`, `clamp`, `map_range`, `isqrt`, and the `Q<F>` fixed-point type |
| `filters.hpp` | `MovingAverage`, `Ema`, `MedianFilter`, `SlewLimiter`, `Schmitt`, `RunningStats` |
| `debounce.hpp` | `SampleDebouncer`, `Debouncer`, and `Button` (press, release, click, long press, multi-click) |

All filters work on any 8/16/32/64-bit integer type, take one sample per `update()` call and return the new
output. Rounding is to nearest, halves away from zero. Not interrupt safe: call from one context.

## Filters

```cpp
#include "filter/filters.hpp"

mcu::MovingAverage<uint16_t, 8> avg;          // mean of the last 8 samples
uint16_t smooth = avg.update(adc_read());

mcu::Ema<int16_t, 3> ema;                     // alpha = 1/8: cheap low-pass, one accumulator and no buffer
int16_t y = ema.update(sample);

mcu::MedianFilter<uint16_t, 5> med;           // removes single-sample spikes
uint16_t clean = med.update(adc_read());
```

| Type | Behaviour |
|---|---|
| `MovingAverage<T, N>` | Running sum, O(1) per sample. While filling, averages what it has, so the first output equals the first input. `full()`, `count()`, `sum()`, `reset()` |
| `Ema<T, Shift>` | `alpha = 1/2^Shift`, time constant about `2^Shift` samples. The first sample primes it. Converges **exactly** to a constant input from above or below and never overshoots (plain `y += (x-y)>>k` stalls short of the target). `reset(initial)` starts from a known value |
| `MedianFilter<T, N>` | Median of the last N, O(N) per sample, so keep N small (3 to 15). Even or partial windows return the lower middle value, so the output is always a real sample |
| `SlewLimiter<T>(max_up, max_down)` | Output moves toward the input by at most the limit per update. Limits are unsigned so full-range steps work (`int8_t` from -128 to 127 is a step of 255). First sample passes through |
| `Schmitt<T>(low, high)` | On at `>= high`, off at `<= low`, holds in between |
| `RunningStats<T>` | `count`, `min`, `max`, rounded `mean`, 64-bit `sum` since `reset()` |

`MovingAverage` and `Ema` use an accumulator type `Acc`, defaulting to a type wide enough for the sum
(`int32_t` for 8/16-bit samples, `int64_t` for 32-bit). Pass a narrower one as the last template argument
to save RAM and time on 8-bit parts when you know the range, for example `Ema<uint16_t, 4, uint32_t>`.
`Ema` checks at compile time that `Acc` can hold `T` plus `Shift` extra bits.

Choosing: **moving average** for steady noise, **EMA** when RAM is tight (one accumulator, no buffer),
**median** for occasional spikes (it ignores outliers instead of smearing them), and median followed by
EMA for both.

## Saturating math, range mapping, square root

```cpp
#include "filter/fixed.hpp"

uint8_t  a = mcu::sat_add<uint8_t>(200, 100);          // 255, not 44
int8_t   b = mcu::sat_sub<int8_t>(-100, 100);          // -128
int32_t  mv = mcu::map_range(adc, 0, 1023, 0, 3300);   // counts to millivolts, rounded and clamped
int32_t  inv = mcu::map_range(25, 0, 100, 255, 0);     // reversed output range is fine
uint32_t r = mcu::isqrt(1000000u);                     // 1000 (integer square root, no float)
```

`map_range` is Arduino's `map()` done properly: it rounds instead of truncating, clamps to the input range,
accepts reversed ranges, and uses a 64-bit intermediate so large spans cannot overflow. If the input range
is empty it returns `out_lo`. `isqrt64` handles 64-bit input.

## Fixed point: `Q<F>`

A signed 32-bit number with `F` fraction bits (`F` from 1 to 30). Resolution is `2^-F` and range about
`+/-2^(31-F)`: `Q<16>` covers +/-32768 with a step of 0.000015.

```cpp
typedef mcu::Q<16> Q16;

Q16 gain = Q16::from_ratio(3, 4);                 // 0.75
Q16 k    = Q16::from_double(0.8125);              // constants only; evaluated at compile time
Q16 y    = gain * Q16::from_int(100) + k;         // 75.8125
int32_t n = y.to_int();                           // 76 (rounded)
```

Arithmetic **saturates** instead of wrapping (`Q16::max_value() + one` stays at max), multiplication and
division round to nearest, and division by zero returns the largest or smallest value instead of crashing.
`raw()` / `from_raw()` give the underlying integer for storage or hardware registers. Comparisons, compound
assignment, unary minus and `abs()` are provided. `abs()` of the most negative value saturates.

Multiply and divide use 64-bit intermediates: fast on 32-bit cores, slower but correct on 8-bit ones.

## Debouncing

```cpp
#include "filter/debounce.hpp"

mcu::Button btn;                                   // active low (pull-up), 20 ms debounce, 800 ms long press

uint8_t ev = btn.update(digitalRead(PIN), clock.millis());
if (ev & mcu::Button::kClick)     { toggle(); }
if (ev & mcu::Button::kLongPress) { enter_menu(); }
```

### Button

`Button(active_low = true, debounce_ms = 20, long_press_ms = 800, multi_click_ms = 0)`. `update(raw_level, now)`
returns a bit mask, usually 0:

| Event | When |
|---|---|
| `kPress` | the button went down (after debouncing) |
| `kRelease` | the button came up |
| `kClick` | a press and release that was not a long press |
| `kLongPress` | held for `long_press_ms`; fires once, while still held, and the later release is **not** a click |

Also `down()`, `held_ms(now)`, `long_pressed()` and `clicks()`.

**Multi-click.** With `multi_click_ms = 0`, `kClick` is reported on release. With a window (say 300), clicks
closer together than the window are counted and `kClick` arrives once, 300 ms after the last release, with
`clicks()` set to the count: 1 is a click, 2 a double click, and so on. A press that turns into a long press
cancels clicks pending before it. The price of the window is that a single click is reported 300 ms late.
`long_press_ms = 0` disables long press.

### Debouncer and SampleDebouncer

Use these directly for switches, limit sensors and anything that is not a user button.

```cpp
mcu::Debouncer door(20, 50);            // must be steady 20 ms to turn on, 50 ms to turn off
door.update(raw, now);
if (door.rose()) { ... }  if (door.fell()) { ... }  uint32_t t = door.held_ms(now);

mcu::SampleDebouncer sw(5);             // call at a fixed rate (say every 1 ms): 5 agreeing samples to flip
sw.update(raw);
```

`Debouncer` is time-based: the output changes once the raw input has differed from it continuously for the
threshold, and any return to the current state cancels the change in progress. It can be called at irregular
intervals. `SampleDebouncer` is an integrator that counts toward 0 or the threshold, so it forgives isolated
glitches gradually; it needs a steady call rate. Both offer `state()`, `changed()`, `rose()`, `fell()`.

Time is the wrapping 32-bit millisecond count, and everything is wrap-safe.

## Test

From this folder:

```
g++ -std=c++11 -Wall -Wextra -Werror -I.. test_filter.cpp -o test_filter && ./test_filter
```

The tests cover saturating math (exhaustive for 8-bit, 200,000 random pairs for wider types, checked against
a 64-bit reference), `map_range` against a double reference, `isqrt` exhaustively for small inputs and by
property for random 32 and 64-bit inputs, `Q<F>` against 64-bit arithmetic, each filter against a naive
reference (moving average, median) or by convergence and no-overshoot properties (EMA, slew limiter),
and the three debouncers with explicit millisecond timelines (bounce, glitch rejection, long press,
multi-click, click-then-hold, active high, counter wrap-around) plus randomized property tests for the
debouncer and button.
