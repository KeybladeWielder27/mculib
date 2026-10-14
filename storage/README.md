# storage

A settings store for raw flash: keys are 16-bit numbers, values are byte strings. Survives power cuts at
any moment, spreads wear over several sectors, and writes nothing when a value has not changed. No heap,
any microcontroller, any flash you can wrap in a few functions.

Header-only, C++11, no exceptions, no RTTI. Built on `hal/flash.hpp` and `checksum/`.

| File | What it is |
|---|---|
| `kv_store.hpp` | `KvStore<MaxSectors>`: put / get / remove, typed helpers, key iteration |
| `../hal/flash.hpp` | `mcu::Flash`: the small interface you implement for your chip |
| `../hal/ram_flash.hpp` | `RamFlash<SectorSize, Sectors, Unit>`: simulated flash for PC tests, with power-cut injection |

## Quick start

```cpp
#include "storage/kv_store.hpp"

enum : uint16_t { kBaud = 1, kName = 2, kChannel = 3, kCalibration = 4 };

MyFlash flash;                    // your class deriving from mcu::Flash (see below)
mcu::KvStore<4> settings(flash, /*first_sector=*/60, /*sector_count=*/4);

void setup() {
  if (settings.begin() != mcu::KvResult::kOk) {
    settings.format();            // unreadable flash: start empty
  }
  uint32_t baud = settings.get_u32_or(kBaud, 115200);   // default if never saved
  char name[16];
  if (settings.get_str(kName, name, sizeof name) != mcu::KvResult::kOk) strcpy(name, "node");
}

void save() {
  settings.put_u32(kBaud, 57600);       // writes only if the value changed
  settings.put_str(kName, "Base-1");
  settings.put(kCalibration, &calib, sizeof calib);   // any bytes
  settings.remove(kChannel);
}
```

`begin()` must be called once at start-up (it can be called again at any time). Every other call returns a
`KvResult`: `kOk`, `kNotFound`, `kFull`, `kTooBig`, `kBadArg`, `kBufferTooSmall` (the length is still
reported), `kNotReady`, `kIoError`, `kCorrupt`.

## API

```cpp
KvResult begin();                 // mount; formats a blank flash; repairs after a power cut
KvResult format();                // erase everything
KvResult put(key, value, len);    // len 0 is a valid (empty) value
KvResult get(key, buf, cap, &len);
KvResult length(key, &len);       bool contains(key);
KvResult remove(key);
KvResult first_key(k);  KvResult next_key(k);       // ascending iteration
put_u8/u16/u32/i32, put_str      // fixed-size integers are stored little-endian
get_u8/u16/u32/i32, get_str, get_u8_or/get_u16_or/get_u32_or/get_i32_or(key, default)
size_t max_value_size();  const KvStats& stats();   // writes, skipped writes, sector opens, compactions
```

Keys are `0..0xFFFE`. A typed `get_*` returns `kBadArg` (and the `_or` forms return the default) when the
stored value has another size, so a layout change in a new firmware version falls back to defaults instead
of reading garbage.

## Implementing the flash port

```cpp
class MyFlash final : public mcu::Flash {
 public:
  size_t sector_count() const override { return 64; }         // the sectors the store may be given
  size_t sector_size()  const override { return 2048; }
  size_t program_unit() const override { return 8; }          // 1, 2, 4, 8 or 16 bytes
  bool read(size_t s, size_t off, void* dst, size_t n) override {
    memcpy(dst, (const void*)(kBase + s * 2048 + off), n);    // memory-mapped, or a SPI read
    return true;
  }
  bool program(size_t s, size_t off, const void* src, size_t n) override {
    return vendor_flash_write(kBase + s * 2048 + off, src, n) == OK;
  }
  bool erase(size_t s) override { return vendor_flash_erase(kBase + s * 2048) == OK; }
};
```

The rules of the interface: erased bytes read `0xFF`; `program` is given offsets and lengths that are
multiples of `program_unit()` and targets erased units only; each unit is written at most once between
erases (so ECC flash works); `read` is byte-granular. Sector numbers belong to you: the store uses the
window `[first_sector, first_sector + sector_count)` you pass to its constructor and never touches
anything else. On a part where reads stall during erase or program (internal flash on some MCUs), do the
work with interrupts and code placement your vendor requires inside `program` and `erase`.

## How it works

The store is a log. `put` appends a record (key, length, value, CRC-16) to the active sector, and the newest
record for a key wins. `remove` appends a tombstone. A power cut can only tear the record being written;
the CRC rejects it, so after a cut every key holds either the value it had before the interrupted call or
the new one, never a mixture.

When the active sector is full, the store moves on to the next erased sector, going round all the sectors
in turn, so erases are spread evenly. It keeps one erased sector in reserve. When only the reserve is
left, the oldest sector is **compacted**: its live records are copied into the reserve sector, that sector
is marked complete, and only then is the old one erased. Interrupted at any point, the next `begin()` either
discards the half-built copy (the old sector was never touched) or erases the old sector (the copy is
complete), even when the erase itself was torn halfway. Compaction costs one erase per sector's worth of
writes, and is the only time live data is moved.

`put` compares with the stored value first and writes nothing if it is identical (`stats().skipped_writes`),
so saving all your settings at every boot costs no wear.

## Limits and costs

- **RAM:** about 70 bytes plus 6 per sector (`MaxSectors` sets the array size; the default is 16). Stack use
  inside calls is about 100 bytes, plus 16 per sector in `begin()`. No index is kept in RAM.
- **Speed:** `get` and `put` read through the log, so their cost grows with the amount of flash in use
  (a CRC pass over each record). Right for settings, not for logging data at high rate. Mounting reads the
  whole region once.
- **Capacity:** roughly `(sectors - 1) x payload`, where payload is the sector size minus a small header.
  Updating a key needs room for the old and the new record at once, and every `put` keeps room for one
  tombstone so a full store can always free itself with `remove`. A `put` that cannot fit returns `kFull`
  after compacting what it can; further failing puts of the same size return quickly without wearing the
  flash. With 2 sectors the capacity is one sector, so use 3 or more when updates are frequent.
  `max_value_size()` is the largest value one `put` accepts.
- **I/O errors:** if the flash reports a failure, the call returns `kIoError` and the store stops answering
  (`kNotReady`) until `begin()` is called again, which re-mounts and repairs.
- **Bit rot:** each record has a CRC, so damage is detected, but not corrected: a damaged record is skipped
  and the key falls back to its previous value if one is still stored, and later records in the same sector
  are not reachable. Keep a version or checksum over your whole settings block if that matters.
- **Not interrupt safe.** Use it from one context.
- **Sector layout** (little-endian): 16-byte header (`"KVS1"`, sequence number, source sequence for a
  compaction target, flags, CRC-16), one program unit for the "copy complete" mark, then records
  `[key:2][info:2][value][crc:2]` padded to the program unit. Not compatible with other formats.

## Test

From this folder:

```
g++ -std=c++11 -Wall -Wextra -Werror -I.. test_kv_store.cpp -o test_kv_store && ./test_kv_store
```

Runs against `RamFlash`, which refuses any program of an already-written unit, so the test also proves
the store respects the flash rules. It covers the basics and every error code, identical writes skipping
the flash, values at every length around the internal chunk size, a full store (including freeing it again
by removing keys), staying inside its sector window, I/O errors on program and erase, random bit flips,
mounting garbage, wear spread across sectors, about 44,000 random operations against a `std::map` on eight
flash geometries (program units 1 to 16, 2 to 8 sectors) with reboots, and the **power-cut sweep**: for
each of six geometries the scenario is run once to count its flash writes and erases, then re-run with the
power cut at every single one of them, with the interrupted program leaving a torn unit or the interrupted
erase leaving the sector half erased (front erased, back erased, or scattered), and sometimes a second cut
during recovery. After each cut the store must mount, hold exactly the state from before or after the
interrupted call, and carry on correctly through the rest of the scenario.
