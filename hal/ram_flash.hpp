// ram_flash.hpp - RAM-backed simulated flash for host-side tests.
//
// Behaves like the Flash interface describes, and checks that the code under
// test respects it:
//
//   * programming a unit that is not erased (including a unit programmed before
//     and not erased since) is counted in violations() and refused;
//   * misaligned or out-of-range accesses are counted and refused.
//
// Fault injection:
//
//   power_fail_at(k, seed)  the k-th write operation (program or erase, counting
//                           from reset_counters()) is interrupted: a program
//                           leaves a random prefix of its units written and one
//                           unit partly written; an erase leaves the sector with
//                           the front or the back erased and the rest still old,
//                           or a random mix of erased and old bits. From then on
//                           every call fails until power_cycle(), as if the
//                           board had lost power.
//   fail_programs(true)     every program fails without touching the flash.
//   fail_erases(true)       likewise for erase.
//   flip_bit(...)           flips one stored bit (bit rot).
//
// Wear: erase_count(sector) and total_erases().
//
// Not for firmware: it holds the whole flash in RAM.

#ifndef MCU_RAM_FLASH_HPP
#define MCU_RAM_FLASH_HPP

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hal/flash.hpp"

namespace mcu {

template <size_t SectorSize, size_t Sectors, size_t Unit = 1>
class RamFlash final : public Flash {
  static_assert(Unit == 1 || Unit == 2 || Unit == 4 || Unit == 8 || Unit == 16,
                "Unit must be 1, 2, 4, 8 or 16");
  static_assert(SectorSize % Unit == 0, "SectorSize must be a multiple of Unit");
  static_assert(Sectors >= 1, "need at least one sector");

 public:
  RamFlash() { wipe(); }

  // ---- Flash interface ----
  size_t sector_count() const override { return Sectors; }
  size_t sector_size() const override { return SectorSize; }
  size_t program_unit() const override { return Unit; }

  bool read(size_t sector, size_t offset, void* dst, size_t n) override {
    if (dead_ || sector >= Sectors || offset > SectorSize || n > SectorSize - offset) return false;
    memcpy(dst, mem_[sector] + offset, n);
    reads_ += n;
    return true;
  }

  bool program(size_t sector, size_t offset, const void* src, size_t n) override {
    if (dead_) return false;
    if (sector >= Sectors || offset > SectorSize || n > SectorSize - offset ||
        offset % Unit != 0 || n % Unit != 0 || n == 0) {
      ++violations_;
      return false;
    }
    const size_t first = offset / Unit, units = n / Unit;
    for (size_t u = 0; u < units; ++u) {
      if (dirty_[sector][first + u]) {
        ++violations_;
        return false;
      }
    }
    if (fail_programs_) return false;
    const uint8_t* s = static_cast<const uint8_t*>(src);
    if (trace_) printf("  [flash] program sector %zu offset %zu len %zu\n", sector, offset, n);
    ++ops_;
    if (fail_at_ != 0 && ops_ == fail_at_) {
      const size_t done = rnd() % (units + 1);  // units fully written
      for (size_t u = 0; u < done; ++u) put_unit(sector, first + u, s + u * Unit, 0);
      if (done < units) {  // one unit with only some bits programmed
        for (size_t b = 0; b < Unit; ++b) {
          const uint8_t keep = (uint8_t)rnd();  // 1 bits: programming did not happen
          mem_[sector][(first + done) * Unit + b] =
              (uint8_t)(mem_[sector][(first + done) * Unit + b] & (s[done * Unit + b] | keep));
        }
      }
      refresh_dirty(sector);  // a unit whose bits all stayed erased still reads as blank
      dead_ = true;
      return false;
    }
    for (size_t u = 0; u < units; ++u) put_unit(sector, first + u, s + u * Unit, 0);
    programmed_bytes_ += n;
    return true;
  }

  bool erase(size_t sector) override {
    if (dead_) return false;
    if (sector >= Sectors) {
      ++violations_;
      return false;
    }
    if (fail_erases_) return false;
    if (trace_) printf("  [flash] erase sector %zu\n", sector);
    ++ops_;
    if (fail_at_ != 0 && ops_ == fail_at_) {
      // Real parts erase in no guaranteed order: simulate a front part erased, a
      // back part erased, or a random scatter of erased and half-erased bytes.
      const uint32_t mode = rnd() % 3;
      const size_t cut = rnd() % (SectorSize + 1);
      for (size_t i = 0; i < SectorSize; ++i) {
        const uint32_t r = rnd();
        if (mode == 0) {
          if (i < cut) mem_[sector][i] = 0xFF;
        } else if (mode == 1) {
          if (i >= cut) mem_[sector][i] = 0xFF;
        } else if (r & 1) {
          mem_[sector][i] = 0xFF;
        } else {
          mem_[sector][i] = (uint8_t)(mem_[sector][i] | (uint8_t)(r >> 8));
        }
      }
      refresh_dirty(sector);
      dead_ = true;
      return false;
    }
    memset(mem_[sector], 0xFF, SectorSize);
    memset(dirty_[sector], 0, sizeof dirty_[sector]);
    ++erase_count_[sector];
    ++total_erases_;
    return true;
  }

  // ---- test controls ----

  // Interrupt the k-th write operation from now (1 = the next one). 0 disarms.
  void power_fail_at(uint32_t k, uint32_t seed) {
    ops_ = 0;
    fail_at_ = k;
    rng_ = seed * 2654435761u + 12345u;
    if (rng_ == 0) rng_ = 1;
  }
  bool dead() const { return dead_; }
  void set_trace(bool on) { trace_ = on; }  // print every program and erase
  void power_cycle() {  // the board is powered again; the flash keeps what it had
    dead_ = false;
    fail_at_ = 0;
  }
  // Write operations (programs and erases) since power_fail_at() / reset_counters().
  uint32_t ops() const { return ops_; }
  void reset_counters() {
    ops_ = 0;
    fail_at_ = 0;
    violations_ = 0;
    programmed_bytes_ = 0;
    reads_ = 0;
  }
  uint32_t violations() const { return violations_; }
  uint32_t erase_count(size_t sector) const { return erase_count_[sector]; }
  uint32_t total_erases() const { return total_erases_; }
  uint64_t programmed_bytes() const { return programmed_bytes_; }
  uint64_t bytes_read() const { return reads_; }
  void fail_programs(bool on) { fail_programs_ = on; }
  void fail_erases(bool on) { fail_erases_ = on; }

  // Flip one bit in place (bit rot). The unit is marked as written.
  void flip_bit(size_t sector, size_t offset, unsigned bit) {
    mem_[sector][offset] = (uint8_t)(mem_[sector][offset] ^ (1u << (bit & 7)));
    refresh_dirty(sector);
  }
  // Direct access for tests that corrupt or inspect content.
  uint8_t* raw(size_t sector) { return mem_[sector]; }
  void mark_all_dirty_from_content() {
    for (size_t s = 0; s < Sectors; ++s) refresh_dirty(s);
  }

  // Back to a factory-fresh device (all erased, counters cleared).
  void wipe() {
    memset(mem_, 0xFF, sizeof mem_);
    memset(dirty_, 0, sizeof dirty_);
    memset(erase_count_, 0, sizeof erase_count_);
    total_erases_ = 0;
    dead_ = false;
    trace_ = false;
    fail_programs_ = fail_erases_ = false;
    rng_ = 1;
    reset_counters();
  }

 private:
  uint32_t rnd() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
  }
  void put_unit(size_t sector, size_t unit, const uint8_t* src, size_t) {
    for (size_t b = 0; b < Unit; ++b)
      mem_[sector][unit * Unit + b] = (uint8_t)(mem_[sector][unit * Unit + b] & src[b]);
    dirty_[sector][unit] = true;
  }
  void refresh_dirty(size_t sector) {
    for (size_t u = 0; u < SectorSize / Unit; ++u) {
      bool blank = true;
      for (size_t b = 0; b < Unit; ++b) blank = blank && mem_[sector][u * Unit + b] == 0xFF;
      dirty_[sector][u] = !blank;
    }
  }

  uint8_t mem_[Sectors][SectorSize];
  bool dirty_[Sectors][SectorSize / Unit];
  uint32_t erase_count_[Sectors];
  uint32_t total_erases_;
  uint32_t ops_, fail_at_, violations_, rng_;
  uint64_t programmed_bytes_, reads_;
  bool dead_, fail_programs_, fail_erases_, trace_;
};

}  // namespace mcu

#endif  // MCU_RAM_FLASH_HPP
