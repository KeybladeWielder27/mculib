// flash.hpp - minimal NOR-style flash interface, microcontroller agnostic.
//
// Models what internal flash and SPI NOR flash have in common:
//
//   * The device is split into equal sectors, the smallest erasable unit.
//   * Erasing a sector sets every byte to 0xFF.
//   * Programming writes bytes into erased space, in multiples of the
//     "program unit" (1 for plain NOR/SPI flash, 4/8/16 for many MCU internal
//     flashes with word or ECC-protected writes), at offsets that are multiples
//     of the program unit.
//   * A program unit may be programmed ONCE after an erase. Code written against
//     this interface never programs the same unit twice, so it also works on
//     flash with ECC where re-programming is not allowed.
//   * Reads are byte granular and may be done at any offset.
//
// Implementations return false when the device reports an error. After a
// failed program or erase the affected area has undefined content (it may be
// partly written); callers treat it as dirty and erase it before reuse.
//
// Ports live next to the project's HAL code: wrap your vendor's flash driver
// in a small class that derives from Flash (see storage/README.md for a sketch).
// Sectors here are numbered from 0 and need not match the vendor's numbering:
// give the library a window of sectors you have reserved for it.
//
// Header-only, C++11, no heap, no platform headers.

#ifndef MCU_FLASH_HPP
#define MCU_FLASH_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

class Flash {
 public:
  // Number of sectors and bytes per sector (all equal).
  virtual size_t sector_count() const = 0;
  virtual size_t sector_size() const = 0;

  // Program granularity in bytes: 1, 2, 4, 8 or 16.
  virtual size_t program_unit() const = 0;

  // Reads `n` bytes at `offset` within `sector`.
  virtual bool read(size_t sector, size_t offset, void* dst, size_t n) = 0;

  // Programs `n` bytes at `offset` within `sector`. `offset` and `n` are
  // multiples of program_unit(), and the target units are erased.
  virtual bool program(size_t sector, size_t offset, const void* src, size_t n) = 0;

  // Erases one sector (all bytes become 0xFF).
  virtual bool erase(size_t sector) = 0;

 protected:
  Flash() {}
  ~Flash() {}
};

}  // namespace mcu

#endif  // MCU_FLASH_HPP
