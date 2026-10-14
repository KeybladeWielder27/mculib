// stream.hpp - minimal non-blocking byte stream interface, microcontroller agnostic.
//
// This is the only thing the portable libraries know about a UART, SPI link,
// USB CDC port, TCP socket or an in-memory pipe. A port for a given platform
// is a small class that derives from mcu::Stream and forwards to the vendor
// API (see ports/arduino_stream.hpp for an example).
//
// Rules every implementation follows:
//   * Nothing here ever blocks. Calls return immediately with how many bytes
//     were actually moved, which may be zero.
//   * Partial transfers are normal. Callers loop or try again later.
//
// Header-only, C++11, no heap. The destructor is deliberately non-virtual and
// protected: nothing is ever deleted through a Stream*, and this keeps
// operator delete (and its code size) out of small-MCU builds.

#ifndef MCU_STREAM_HPP
#define MCU_STREAM_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

// Returned by Stream::write_space() when the stream cannot tell how much it
// can accept (writes may then block or drop, depending on the platform).
constexpr size_t kUnboundedSpace = (size_t)-1;

class Stream {
 public:
  // Copies up to `max` received bytes into dst. Returns the number copied
  // (0 if nothing is waiting).
  virtual size_t read(uint8_t* dst, size_t max) = 0;

  // Queues up to `len` bytes for sending. Returns the number accepted
  // (0 if there is no room right now). Accepted bytes are never taken back.
  virtual size_t write(const uint8_t* src, size_t len) = 0;

  // Number of bytes that read() could return right now.
  virtual size_t available() const = 0;

  // Number of bytes that write() would accept right now, or kUnboundedSpace
  // if unknown. Lets a caller check that a whole frame fits before sending
  // any of it, so frames are not split by a full transmit buffer.
  virtual size_t write_space() const = 0;

  // ---- convenience wrappers (not virtual) ----

  // Reads one byte. Returns false if none is waiting.
  bool read_byte(uint8_t& out) { return read(&out, 1) == 1; }

  // Writes one byte. Returns false if there is no room.
  bool write_byte(uint8_t b) { return write(&b, 1) == 1; }

 protected:
  Stream() {}
  ~Stream() {}
};

}  // namespace mcu

#endif  // MCU_STREAM_HPP
