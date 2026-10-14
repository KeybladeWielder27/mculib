// loopback_stream.hpp - an in-memory Stream for tests, simulation and wiring
// two software modules together without any hardware.
//
// The stream has two sides:
//   * the "device" side is what your code under test sees through the Stream
//     interface: read() returns bytes you injected, write() collects bytes
//     into a transmit buffer.
//   * the "wire" side is what a test (or another module) uses to feed the
//     device and to look at what it sent: inject() and take().
//
//   mcu::LoopbackStream<64, 64> port;      // 64-byte receive and transmit buffers
//   port.inject(rx_bytes, n);              // pretend these arrived on the wire
//   framer.poll(port);                     // code under test reads from `port`
//   uint8_t out[32];
//   size_t got = port.take(out, sizeof out);   // what the code under test sent
//
// Capacities must be powers of two (see ring_buffer.hpp). Like all the
// libraries in this collection: header-only, C++11, no heap.
//
// Include path: the root of the collection must be on the include path, as it
// pulls in "ring_buffer/ring_buffer.hpp".

#ifndef MCU_LOOPBACK_STREAM_HPP
#define MCU_LOOPBACK_STREAM_HPP

#include "hal/stream.hpp"
#include "ring_buffer/ring_buffer.hpp"

namespace mcu {

template <size_t RxN, size_t TxN>
class LoopbackStream final : public Stream {
 public:
  LoopbackStream() {}

  // ---- device side (Stream interface) ----
  size_t read(uint8_t* dst, size_t max) override { return rx_.read(dst, max); }
  size_t write(const uint8_t* src, size_t len) override { return tx_.write(src, len); }
  size_t available() const override { return rx_.size(); }
  size_t write_space() const override { return tx_.free_space(); }

  // ---- wire side ----

  // Delivers bytes to the device. Returns how many fit in the receive buffer.
  size_t inject(const uint8_t* src, size_t len) { return rx_.write(src, len); }

  // Collects bytes the device has written. Returns how many were copied.
  size_t take(uint8_t* dst, size_t max) { return tx_.read(dst, max); }

  // Bytes written by the device and not yet taken.
  size_t tx_size() const { return tx_.size(); }

  // Space left for inject().
  size_t rx_space() const { return rx_.free_space(); }

  void clear() {
    rx_.clear();
    tx_.clear();
  }

 private:
  RingBuffer<uint8_t, RxN> rx_;
  RingBuffer<uint8_t, TxN> tx_;
};

}  // namespace mcu

#endif  // MCU_LOOPBACK_STREAM_HPP
