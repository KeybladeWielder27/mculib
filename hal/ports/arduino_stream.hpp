// ports/arduino_stream.hpp - adapts any Arduino-style serial object to mcu::Stream.
//
// This header does NOT include Arduino.h. It is a template over the Arduino
// object's type, so it works with Serial, Serial1, SoftwareSerial, USB CDC,
// WiFiClient, or anything else with the usual Arduino stream methods:
//
//   int  available();
//   int  read();                              // -1 if nothing
//   size_t write(const uint8_t*, size_t);
//   int  availableForWrite();                 // optional, see below
//
// Usage:
//   mcu::ArduinoStream<HardwareSerial> uart(Serial1);
//   mcu::FunctionClock clock(millis);
//   framer.poll(uart);                        // framer only sees mcu::Stream
//
// Non-blocking writes: Arduino's write() blocks when the transmit buffer is
// full. If your core implements availableForWrite() properly (HardwareSerial
// on AVR, SAMD, STM32, ESP32, RP2040 do), leave `use_available_for_write` at
// true and writes are limited to the free space, so they never block. On
// objects where availableForWrite() just returns 0 (some SoftwareSerial and
// older Print-based classes), pass false: write() then passes everything
// through, may block, and write_space() reports kUnboundedSpace.

#ifndef MCU_ARDUINO_STREAM_HPP
#define MCU_ARDUINO_STREAM_HPP

#include "hal/stream.hpp"

namespace mcu {

template <typename S>
class ArduinoStream final : public Stream {
 public:
  explicit ArduinoStream(S& s, bool use_available_for_write = true)
      : s_(s), limit_writes_(use_available_for_write) {}

  size_t read(uint8_t* dst, size_t max) override {
    size_t n = 0;
    while (n < max && s_.available() > 0) {
      int c = s_.read();
      if (c < 0) break;
      dst[n++] = (uint8_t)c;
    }
    return n;
  }

  size_t write(const uint8_t* src, size_t len) override {
    if (limit_writes_) {
      size_t room = write_space();
      if (len > room) len = room;
    }
    if (len == 0) return 0;
    return (size_t)s_.write(src, len);
  }

  size_t available() const override {
    int n = s_.available();
    return n > 0 ? (size_t)n : 0;
  }

  size_t write_space() const override {
    if (!limit_writes_) return kUnboundedSpace;
    int n = s_.availableForWrite();
    return n > 0 ? (size_t)n : 0;
  }

 private:
  S& s_;
  bool limit_writes_;
};

}  // namespace mcu

#endif  // MCU_ARDUINO_STREAM_HPP
