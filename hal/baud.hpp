// baud.hpp - changing a serial port's baud rate, microcontroller agnostic.
//
// Some devices change their own baud rate on command (an XBee after ATBD, a
// GPS after a configuration sentence, most AT modems after AT+IPR). A driver
// for such a device must then retune the host UART to match. mcu::Stream does
// not know about baud rates (many streams have none), so that ability is a
// separate, optional interface the driver takes alongside the Stream.
//
//   static bool xbee_baud(uint32_t b) { Serial1.updateBaudRate(b); return true; }   // ESP32
//   mcu::FunctionBaudControl baud(xbee_baud);
//
//   mcu::ArduinoBaudControl<HardwareSerial> baud(Serial1);   // any core: end() then begin(b)
//
// Header-only, C++11, no heap, no platform headers.

#ifndef MCU_BAUD_HPP
#define MCU_BAUD_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

class BaudControl {
 public:
  // Reconfigures the port to `baud` bits per second. Returns false if the
  // rate is not supported (the port then keeps its old rate). Implementations
  // should let bytes already queued for sending finish first where the
  // platform allows it.
  virtual bool set_baud(uint32_t baud) = 0;

 protected:
  BaudControl() {}
  ~BaudControl() {}
};

// Wraps any `bool f(uint32_t baud)` function.
class FunctionBaudControl final : public BaudControl {
 public:
  typedef bool (*Fn)(uint32_t baud);
  explicit FunctionBaudControl(Fn fn) : fn_(fn) {}
  bool set_baud(uint32_t baud) override { return fn_ != 0 && fn_(baud); }

 private:
  Fn fn_;
};

// Records requested rates instead of touching hardware. For tests and
// simulations. Optionally refuses rates above `max_baud`.
class ManualBaudControl final : public BaudControl {
 public:
  explicit ManualBaudControl(uint32_t initial = 9600, uint32_t max_baud = 0xFFFFFFFFu)
      : baud_(initial), max_(max_baud), changes_(0) {}

  bool set_baud(uint32_t baud) override {
    if (baud == 0 || baud > max_) return false;
    baud_ = baud;
    ++changes_;
    return true;
  }

  uint32_t baud() const { return baud_; }
  uint32_t changes() const { return changes_; }

 private:
  uint32_t baud_;
  uint32_t max_;
  uint32_t changes_;
};

}  // namespace mcu

#endif  // MCU_BAUD_HPP
