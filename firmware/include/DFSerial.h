#ifndef DFSerial_h
#define DFSerial_h

#include "mbed.h"  // Required for mbed::BufferedSerial and digitalPinToPinName()

/*******************************************************
 * DFSerial Library
 *
 * This library wraps mbed::BufferedSerial to act as an Arduino Stream.
 * It logs every sent and received byte to the USB Serial port when the
 * public member "logging" is set to true.
 *
 * Example usage:
 *    #include "DFSerial.h"
 *
 *    // Create a serial instance on your desired pins:
 *    DFSerial mySerial(digitalPinToPinName(D1), digitalPinToPinName(D0), 115200);
 *    mySerial.logging = true; // Enable logging
 *
 *    // Then pass 'mySerial' to libraries expecting a Stream.
 *******************************************************/
class DFSerial : public Stream {
public:
  // Public member to enable/disable logging.
  bool logging = true;
  
  // Constructor: pass TX pin, RX pin (as mbed PinName) and the baud rate.
  DFSerial(PinName tx, PinName rx, int baud)
    : _serial(tx, rx, baud) { }
  
  // Returns the number of bytes available for reading.
  int available() override {
    return _serial.readable();
  }
  
  // Reads one byte. Logs it if logging is enabled.
  int read() override {
    char c;
    if (_serial.read(&c, 1)) {
      if (logging) {
        Serial.print("RX: 0x");
        Serial.println((uint8_t)c, HEX);
      }
      return static_cast<int>(c);
    }
    return -1;
  }
  
  // Peek is not implemented.
  int peek() override {
    return -1;
  }
  
  // flush() does nothing in this implementation.
  void flush() override { }
  
  // Writes one byte. Logs it if logging is enabled.
  size_t write(uint8_t c) override {
    if (logging) {
      Serial.print("TX: 0x");
      Serial.println((uint8_t)c, HEX);
    }
    _serial.write(&c, 1);
    return 1;
  }
  
  // Writes an array of bytes, logging each if enabled.
  size_t write(const uint8_t *buffer, size_t size) override {
    for (size_t i = 0; i < size; i++) {
      write(buffer[i]);
    }
    return size;
  }
  
private:
  mbed::BufferedSerial _serial;
};

#endif // DFSerial_h
