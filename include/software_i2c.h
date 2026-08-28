#pragma once

#include <Wire.h>

/**
 * Minimal master-only, software-backed TwoWire implementation.
 *
 * The STM32duino VL53L4CX driver stores a TwoWire pointer and calls its
 * virtual methods, allowing this implementation to provide an independent
 * I2C bus on ordinary GPIO pins without modifying the vendor library.
 */
class SoftwareI2C final : public arduino::MbedI2C {
 public:
  SoftwareI2C(pin_size_t sda_pin, pin_size_t scl_pin);

  void begin() override;
  void end() override;
  void setClock(uint32_t frequency) override;
  void beginTransmission(uint8_t address) override;
  uint8_t endTransmission(bool stop_bit) override;
  uint8_t endTransmission() override;
  size_t requestFrom(uint8_t address, size_t length, bool stop_bit) override;
  size_t requestFrom(uint8_t address, size_t length) override;
  size_t write(uint8_t value) override;
  size_t write(const uint8_t *data, int length) override;
  int available() override;
  int read() override;
  int peek() override;
  void flush() override;

 private:
  static constexpr size_t BUFFER_SIZE = 256;
  static constexpr uint32_t CLOCK_STRETCH_TIMEOUT_US = 2000;

  pin_size_t sda_pin_;
  pin_size_t scl_pin_;
  uint32_t half_period_us_ = 5;
  uint8_t tx_address_ = 0;
  uint8_t tx_buffer_[BUFFER_SIZE] = {};
  size_t tx_length_ = 0;
  uint8_t rx_buffer_[BUFFER_SIZE] = {};
  size_t rx_length_ = 0;
  size_t rx_index_ = 0;
  bool bus_held_ = false;

  void driveLow(pin_size_t pin);
  void release(pin_size_t pin);
  bool waitSclHigh();
  bool startCondition();
  void stopCondition();
  bool writeByte(uint8_t value);
  uint8_t readByte(bool acknowledge);
  void recoverBus();
};
