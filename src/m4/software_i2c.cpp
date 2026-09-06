#include "software_i2c.h"

#include <algorithm>

SoftwareI2C::SoftwareI2C(pin_size_t sda_pin, pin_size_t scl_pin)
    : arduino::MbedI2C(static_cast<int>(sda_pin),
                       static_cast<int>(scl_pin)),
      sda_pin_(sda_pin),
      scl_pin_(scl_pin)
{
}

void SoftwareI2C::driveLow(pin_size_t pin)
{
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void SoftwareI2C::release(pin_size_t pin)
{
  pinMode(pin, INPUT_PULLUP);
}

bool SoftwareI2C::waitSclHigh()
{
  release(scl_pin_);
  const uint32_t startedUs = micros();
  while (digitalRead(scl_pin_) == LOW) {
    if (micros() - startedUs >= CLOCK_STRETCH_TIMEOUT_US)
      return false;
  }
  delayMicroseconds(half_period_us_);
  return true;
}

void SoftwareI2C::recoverBus()
{
  release(sda_pin_);
  release(scl_pin_);
  delayMicroseconds(half_period_us_);
  if (digitalRead(sda_pin_) == HIGH)
    return;

  for (uint8_t pulse = 0; pulse < 9 && digitalRead(sda_pin_) == LOW; ++pulse) {
    driveLow(scl_pin_);
    delayMicroseconds(half_period_us_);
    waitSclHigh();
  }
  stopCondition();
}

void SoftwareI2C::begin()
{
  bus_held_ = false;
  recoverBus();
}

void SoftwareI2C::end()
{
  if (bus_held_)
    stopCondition();
  release(sda_pin_);
  release(scl_pin_);
}

void SoftwareI2C::setClock(uint32_t frequency)
{
  // GPIO and interrupt latency make 100 kHz the sensible upper bound here.
  const uint32_t bounded = std::max<uint32_t>(10000, std::min<uint32_t>(frequency, 100000));
  half_period_us_ = std::max<uint32_t>(1, 500000UL / bounded);
}

bool SoftwareI2C::startCondition()
{
  release(sda_pin_);
  if (!waitSclHigh())
    return false;
  if (!bus_held_ && digitalRead(sda_pin_) == LOW)
    return false;

  delayMicroseconds(half_period_us_);
  driveLow(sda_pin_);
  delayMicroseconds(half_period_us_);
  driveLow(scl_pin_);
  bus_held_ = true;
  return true;
}

void SoftwareI2C::stopCondition()
{
  driveLow(sda_pin_);
  delayMicroseconds(half_period_us_);
  if (waitSclHigh()) {
    release(sda_pin_);
    delayMicroseconds(half_period_us_);
  }
  bus_held_ = false;
}

bool SoftwareI2C::writeByte(uint8_t value)
{
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    if (value & mask)
      release(sda_pin_);
    else
      driveLow(sda_pin_);
    delayMicroseconds(half_period_us_);
    if (!waitSclHigh())
      return false;
    driveLow(scl_pin_);
  }

  release(sda_pin_);
  delayMicroseconds(half_period_us_);
  if (!waitSclHigh())
    return false;
  const bool acknowledged = digitalRead(sda_pin_) == LOW;
  driveLow(scl_pin_);
  return acknowledged;
}

uint8_t SoftwareI2C::readByte(bool acknowledge)
{
  uint8_t value = 0;
  release(sda_pin_);
  for (uint8_t bit = 0; bit < 8; ++bit) {
    value <<= 1;
    delayMicroseconds(half_period_us_);
    if (!waitSclHigh())
      break;
    if (digitalRead(sda_pin_) == HIGH)
      value |= 1;
    driveLow(scl_pin_);
  }

  if (acknowledge)
    driveLow(sda_pin_);
  else
    release(sda_pin_);
  delayMicroseconds(half_period_us_);
  waitSclHigh();
  driveLow(scl_pin_);
  release(sda_pin_);
  return value;
}

void SoftwareI2C::beginTransmission(uint8_t address)
{
  tx_address_ = address;
  tx_length_ = 0;
}

uint8_t SoftwareI2C::endTransmission(bool stop_bit)
{
  if (!startCondition()) {
    recoverBus();
    return 4;
  }
  if (!writeByte(static_cast<uint8_t>(tx_address_ << 1))) {
    stopCondition();
    return 2;
  }
  for (size_t i = 0; i < tx_length_; ++i) {
    if (!writeByte(tx_buffer_[i])) {
      stopCondition();
      return 3;
    }
  }
  if (stop_bit)
    stopCondition();
  return 0;
}

uint8_t SoftwareI2C::endTransmission()
{
  return endTransmission(true);
}

size_t SoftwareI2C::requestFrom(uint8_t address, size_t length, bool stop_bit)
{
  rx_index_ = 0;
  rx_length_ = 0;
  length = std::min(length, BUFFER_SIZE);
  if (!startCondition() ||
      !writeByte(static_cast<uint8_t>((address << 1) | 1U))) {
    stopCondition();
    return 0;
  }

  for (size_t i = 0; i < length; ++i)
    rx_buffer_[rx_length_++] = readByte(i + 1 < length);
  if (stop_bit)
    stopCondition();
  return rx_length_;
}

size_t SoftwareI2C::requestFrom(uint8_t address, size_t length)
{
  return requestFrom(address, length, true);
}

size_t SoftwareI2C::write(uint8_t value)
{
  if (tx_length_ >= BUFFER_SIZE)
    return 0;
  tx_buffer_[tx_length_++] = value;
  return 1;
}

size_t SoftwareI2C::write(const uint8_t *data, int length)
{
  if (data == nullptr || length <= 0)
    return 0;
  const size_t writable = std::min<size_t>(
      static_cast<size_t>(length), BUFFER_SIZE - tx_length_);
  memcpy(tx_buffer_ + tx_length_, data, writable);
  tx_length_ += writable;
  return writable;
}

int SoftwareI2C::available()
{
  return static_cast<int>(rx_length_ - rx_index_);
}

int SoftwareI2C::read()
{
  return rx_index_ < rx_length_ ? rx_buffer_[rx_index_++] : -1;
}

int SoftwareI2C::peek()
{
  return rx_index_ < rx_length_ ? rx_buffer_[rx_index_] : -1;
}

void SoftwareI2C::flush()
{
}
