// Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_I2C_hpp_
#define _btr_I2C_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

// PROJECT INCLUDES
#include "devices/defines.hpp"
#include "utility/value_codec.hpp"

namespace btr
{

/**
 * The class implements I2C protocol handling for AVR/STM32 platforms.
 */
class I2C
{
public:

  /**
   *  See ATmega2560 datasheet p.249 for system codes.
   */
  enum STATUS_CODE {
    // TW_BUS_ERROR is defined as 0x0; 0x1 - 0x7 are not defined, use them.
    SUCCESS = 1,
    TIMEOUT = 2
  };

// LIFECYCLE

  I2C() = delete;
  ~I2C() = delete;

// OPERATIONS

  /**
   * Set pull-up resistors high, speed slow, and enable TWI.
   */
  static void init(uint32_t timeout);

  /**
   * Disable TWI.
   */
  static void shutdown();

  /**
   * Set/get timeout.
   *
   * @param timeout - timeout in milliseconds
   */
  static uint32_t timeout(uint32_t* timeout = nullptr);

  /**
   * Activate or deactivate internal pull-up resistors.
   *
   * @param activate - if false - deactivate pull-ups, if true - activate
   */
  static void setPullups(bool activate);

  /**
   * Set TWI line speed.
   *
   * @param fast - when true, the line is set to 400kHz. Otherwise, it's set to 100kHz
   */
  static void setSpeed(bool fast);

  /**
   * Scan for i2c devices and provide the number of available devices.
   *
   * @param device_count - the number of available devices
   * @return one of STATUS_CODE or system value between 0x8 and 0xFF
   */
  static uint8_t scan(uint8_t* device_count);

  /**
   * Write address and register only.
   *
   * @param addr - the address of a device to send the data to
   * @param reg - the address of a register on the device
   * @return one of STATUS_CODE or system value between 0x8 and 0xFF
   */
  static uint8_t write(uint8_t addr, uint8_t reg);

  /**
   * Write address, register and variable-byte value.
   *
   * @param addr - the address of a device to send the data to
   * @param reg - the address of a register on the device
   * @param value - the value
   * @return one of STATUS_CODE or system value between 0x8 and 0xFF
   */
  template<typename T>
  static uint8_t write(uint8_t addr, uint8_t reg, T value);

  /**
   * Write address, register and multi-byte value.
   *
   * @param addr - the address of a device to send the data to
   * @param reg - the address of a register on the device
   * @param buff - the buffer with data
   * @param count - the number of bytes in buff
   * @return one of STATUS_CODE or system value between 0x8 and 0xFF
   */
  static uint8_t write(uint8_t addr, uint8_t reg, const uint8_t* buff, uint8_t count);

  /**
   * Read multi-byte value.
   *
   * @param addr - the address of a device to send the data to
   * @param buff - the buffer to store the data to
   * @param count - the number of bytes in buff
   * @param with_reg - the function is called from read() with reg parameter, so do not call
   *  stop() at then read end.
   * @return one of STATUS_CODE or system value between 0x8 and 0xFF
   *
   */
  static uint8_t read(uint8_t addr, uint8_t* buff, uint8_t count, bool with_reg);

  /**
   * Read multi-byte value from a specific register.
   *
   * @param addr - address of a device to send the data to
   * @param reg - register to read from
   * @param buff - buffer to store the data in
   * @param count - number of bytes to read
   * @return one of STATUS_CODE or system value between 0x8 and 0xFF
   */
  static uint8_t read(uint8_t addr, uint8_t reg, uint8_t* buff, uint8_t count);

private:

// OPERATIONS

  static uint8_t start();
  static uint8_t stop();
  static void reset();
  static uint8_t sendAddress(uint8_t value);
  static uint8_t sendByte(uint8_t value);
  static uint8_t receiveByte(bool ack);
  static uint8_t receiveByte(bool ack, uint8_t* target);
  static uint8_t waitCompletion();
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

template<typename T>
uint8_t I2C::write(uint8_t addr, uint8_t reg, T value)
{
  if (sizeof(T) > 1 && ValueCodec::isLittleEndian()) {
    ValueCodec::swap(&value);
  }

  const uint8_t* buff = reinterpret_cast<uint8_t*>(&value);
  return write(addr, reg, buff, sizeof(T));
}

} // namespace btr

#endif // _btr_I2C_hpp_
