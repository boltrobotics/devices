// Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_I2C_hpp_
#define _btr_I2C_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

// PROJECT INCLUDES
#include "utility/value_codec.hpp"
#include "devices/avr/avr_macros.h"

#define DEFAULT_TIMEOUT 1000

namespace btr
{

/**
 * The class implements I2C protocol.
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
  static void begin(uint32_t timeout);

  /**
   * Set/get timeout.
   *
   * @param timeout - the timeout in microseconds
   */
  static uint32_t timeout(uint32_t* new_value = nullptr);

  /**
   * Activate or deactivate internal pull-up resistors.
   *
   * @param activate - deactivate pull-ups if false, otherwise activate
   */
  static void setPullups(bool activate);

  /**
   * Set TWI line speed.
   *
   * @param fast - when true, the line is set to 400kHz. Otherwise, it's set to 100kHz
   */
  static void setSpeed(bool fast);

  /**
   * Disable TWI.
   */
  static void end();

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
   * Write address, register and single-byte value.
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
   * @param addr - the address of a device to send the data to
   * @param reg - the register to read from
   * @param buff - the buffer to store the data to
   * @return one of STATUS_CODE or system value between 0x8 and 0xFF
   */
  static uint8_t read(uint8_t addr, uint8_t reg, uint8_t* buff, uint8_t count);

private:

// OPERATIONS

  static uint8_t start();
  static uint8_t sendAddress(uint8_t value);
  static uint8_t sendByte(uint8_t value);
  static uint8_t receiveByte(bool ack);
  static uint8_t receiveByte(bool ack, uint8_t* target);
  static uint8_t stop();
  static void reset();
  static uint8_t waitCompletion();
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

inline void I2C::begin(uint32_t new_timeout)
{
  timeout(&new_timeout);

  //setPullups(true);
  setSpeed(false);

  // Enable TWI operation and interface
  set_reg(TWCR, BV(TWEN) | BV(TWEA));
}

inline uint32_t I2C::timeout(uint32_t* new_timeout)
{
  static uint32_t timeout = DEFAULT_TIMEOUT;

  if (new_timeout != nullptr) {
    timeout = *new_timeout;
  }
  return timeout;
}

inline void I2C::setPullups(bool activate)
{
  // See discussion about external vs internal pull-ups:
  // http://www.avrfreaks.net/forum/twi-i2c-without-external-pull-resistors:

  if (activate) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    set_bit(PORTC, 4);
    set_bit(PORTC, 5);
#else
    // ATmega2560: PD0 (SCL0), PD1 (SDA)
    set_bit(PORTD, 0);
    set_bit(PORTD, 1);
#endif
  } else {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    clear_bit(PORTC, 4);
    clear_bit(PORTC, 5);
#else
    clear_bit(PORTD, 0);
    clear_bit(PORTD, 1);
#endif
  }
}

inline void I2C::setSpeed(bool fast)
{
#if defined (X86_HOST)
  (void)(fast);
#else
  // Set prescaler to 1
  clear_bit(TWSR, TWPS0);
  clear_bit(TWSR, TWPS1);
  uint32_t scaler = (fast ? 400000 : 100000);
  set_reg(TWBR, ((F_CPU / scaler) - 16) / 2);
#endif
}

inline void I2C::end()
{
  // Switch off TWI and terminate all ongoing transmissions
  set_reg(TWCR, 0);
}

inline uint8_t I2C::scan(uint8_t* device_count)
{
  uint8_t result = SUCCESS;
  *device_count = 0;

  for (uint8_t s = 0; s <= 0x0; s++) {
    result = start();

    if (SUCCESS == result) {
      result = sendAddress(SLA_W(s));

      if (SUCCESS == result) {
        (*device_count)++;
      } else {
        break;
      }
    } else {
      break;
    }
    stop();
  }
  return result;
}

inline uint8_t I2C::write(uint8_t addr, uint8_t reg)
{
  uint8_t result = start();

  if (SUCCESS == result) {
    result = sendAddress(SLA_W(addr));

    if (SUCCESS == result) {
      result = sendByte(reg);
    }
    stop();
  }
  return result;
}

template<typename T>
inline uint8_t I2C::write(uint8_t addr, uint8_t reg, T value)
{
  if (sizeof(T) > 1 && ValueCodec::isLittleEndian()) {
    ValueCodec::swap(&value);
  }

  const uint8_t* buff = reinterpret_cast<uint8_t*>(&value);
  return write(addr, reg, buff, sizeof(T));
}

inline uint8_t I2C::write(uint8_t addr, uint8_t reg, const uint8_t* buff, uint8_t count)
{
  uint8_t result = start();

  if (SUCCESS == result) {
    result = sendAddress(SLA_W(addr));

    if (SUCCESS == result) {
      result = sendByte(reg);

      if (SUCCESS == result) {
        for (uint8_t i = 0; i < count; i++) {
          result = sendByte(buff[i]);

          if (SUCCESS != result) {
            break;
          }
        }
      }
    } // sendAddress
    stop();
  }
  return result;
}

inline uint8_t I2C::read(uint8_t addr, uint8_t* buff, uint8_t count, bool with_reg)
{
  uint8_t result = start();

  if (SUCCESS == result) {
    result = sendAddress(SLA_R(addr));

    if (SUCCESS == result) {
      if (count == 0) {
        count++;
      }

      uint8_t nack = count - 1;

      for (uint8_t i = 0; i < count; i++) {
        if (i == nack) {
          result = receiveByte(false);

          if (SUCCESS == result) {
            if (TW_MR_DATA_NACK != TW_STATUS) {
              result = TW_STATUS;
              break;
            }
          } else {
            break;
          }
        } else {
          result = receiveByte(true);

          if (SUCCESS == result) {
            if (TW_MR_DATA_ACK != TW_STATUS) {
              result = TW_STATUS;
              break;
            }
          } else {
            break;
          }
        }
        buff[i] = TWDR;
      } // for loop
    } // sendAddress

    if (!with_reg) {
      stop();
    }
  } // start
  return result;
}

inline uint8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t* buff, uint8_t count)
{
  uint8_t result = start();

  if (SUCCESS == result) {
    result = sendAddress(SLA_W(addr));

    if (SUCCESS == result) {
      result = sendByte(reg);

      if (SUCCESS == result) {
        result = read(addr, buff, count, true);
      }
    }
    stop();
  }
  return result;
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

inline uint8_t I2C::start()
{
  set_reg(TWCR, BV(TWINT) | BV(TWSTA) | BV(TWEN));

  uint8_t result = waitCompletion();

  if (SUCCESS == result) {
    if (TW_MT_ARB_LOST == TW_STATUS) {
      result = TW_STATUS;
      reset();
    }
  }
  return result;
}

inline uint8_t I2C::sendAddress(uint8_t value)
{
  set_reg(TWDR, value);
  set_reg(TWCR, BV(TWINT) | BV(TWEN));

  uint8_t result = waitCompletion();

  if (SUCCESS == result) {
    if (TW_MT_SLA_NACK == TW_STATUS || TW_MR_SLA_NACK == TW_STATUS) {
      result = TW_STATUS;
      stop();
    } else if ((TW_MT_SLA_ACK != TW_STATUS) && (TW_MR_SLA_ACK != TW_STATUS)) {
      result = TW_STATUS;
      reset();
    }
  }
  return result;
}

inline uint8_t I2C::sendByte(uint8_t value)
{
  set_reg(TWDR, value);
  set_reg(TWCR, BV(TWINT) | BV(TWEN));

  uint8_t result = waitCompletion();

  if (SUCCESS == result) {
    if (TW_MT_DATA_NACK == TW_STATUS) {
      result = TW_STATUS;
      stop();
    } else if (TW_MT_DATA_ACK != TW_STATUS) {
      result = TW_STATUS;
      reset();
    }
  }
  return result;
}

inline uint8_t I2C::receiveByte(bool ack)
{
  set_reg(TWCR, BV(TWINT) | BV(TWEN));

  if (ack) {
    set_reg(TWCR, TWCR | BV(TWEA));
  }

  uint8_t result = waitCompletion();

  if (SUCCESS == result) {
    if (TW_MT_ARB_LOST == TW_STATUS) {
      result = TW_STATUS;
      reset();
    }
  }
  return result;
}

inline uint8_t I2C::receiveByte(bool ack, uint8_t* value)
{
  uint8_t result = I2C::receiveByte(ack);

  if (SUCCESS == result) {
    *value = TWDR;

    if (ack) {
      if (TW_MR_DATA_ACK != TW_STATUS) {
        result = TW_STATUS;
        *value = 0x0;
      }
    } else {
      if (TW_MR_DATA_NACK != TW_STATUS) {
        result = TW_STATUS;
        *value = 0x0;
      }
    }
  }
  return result;
}

inline uint8_t I2C::stop()
{
  uint8_t result = SUCCESS;
  set_reg(TWCR, BV(TWINT) | BV(TWEN) | BV(TWSTO));

  if (timeout() > 0) {
    uint32_t begin_time = micros();

    while (bit_is_set(TWCR, TWSTO)) {
      if ((micros() - begin_time) >= timeout()) {
        result = TIMEOUT;
        reset();
        break;
      }
    }
  } else {
    loop_until_bit_is_clear(TWCR, TWSTO);
  }

  return result;
}

inline void I2C::reset()
{
  // Re-initialize TWI
  end();
  set_reg(TWCR, BV(TWEN) | BV(TWEA));
}

inline uint8_t I2C::waitCompletion()
{
  uint8_t result = SUCCESS;

  if (timeout() > 0) {
    uint32_t begin_time = micros();

    while (bit_is_clear(TWCR, TWINT)) {
      if ((micros() - begin_time) >= timeout()) {
        result = TIMEOUT;
        reset();
        break;
      }
    }
  } else {
    loop_until_bit_is_set(TWCR, TWINT);
  }
  return result;
}

} // namespace btr

#endif // _btr_I2C_hpp_
