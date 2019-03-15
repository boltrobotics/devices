// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// PROJECT INCLUDES
#include "devices/i2c.hpp"  // class implemented
#include "devices/time.hpp"

// SYSTEM INCLDUES
#include <util/twi.h>
#include <util/delay.h>

#if BTR_I2C_ENABLED > 0

namespace btr
{

uint8_t I2C::buff_[sizeof(uint64_t)] = { 0 };

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

void I2C::init()
{
  //setPullups(true);
  setSpeed(BTR_I2C_FAST_SPEED);

  // Enable TWI operation and interface.
  set_reg(TWCR, BV(TWEN) | BV(TWEA));
}

void I2C::shutdown()
{
  // Switch off TWI and terminate all ongoing transmissions
  set_reg(TWCR, 0);
}

void I2C::setPullups(bool activate)
{
  // See discussion about external vs internal pull-ups:
  // http://www.avrfreaks.net/forum/twi-i2c-without-external-pull-resistors:

  if (activate) {
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    set_bit(PORTC, 4);
    set_bit(PORTC, 5);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // ATmega2560: PD0 (SCL0), PD1 (SDA)
    set_bit(PORTD, 0);
    set_bit(PORTD, 1);
#endif
  } else {
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    clear_bit(PORTC, 4);
    clear_bit(PORTC, 5);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    clear_bit(PORTD, 0);
    clear_bit(PORTD, 1);
#endif
  }
}

void I2C::setSpeed(bool fast)
{
  // Set prescaler to 1
  clear_bit(TWSR, TWPS0);
  clear_bit(TWSR, TWPS1);
  uint32_t scaler = (fast ? 400000 : 100000);
  set_reg(TWBR, ((F_CPU / scaler) - 16) / 2);
}

uint8_t I2C::scan(uint8_t* device_count)
{
  uint8_t result = SUCCESS;
  *device_count = 0;

  for (uint8_t s = 0; s < 128; s++) {
    result = start();

    if (SUCCESS == result) {
      result = sendAddress(BTR_I2C_WRITE_ADDR(s));

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

uint8_t I2C::write(uint8_t addr, uint8_t reg)
{
  uint8_t result = start();

  if (SUCCESS == result) {
    result = sendAddress(BTR_I2C_WRITE_ADDR(addr));

    if (SUCCESS == result) {
      result = sendByte(reg);
    }
    stop();
  }
  return result;
}

uint8_t I2C::write(uint8_t addr, uint8_t reg, const uint8_t* buff, uint8_t count)
{
  uint8_t result = start();

  if (SUCCESS == result) {
    result = sendAddress(BTR_I2C_WRITE_ADDR(addr));

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

uint8_t I2C::read(uint8_t sid, uint8_t* buff, uint8_t count, bool stop_comm)
{
  uint8_t result = start();

  if (SUCCESS == result) {
    result = sendAddress(BTR_I2C_READ_ADDR(sid));

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

    if (stop_comm) {
      stop();
    }
  } // start
  return result;
}

uint8_t I2C::read(uint8_t sid, uint8_t reg, uint8_t* buff, uint8_t count)
{
  uint8_t result = start();

  if (SUCCESS == result) {
    result = sendAddress(BTR_I2C_WRITE_ADDR(sid));

    if (SUCCESS == result) {
      result = sendByte(reg);

      if (SUCCESS == result) {
        result = read(sid, buff, count, false);
      }
    }
    stop();
  }
  return result;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

uint8_t I2C::start()
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

uint8_t I2C::stop()
{
  uint8_t result = SUCCESS;
  set_reg(TWCR, BV(TWINT) | BV(TWEN) | BV(TWSTO));

  if (BTR_I2C_IO_TIMEOUT_MS > 0) {
    uint32_t start_ms = Time::millis();

    while (bit_is_set(TWCR, TWSTO)) {
      if ((Time::millis() - start_ms) >= BTR_I2C_IO_TIMEOUT_MS) {
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

void I2C::reset()
{
  // Re-initialize TWI
  shutdown();
  set_reg(TWCR, BV(TWEN) | BV(TWEA));
}

uint8_t I2C::sendAddress(uint8_t value)
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

uint8_t I2C::sendByte(uint8_t value)
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

uint8_t I2C::receiveByte(bool ack)
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

uint8_t I2C::receiveByte(bool ack, uint8_t* value)
{
  uint8_t result = I2C::receiveByte(ack);

  if (SUCCESS == result) {
    *value = TWDR;

    if (ack) {
      if (TW_MR_DATA_ACK != TW_STATUS) {
        result = TW_STATUS;
        *value = 0;
      }
    } else {
      if (TW_MR_DATA_NACK != TW_STATUS) {
        result = TW_STATUS;
        *value = 0;
      }
    }
  }
  return result;
}

uint8_t I2C::waitCompletion()
{
  uint8_t result = SUCCESS;

  if (BTR_I2C_IO_TIMEOUT_MS > 0) {
    uint32_t start_ms = Time::millis();

    while (bit_is_clear(TWCR, TWINT)) {
      if ((Time::millis() - start_ms) >= BTR_I2C_IO_TIMEOUT_MS) {
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

#endif // BTR_I2C_ENABLED > 0
