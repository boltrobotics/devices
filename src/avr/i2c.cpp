// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// PROJECT INCLUDES
#include "devices/defines.hpp"
#include "devices/i2c.hpp"  // class implemented
#include "devices/time.hpp"
#include "i2c_private.hpp"

#if BTR_ARD > 0
#include <Arduino.h>
#endif

// SYSTEM INCLDUES
#include <util/twi.h>
#include <util/delay.h>

#if BTR_I2C0_ENABLED > 0

namespace btr
{

static I2C i2c_0(0);

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

// static
I2C* I2C::instance(uint32_t dev_id, bool open)
{
  (void) dev_id;

  if (open) {
    i2c_0.open();
  }
  return &i2c_0;
}

void I2C::open()
{
  //setPullups(true);
  setSpeed(BTR_I2C_FAST_SPEED);

  // Enable TWI operation and interface.
  set_reg(TWCR, BV(TWEN) | BV(TWEA));
  open_ = true;
}

void I2C::close()
{
  // Switch off TWI and terminate all ongoing transmissions
  set_reg(TWCR, 0);
  open_ = false;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

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

uint32_t I2C::start(uint8_t addr, uint8_t rw)
{
  set_reg(TWCR, BV(TWINT) | BV(TWSTA) | BV(TWEN));
  uint32_t rc = waitBusy();

  if (is_ok(rc)) {
    if (TW_MT_ARB_LOST == TW_STATUS) {
      rc = BTR_DEV_ESTART;
      reset();
      return rc;
    }

    uint8_t addr_rw = 0;

    if (BTR_I2C_READ == rw) {
      addr_rw = BTR_I2C_READ_ADDR(addr);
    } else {
      addr_rw = BTR_I2C_WRITE_ADDR(addr);
    }

    set_reg(TWDR, addr_rw);
    set_reg(TWCR, BV(TWINT) | BV(TWEN));

    rc = waitBusy();

    if (is_ok(rc)) {
      if (TW_MT_SLA_NACK == TW_STATUS || TW_MR_SLA_NACK == TW_STATUS) {
        rc = BTR_DEV_ENOACK;
        stop();
      } else if ((TW_MT_SLA_ACK != TW_STATUS) && (TW_MR_SLA_ACK != TW_STATUS)) {
        rc = BTR_DEV_ESTART;
        reset();
      }
    }
  }
  return rc;
}

uint32_t I2C::stop()
{
  uint32_t rc = BTR_DEV_ENOERR;
  set_reg(TWCR, BV(TWINT) | BV(TWEN) | BV(TWSTO));

  if (BTR_I2C_IO_TIMEOUT_MS > 0) {
    uint32_t start_ms = MILLIS();

    while (bit_is_set(TWCR, TWSTO)) {
      if (TM_DIFF(MILLIS(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_DEV_ETIMEOUT;
        reset();
        break;
      }
    }
  } else {
    loop_until_bit_is_clear(TWCR, TWSTO);
  }
  return rc;
}

uint32_t I2C::sendByte(uint8_t val)
{
  set_reg(TWDR, val);
  set_reg(TWCR, BV(TWINT) | BV(TWEN));

  uint32_t rc = waitBusy();

  if (is_ok(rc)) {
    if (TW_MT_DATA_NACK == TW_STATUS) {
      rc = BTR_DEV_ESENDBYTE;
      stop();
    } else if (TW_MT_DATA_ACK != TW_STATUS) {
      rc = BTR_DEV_ESENDBYTE;
      reset();
    }
  }
  return rc;
}

uint32_t I2C::receiveByte(bool expect_ack, uint8_t* val)
{
  set_reg(TWCR, BV(TWINT) | BV(TWEN));

  if (expect_ack) {
    set_reg(TWCR, TWCR | BV(TWEA));
  }

  uint32_t rc = waitBusy();

  if (is_ok(rc)) {
    if (TW_MT_ARB_LOST == TW_STATUS) {
      rc = BTR_DEV_ERECVBYTE;
      reset();
      return rc;
    }
  }

  *val = TWDR;

  if (expect_ack) {
    if (TW_MR_DATA_ACK != TW_STATUS) {
      rc = BTR_DEV_ENOACK;
    }
  } else {
    if (TW_MR_DATA_NACK != TW_STATUS) {
      rc = BTR_DEV_ENONACK;
    }
  }
  return rc;
}

uint32_t I2C::waitBusy()
{
  uint32_t rc = BTR_DEV_ENOERR;

  if (BTR_I2C_IO_TIMEOUT_MS > 0) {
    uint32_t start_ms = MILLIS();

    while (bit_is_clear(TWCR, TWINT)) {
      if (TM_DIFF(MILLIS(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_DEV_ETIMEOUT;
        reset();
        break;
      }
    }
  } else {
    loop_until_bit_is_set(TWCR, TWINT);
  }
  return rc;
}

} // namespace btr

#endif // BTR_I2C0_ENABLED > 0
