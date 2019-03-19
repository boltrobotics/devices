// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// PROJECT INCLUDES
#include "devices/i2c.hpp"

#if BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

I2C::I2C(uint32_t dev_id)
  :
    dev_id_(dev_id),
    buff_()
{
}

//============================================= OPERATIONS =========================================

uint32_t I2C::scan()
{
  uint32_t rc = BTR_IO_ENOERR;
  uint32_t count = 0;

  for (uint8_t addr = 0; addr < BTR_I2C_SCAN_MAX; addr++) {
    rc = start(addr, BTR_I2C_READ);

    if (BTR_IO_OK(rc)) {
      ++count;
    } else {
      break;
    }
    stop();
  }
  return (rc | count);
}

uint32_t I2C::write(uint8_t addr, uint8_t reg, const uint8_t* buff, uint8_t bytes)
{
  uint32_t rc = start(addr, BTR_I2C_WRITE);
  uint32_t count = 0;

  if (BTR_IO_OK(rc)) {
    rc = sendByte(reg);

    if (BTR_IO_OK(rc)) {
      ++count;

      for (uint8_t i = 0; i < bytes; i++) {
        rc = sendByte(buff[i]);

        if (BTR_IO_ERR(rc)) {
          break;
        }
        ++count;
      }
    }
    stop();
  }
  return (rc | count);
}

uint32_t I2C::read(uint8_t addr, uint8_t reg, uint8_t* buff, uint8_t count)
{
  uint32_t rc = start(addr, BTR_I2C_WRITE);

  if (BTR_IO_OK(rc)) {
    rc = sendByte(reg);

    if (BTR_IO_OK(rc)) {
      // Stop then start in read (I2C restart).
      stop();
      rc = read(addr, buff, count, false);
    }
    stop();
  }
  return rc;
}

uint32_t I2C::read(uint8_t addr, uint8_t* buff, uint8_t bytes, bool stop_comm)
{
  uint32_t rc = start(addr, BTR_I2C_READ);
  uint32_t count = 0;

  if (BTR_IO_OK(rc)) {
    if (bytes == 0) {
      bytes++;
    }

    uint8_t nack = bytes - 1;

    for (uint8_t i = 0; i < bytes; i++) {
      if (i != nack) {
        rc = receiveByte(true, &buff[i]);
      } else {
        rc = receiveByte(false, &buff[i]);
      }
      if (BTR_IO_ERR(rc)) {
        break;
      }
    }
    if (stop_comm) {
      stop();
    }
  }
  return (rc | count);
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

void I2C::reset()
{
  close();
  open();
}

} // namespace btr

#endif // BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0
