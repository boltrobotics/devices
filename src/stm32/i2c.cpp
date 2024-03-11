// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <cstddef>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

// PROJECT INCLUDES
#include "devices/i2c.hpp"  // class partially implemented
#include "devices/time.hpp"
#include "FreeRTOS.h"
#include "task.h"

#if BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

namespace btr
{

#if BTR_I2C0_ENABLED > 0
static I2C i2c_0(I2C1);
#endif
#if BTR_I2C1_ENABLED > 0
static I2C i2c_1(I2C2);
#endif

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

// static
I2C* I2C::instance(uint32_t id, bool open)
{
  switch (id) {
#if BTR_I2C0_ENABLED > 0
    case 0:
      if (open) {
        i2c_0.open();
      }
      return &i2c_0;
#endif
#if BTR_I2C1_ENABLED > 0
    case 1:
      if (open) {
        i2c_1.open();
      }
      return &i2c_1;
#endif
    default:
      set_status(dev::status(), BTR_DEV_EINVAL);
      return nullptr;
  }
}

void I2C::open()
{
	rcc_periph_clock_enable(RCC_GPIOB);
	//rcc_periph_clock_enable(RCC_AFIO);

  if (I2C1 == bus_handle_) {
    rcc_periph_clock_enable(RCC_I2C1);
	  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);
    gpio_set(GPIOB, GPIO6 | GPIO7);
    gpio_primary_remap(0, 0); 
  } else {
    rcc_periph_clock_enable(RCC_I2C2);
	  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO10 | GPIO11);
    gpio_set(GPIOB, GPIO10 | GPIO11);
  }

	i2c_peripheral_disable(bus_handle_);
	//i2c_reset(bus_handle_); OBSOLETE?
	I2C_CR1(bus_handle_) &= ~I2C_CR1_STOP; // Clear stop.

  // Set bus speed
  i2c_set_clock_frequency(bus_handle_, 36);
  i2c_speeds speed = (uint32_t(BTR_I2C_SPEED) > 100000 ? i2c_speed_fm_400k : i2c_speed_sm_100k);
  // Set mode, ccr, and trise.
  i2c_set_speed(bus_handle_, speed, (rcc_apb1_frequency / 1000000));

	i2c_set_dutycycle(bus_handle_, I2C_CCR_DUTY_DIV2);
	//i2c_set_own_7bit_slave_address(bus_handle_, 0x23);
	i2c_peripheral_enable(bus_handle_);
  open_ = true;
}

void I2C::close()
{
	i2c_peripheral_disable(bus_handle_);
  open_ = false;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

uint32_t I2C::start(uint8_t addr, uint8_t rw)
{
  uint32_t rc = waitBusy();

  if (is_ok(rc)) {
    // Clear acknowledge failure.
    I2C_SR1(bus_handle_) &= ~I2C_SR1_AF;
		// Disable stop generation.
    i2c_clear_stop(bus_handle_);

    if (BTR_I2C_READ == rw) {
      i2c_enable_ack(bus_handle_);
    }

    i2c_send_start(bus_handle_);
    uint32_t start_ms = Time::millis();

    while (false == (
          (I2C_SR1(bus_handle_) & I2C_SR1_SB) &&
          (I2C_SR2(bus_handle_) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
    {
      if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_DEV_ETIMEOUT;
        reset();
        return rc;
      }
      taskYIELD();
    }

    i2c_send_7bit_address(bus_handle_, addr, rw);
    start_ms = Time::millis();

    // Wait until the address is sent and ACK received, or either NACK or time-out occurs.
    while (false == (I2C_SR1(bus_handle_) & I2C_SR1_ADDR)) {
      // Check if ACK Failed.
      if (I2C_SR1(bus_handle_) & I2C_SR1_AF) {
        rc = BTR_DEV_ENOACK;
        stop();
        return rc;
      }
      if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_DEV_ETIMEOUT;
        stop();
        return rc;
      }
      taskYIELD();
    }
    // Clear status register.
    I2C_SR2(bus_handle_);
  }
  return 0;
}

uint32_t I2C::stop()
{
  i2c_send_stop(bus_handle_);
  return BTR_DEV_ENOERR;
}

uint32_t I2C::sendByte(uint8_t val)
{
	i2c_send_data(bus_handle_, val);

  uint32_t rc = BTR_DEV_ENOERR;
  uint32_t start_ms = Time::millis();

  // Wait for send to finish or time out.
	while (false == (I2C_SR1(bus_handle_) & (I2C_SR1_BTF))) {
    if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
      rc = BTR_DEV_ETIMEOUT;
      stop();
      break;
    }
		taskYIELD();
	}
  return rc;
}

uint32_t I2C::receiveByte(bool expect_ack, uint8_t* val)
{
	if (false == expect_ack) {
		i2c_disable_ack(bus_handle_);
  } else {
    i2c_enable_ack(bus_handle_);
  }

  uint32_t rc = BTR_DEV_ENOERR;
  uint32_t start_ms = Time::millis();

	while (false == (I2C_SR1(bus_handle_) & I2C_SR1_RxNE)) {
    if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
      rc = BTR_DEV_ETIMEOUT;
      reset();
      return rc;
    }
		taskYIELD();
	}
	
  *val = i2c_get_data(bus_handle_);
  return rc;
}

uint32_t I2C::waitBusy()
{
  uint32_t rc = BTR_DEV_ENOERR;

  if (BTR_I2C_IO_TIMEOUT_MS > 0) {
    uint32_t start_ms = Time::millis();

    while (I2C_SR2(bus_handle_) & I2C_SR2_BUSY) {
      if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_DEV_ETIMEOUT;
        reset();
        break;
      }
      taskYIELD();
    }
  } else {
    while (I2C_SR2(bus_handle_) & I2C_SR2_BUSY) {
      taskYIELD();
    }
  }
  return rc;
}

} // namespace btr

#endif // BTR_I2C_ENABLED > 0
