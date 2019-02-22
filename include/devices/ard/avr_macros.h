// Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// BV was renamed to _BV. _BV is not in C standard. For portability, use old-style BV macro.
#define BV(bit) (1 << bit)

// TWI
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)

// AVR /////////////////////////////////////////////////////////////////////////////////////////////
#if defined (BOARD_AVR)

#include <util/twi.h>
#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>

// Macros
#define set_reg(reg, val)     (reg = val)
#define set_bit(sfr, bit)     (_SFR_BYTE(sfr) |= BV(bit))  // old sbi()
#define clear_bit(sfr, bit)   (_SFR_BYTE(sfr) &= ~BV(bit)) // old cbi()
#define toggle_bit(sfr, bit)  (_SFR_BYTE(sfr) ^= BV(bit))

// STM32 ///////////////////////////////////////////////////////////////////////////////////////////
#elif defined (BOARD_STM32)

// x86 /////////////////////////////////////////////////////////////////////////////////////////////
#else

#define millis() 1
#define micros() 1
#define loop_until_bit_is_set(sfr, bit)
#define loop_until_bit_is_clear(sfr, bit)
#define bit_is_set(sfr, bit)      0
#define bit_is_clear(sfr, bit)    0
#define set_reg(reg, val) (void)(val);
#define set_bit(sfr, bit)
#define clear_bit(sfr, bit)
#define toggle_bit(sfr, bit)

#define F_CPU                     0x0
#define TWCR                      0x0
#define TWEN                      0x0
#define TWEA                      0x0
#define TWSR                      0x0
#define TWDR                      0x0
#define TWINT                     0x0
#define TWSTA                     0x0
#define TWSTO                     0x0
#define TW_START                  0x08
#define TW_REP_START              0x10
#define TW_MT_SLA_ACK             0x18
#define TW_MT_SLA_NACK            0x20
#define TW_MT_DATA_ACK            0x28
#define TW_MT_DATA_NACK           0x30
#define TW_MT_ARB_LOST            0x38
#define TW_MR_ARB_LOST            0x38
#define TW_MR_SLA_ACK             0x40
#define TW_MR_SLA_NACK            0x48
#define TW_MR_DATA_ACK            0x50
#define TW_MR_DATA_NACK           0x58
#define TW_ST_SLA_ACK             0xA8
#define TW_ST_ARB_LOST_SLA_ACK    0xB0
#define TW_ST_DATA_ACK            0xB8
#define TW_ST_DATA_NACK           0xC0
#define TW_ST_LAST_DATA           0xC8
#define TW_SR_SLA_ACK             0x60
#define TW_SR_ARB_LOST_SLA_ACK    0x68
#define TW_SR_GCALL_ACK           0x70
#define TW_SR_ARB_LOST_GCALL_ACK  0x78
#define TW_SR_DATA_ACK            0x80
#define TW_SR_DATA_NACK           0x88
#define TW_SR_GCALL_DATA_ACK      0x90
#define TW_SR_GCALL_DATA_NACK     0x98
#define TW_SR_STOP                0xA0
#define TW_NO_INFO                0xF8
#define TW_BUS_ERROR              0x00
#define TW_STATUS_MASK            0x0 // (_BV(TWS7)|_BV(TWS6)|_BV(TWS5)|_BV(TWS4)|_BV(TWS3))
#define TW_STATUS                 (TWSR & TW_STATUS_MASK)

#endif // BOARD_ comparison
