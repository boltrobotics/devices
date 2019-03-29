// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

/** @file */

#ifndef _devices_btr_Defines_hpp_
#define _devices_btr_Defines_hpp_

// SYSTEM INCLUDES
#define _STDC_FORMAT_MACROS
#include <inttypes.h>

namespace btr
{

//==================================================================================================
// General {

#if BTR_AVR > 0 || BTR_ARD > 0
//! @cond Suppress_Doxygen_Warning
#include <avr/io.h>
//! @endcond

// BV was renamed to _BV. _BV is not in C standard. For portability, use old-style BV macro.
#define BV(bit)                 (1 << bit)
#define set_bit(sfr, bit)       (_SFR_BYTE(sfr) |= BV(bit))  // old sbi()
#define clear_bit(sfr, bit)     (_SFR_BYTE(sfr) &= ~BV(bit)) // old cbi()
#define toggle_bit(sfr, bit)    (_SFR_BYTE(sfr) ^= BV(bit))  
#define set_reg(reg, val)       (reg = val)

#endif // BTR_AVR > 0 || BTR_ARD > 0

// } General

//==================================================================================================
// Time {

#ifndef BTR_TIME_ENABLED 
#if BTR_STM32 > 0 || BTR_AVR > 0 || BTR_X86 > 0
#define BTR_TIME_ENABLED        1
#elif BTR_ARD > 0
#define BTR_TIME_ENABLED        0
#endif // #if BTR_STM32 > 0 || BTR_AVR > 0 || BTR_X86 > 0
#endif

#if BTR_ARD > 0
#define MILLIS()                (millis())
#define SEC()                   (MILLIS() / 1000)
#define TMDIFF(a,b)             (((UINT32_MAX + a - b) % UINT32_MAX))
#elif BTR_AVR > 0 || BTR_STM32 > 0
#define MILLIS()                (Time::millis())
#define SEC()                   (Time::sec())
#define TM_DIFF(a,b)            (Time::diff(a, b))
#endif

/** Check if timeout is greater than 0, if so, check if time window has expired. */
#define IS_TIMEOUT(timeout_ms, start_ms) \
  (timeout_ms > 0 && (TM_DIFF(MILLIS(), start_ms) > timeout_ms))

// } Time

//==================================================================================================
// LED {

#if BTR_STM32 > 0

// The LED in BlackPill use GPIOB, GPIO12, in BluePill GPIOC, GPIO13
#ifndef BTR_BUILTIN_LED_CLK
#define BTR_BUILTIN_LED_CLK   RCC_GPIOB
#endif
#ifndef BTR_BUILTIN_LED_PORT
#define BTR_BUILTIN_LED_PORT  GPIOB
#endif
#ifndef BTR_BUILTIN_LED_PIN
#define BTR_BUILTIN_LED_PIN   GPIO12
#endif

#define LED_ENABLE() \
  do { \
    rcc_periph_clock_enable(BTR_BUILTIN_LED_CLK); \
    gpio_set_mode(BTR_BUILTIN_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, \
        GPIO_CNF_OUTPUT_PUSHPULL, BTR_BUILTIN_LED_PIN); \
  } while (0);

#define LED_ON() (gpio_clear(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN))
#define LED_OFF() (gpio_set(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN))
#define LED_TOGGLE() (gpio_toggle(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN))

#elif BTR_AVR > 0 || BTR_ARD > 0

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#define BTR_BUILTIN_LED_PORT  PORTB
#define BTR_BUILTIN_LED_PIN   PB5
#define BTR_BUILTIN_LED_DDR   DDRB
#elif defined(__AVR__ATmega1280__) || defined(__AVR_ATmega2560__)
#define BTR_BUILTIN_LED_PORT  PORTB
#define BTR_BUILTIN_LED_PIN   PB7
#define BTR_BUILTIN_LED_DDR   DDRB
#else
#warning AVR device is not supported
#endif

#define LED_ON()  \
  do { \
    BTR_BUILTIN_LED_DDR  |= (1 << BTR_BUILTIN_LED_PIN); \
    BTR_BUILTIN_LED_PORT |= (1 << BTR_BUILTIN_LED_PIN); \
  } while (0);
#define LED_OFF() \
  do { \
    BTR_BUILTIN_LED_DDR  &= ~(1 << BTR_BUILTIN_LED_PIN); \
    BTR_BUILTIN_LED_PORT &= ~(1 << BTR_BUILTIN_LED_PIN); \
  } while (0);
#define LED_TOGGLE() \
  do { \
    BTR_BUILTIN_LED_DDR  ^= (1 << BTR_BUILTIN_LED_PIN); \
    BTR_BUILTIN_LED_PORT ^= (1 << BTR_BUILTIN_LED_PIN); \
  } while (0);
#endif // stm32, avr, ard

// } LED


//==================================================================================================
// Status {

#define BTR_DEV_ENOERR          0x00000000
#define BTR_DEV_ENODATA         0x00010000
#define BTR_DEV_EOVERFLOW       0x00020000
#define BTR_DEV_EPARITY         0x00040000
#define BTR_DEV_EOVERRUN        0x00080000
#define BTR_DEV_EFRAME          0x00100000
#define BTR_DEV_ETIMEOUT        0x00110000
#define BTR_DEV_ENOTOPEN        0x00120000
#define BTR_DEV_ESTART          0x00140000
#define BTR_DEV_ESENDBYTE       0x00180000
#define BTR_DEV_ERECVBYTE       0x00200000
#define BTR_DEV_ENOACK          0x00210000
#define BTR_DEV_ENONACK         0x00220000
#define BTR_DEV_EINVAL          0x00240000
#define BTR_DEV_EINIT           0x00280000

#ifndef BTR_STATUS_ENABLED
#define BTR_STATUS_ENABLED      1
#endif

namespace dev
{
/** Provide this module's status accumulator or nullptr if BTR_STATUS_ENABLED is 0. */
uint32_t* status();
}

// } Status

//==================================================================================================
// I2C {

/** On STM32F103C8T6, I2C0 refers to SCL1/SDA1, I2C1 to SCL2/SDA2. On AVR, only I2C0 is used. */
#ifndef BTR_I2C0_ENABLED
#define BTR_I2C0_ENABLED      0
#endif
#ifndef BTR_I2C1_ENABLED
#define BTR_I2C1_ENABLED      0
#endif

#if BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

//------------------------------------------------------------------------------

#if BTR_AVR > 0 || BTR_ARD > 0
#include <util/twi.h>

/** I2C write operation. */
#define BTR_I2C_WRITE               TW_WRITE
/** I2C read operation. */
#define BTR_I2C_READ                TW_READ

#elif BTR_STM32 > 0
#include <libopencm3/stm32/i2c.h>

/** Define I2C write operation. */
#define BTR_I2C_WRITE               I2C_WRITE 
/** Define I2C read operation. */
#define BTR_I2C_READ                I2C_READ

#else
/** Define I2C write operation. */
#define BTR_I2C_WRITE               0 
/** Define I2C read operation. */
#define BTR_I2C_READ                1

#endif // BTR_AVR > 0 || BTR_ARD > 0 || BTR_STM32 > 0

//------------------------------------------------------------------------------

#ifndef BTR_I2C_IO_TIMEOUT_MS
#define BTR_I2C_IO_TIMEOUT_MS       100
#endif
/** Warn if time-out is zero. */
#if BTR_I2C_IO_TIMEOUT_MS == 0
#error "BTR_I2C_IO_TIMEOUT_MS is 0"
#endif
/** Wehn set to 1, bus speed is set to 400kHz, otherwise bus speed is set to 100kHz. */
#ifndef BTR_I2C_FAST_SPEED
#define BTR_I2C_FAST_SPEED          0
#endif
/** Maximum number of devices to scan. Absolute maximum is UINT8_MAX. */
#ifndef BTR_I2C_SCAN_MAX
#define BTR_I2C_SCAN_MAX            128
#endif

#define BTR_I2C_WRITE_ADDR(addr)    (addr << 1)
#define BTR_I2C_READ_ADDR(addr)     ((addr << 1) + 1)

#endif // BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

// } I2C

//==================================================================================================
// USART/USB {

/** Supported parity types. */
typedef enum
{
  NONE,
  ODD,
  EVEN
} ParityType;

#define PARITY_FROM_INT(v) ((v==1 ? ParityType::ODD : (v==2 ? ParityType::EVEN : ParityType::NONE)))

/** Supported stop bit types. */
typedef enum
{
  ONE,
  ONEPOINTFIVE,
  TWO
} StopBitsType;

#define TO_STOP_BITS(v) ((v==2 ? StopBitsType::TWO : StopBitsType::ONE))

/** Data flow direction. */
typedef enum
{
  IN,
  OUT,
  INOUT
} DirectionType;

#if BTR_STM32 > 0

#define BTR_USART_MIN_ID        0
#define BTR_USART_MAX_ID        2

#elif BTR_AVR > 0 || BTR_ARD > 0

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#define BTR_USART_MIN_ID        0
#define BTR_USART_MAX_ID        0
#elif defined(__AVR__ATmega1280__) || defined(__AVR_ATmega2560__)
#define BTR_USART_MIN_ID        0
#define BTR_USART_MAX_ID        3
#endif // AVR board

#endif // Platform

#ifndef BTR_USB0_ENABLED
#define BTR_USB0_ENABLED        0
#endif

#ifndef BTR_USART0_ENABLED
#define BTR_USART0_ENABLED      0
#endif
#ifndef BTR_USART1_ENABLED
#define BTR_USART1_ENABLED      0
#endif
#ifndef BTR_USART2_ENABLED
#define BTR_USART2_ENABLED      0
#endif
#ifndef BTR_USART3_ENABLED
#define BTR_USART3_ENABLED      0
#endif

#ifndef BTR_USART_RX_DELAY_US
#define BTR_USART_RX_DELAY_US   5000
#endif
#ifndef BTR_USART_TX_DELAY_US
#define BTR_USART_TX_DELAY_US   5000
#endif
#define BTR_USART_TX_DELAY_MS   (BTR_USART_TX_DELAY_US / 1000) 
#define BTR_USART_RX_DELAY_MS   (BTR_USART_RX_DELAY_US / 1000) 

#ifndef BTR_USART_IO_TIMEOUT_MS
#define BTR_USART_IO_TIMEOUT_MS 100
#endif
#ifndef BTR_USART_RX_TIMEOUT_MS
#define BTR_USART_RX_TIMEOUT_MS BTR_USART_IO_TIMEOUT_MS
#endif
#ifndef BTR_USART_TX_TIMEOUT_MS
#define BTR_USART_TX_TIMEOUT_MS BTR_USART_IO_TIMEOUT_MS
#endif

#ifndef BTR_USART_RX_BUFF_SIZE
#define BTR_USART_RX_BUFF_SIZE  64
#endif
#ifndef BTR_USART_TX_BUFF_SIZE
#define BTR_USART_TX_BUFF_SIZE  64
#endif
#ifndef BTR_USART_IR_BUFF_SIZE
#define BTR_USART_IR_BUFF_SIZE  16
#endif
/** USB doesn't work with lower values like 64 bytes. Keep it at 128. TODO look into it */
//#ifndef BTR_USART_CR_BUFF_SIZE
#define BTR_USART_CR_BUFF_SIZE  128
//#endif

#ifndef BTR_USART_USE_2X
#define BTR_USART_USE_2X        1
#endif

#ifndef BTR_USART0_BAUD
#define BTR_USART0_BAUD         115200
#endif
#ifndef BTR_USART1_BAUD
#define BTR_USART1_BAUD         115200
#endif
#ifndef BTR_USART2_BAUD
#define BTR_USART2_BAUD         115200
#endif
#ifndef BTR_USART3_BAUD
#define BTR_USART3_BAUD         115200
#endif

#ifndef BTR_USART0_DATA_BITS
#define BTR_USART0_DATA_BITS    8
#endif
#ifndef BTR_USART1_DATA_BITS
#define BTR_USART1_DATA_BITS    8
#endif
#ifndef BTR_USART2_DATA_BITS
#define BTR_USART2_DATA_BITS    8
#endif
#ifndef BTR_USART3_DATA_BITS
#define BTR_USART3_DATA_BITS    8
#endif

#ifndef BTR_USART0_PARITY
#define BTR_USART0_PARITY       ParityType::NONE
#endif
#ifndef BTR_USART1_PARITY
#define BTR_USART1_PARITY       ParityType::NONE
#endif
#ifndef BTR_USART2_PARITY
#define BTR_USART2_PARITY       ParityType::NONE
#endif
#ifndef BTR_USART3_PARITY
#define BTR_USART3_PARITY       ParityType::NONE
#endif

#ifndef BTR_USART0_STOP_BITS
#define BTR_USART0_STOP_BITS    StopBitsType::ONE
#endif
#ifndef BTR_USART1_STOP_BITS
#define BTR_USART1_STOP_BITS    StopBitsType::ONE
#endif
#ifndef BTR_USART2_STOP_BITS
#define BTR_USART2_STOP_BITS    StopBitsType::ONE
#endif
#ifndef BTR_USART3_STOP_BITS
#define BTR_USART3_STOP_BITS    StopBitsType::ONE
#endif

#ifndef BTR_USART0_RTS
#define BTR_USART0_RTS          0
#endif
#ifndef BTR_USART1_RTS
#define BTR_USART1_RTS          0
#endif
#ifndef BTR_USART2_RTS
#define BTR_USART2_RTS          0
#endif
#ifndef BTR_USART3_RTS
#define BTR_USART3_RTS          0
#endif

#ifndef BTR_USART0_CTS
#define BTR_USART0_CTS          0
#endif
#ifndef BTR_USART1_CTS
#define BTR_USART1_CTS          0
#endif
#ifndef BTR_USART2_CTS
#define BTR_USART2_CTS          0
#endif
#ifndef BTR_USART3_CTS
#define BTR_USART3_CTS          0
#endif

#define BTR_USART_CONFIG(parity, stop_bits, data_bits) \
  ( (parity == ParityType::NONE ? 0 : (parity == ParityType::EVEN ? 32 : 48)) \
    + (stop_bits == StopBitsType::ONE ? 0 : 8) + (2 * (data_bits - 5)) )

// } USART/USB

//==================================================================================================
// VL53L0X {

/** When enabling VL53L0X, also enable BTR_I2C0_ENABLED. */
#ifndef BTR_VL53L0X_ENABLED
#define BTR_VL53L0X_ENABLED         0
#endif

#ifndef BTR_VL53L0X_PORT_I2C
#if BTR_STM32 > 0
#define BTR_VL53L0X_PORT_I2C             1
#else
#define BTR_VL53L0X_PORT_I2C             0
#endif
#endif // BTR_VL53L0X_PORT_I2C_DEV

#ifndef BTR_VL53L0X_ADDR_DFLT
#define BTR_VL53L0X_ADDR_DFLT       0b0101001
#endif
#ifndef BTR_VL53L0X_COMPENSATE_MM
#define BTR_VL53L0X_COMPENSATE_MM   -10
#endif
#ifndef BTR_VL53L0X_TIMEOUT_MS
#define BTR_VL53L0X_TIMEOUT_MS      1000
#endif
#ifndef BTR_VL53L0X_BUDGET_US
#define BTR_VL53L0X_BUDGET_US       32000
#endif
#ifndef BTR_VL53L0X_LIMIT_MCPS_MIN
#define BTR_VL53L0X_LIMIT_MCPS_MIN  0
#endif
#ifndef BTR_VL53L0X_LIMIT_MCPS_MAX
#define BTR_VL53L0X_LIMIT_MCPS_MAX  511.99
#endif

/** Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs from register. */
#define BTR_VL53L0X_DECODE_VCSEL(val) (((val) + 1) << 1)
/** Encode VCSEL pulse period register value from period in PCLKs. */
#define BTR_VL53L0X_ENCODE_VCSEL(val) (((val) >> 1) - 1)
/** Calculate macro period in *nanoseconds* from VCSEL period in PCLKs.
 * PLL_period_ps = 1655; macro_period_vclks = 2304. */
#define BTR_VL53L0X_CALC_PERIOD(val) ((((uint32_t) 2304 * (val) * 1655) + 500) / 1000)

// } VL53L0X

} // namespace btr

#endif // _devices_btr_Defines_hpp_
