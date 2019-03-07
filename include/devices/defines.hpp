// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

/** @file */

#ifndef _btr_Defines_hpp_
#define _btr_Defines_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

namespace btr
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// General {

#if BTR_AVR > 0
//! @cond Suppress_Doxygen_Warning
#include <avr/io.h>
//! @endcond

#define BV(bit)                 (1 << bit)
#define set_bit(sfr, bit)       (_SFR_BYTE(sfr) |= BV(bit))  // old sbi()
#define clear_bit(sfr, bit)     (_SFR_BYTE(sfr) &= ~BV(bit)) // old cbi()
#define toggle_bit(sfr, bit)    (_SFR_BYTE(sfr) ^= BV(bit))  

#endif // BTR_AVR > 0

// } General

////////////////////////////////////////////////////////////////////////////////////////////////////
// Time {

#if BTR_AVR > 0 || BTR_ARD > 0

#ifndef BTR_TIME_ENABLED 
#define BTR_TIME_ENABLED        1
#endif

#endif // Platform

// } Time

////////////////////////////////////////////////////////////////////////////////////////////////////
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

// LED pins apply only to Arduino boards. An AVR board is assumed to have the same set-up.
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P_)
#define BTR_BUILTIN_LED_PORT  PORTB
#define BTR_BUILTIN_LED_PIN   PB5
#define BTR_BUILTIN_LED_DDR   DDRB
#elif defined(__AVR__ATmega1280__) || defined(__AVR_ATmega2560__)
#define BTR_BUILTIN_LED_PORT  PORTB
#define BTR_BUILTIN_LED_PIN   PB7
#define BTR_BUILTIN_LED_DDR   DDRB
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

////////////////////////////////////////////////////////////////////////////////////////////////////
// USART {

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

#define BTR_USART_NO_DATA       0x010000
#define BTR_USART_OVERFLOW_ERR  0x020000
#define BTR_USART_PARITY_ERR    0x040000
#define BTR_USART_OVERRUN_ERR   0x080000
#define BTR_USART_FRAME_ERR     0x100000
#define BTR_USART_TIMEDOUT_ERR  0x110000

#if BTR_STM32 > 0

#define BTR_USART_MIN_ID        0
#define BTR_USART_MAX_ID        2

#elif BTR_AVR > 0 || BTR_ARD > 0

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P_)
#define BTR_USART_MIN_ID        0
#define BTR_USART_MAX_ID        0
#elif defined(__AVR__ATmega1280__) || defined(__AVR_ATmega2560__)
#define BTR_USART_MIN_ID        0
#define BTR_USART_MAX_ID        3
#endif // AVR board

#endif // Platform

#ifndef BTR_USB0_ENABLED
#define BTR_USB0_ENABLED      0
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
#ifndef BTR_USART_CR_BUFF_SIZE
#define BTR_USART_CR_BUFF_SIZE  64
#endif

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

// } USART

} // namespace btr

#endif // _btr_Defines_hpp_
