// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Defines_hpp_
#define _btr_Defines_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

namespace btr
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// LED {
#if defined(stm32)

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

#define LED_ON() (gpio_clear(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN))
#define LED_OFF() (gpio_set(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN))
#define LED_TOGGLE() (gpio_toggle(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN))

#elif defined(avr) || defined(ard)

// LED pins apply only to Arduino boards. An AVR board is assumed to have the same set-up.
#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328P_)
#define BTR_BUILTIN_LED_PORT  PORTB
#define BTR_BUILTIN_LED_PIN   PB5
#define BTR_BUILTIN_LED_DDR   DDRB
#elif defined(__AVR__ATmega1280__) || defined(__AVR_ATmega2560__)
#define BTR_BUILTIN_LED_PORT  PORTB
#define BTR_BUILTIN_LED_PIN   PB7
#define BTR_BUILTIN_LED_DDR   DDRB
#endif

#define LED_ON()  (BTR_BUILTIN_LED_DDR  |= (1<<BTR_BUILTIN_LED_PIN); \
                   BTR_BUILTIN_LED_PORT |= (1<<BTR_BUILTIN_LED_PIN))
#define LED_OFF() (BTR_BUILTIN_LED_DDR  &= ~(1<<BTR_BUILTIN_LED_PIN); \
                   BTR_BUILTIN_LED_PORT &= ~(1<<BTR_BUILTIN_LED_PIN))
#define LED_TOGGLE() (BTR_BUILTIN_LED_DDR  ^= (1<<BTR_BUILTIN_LED_PIN); \
                      BTR_BUILTIN_LED_PORT ^= (1<<BTR_BUILTIN_LED_PIN))
#endif // stm32, avr, ard

// } LED

////////////////////////////////////////////////////////////////////////////////////////////////////
// USART {

#if defined(stm32)
#define BTR_USART_MIN_ID 1
#define BTR_USART_MAX_ID 3
#elif defined(avr) || defined(ard)
#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328P_)
#define BTR_USART_MIN_ID 1
#define BTR_USART_MAX_ID 1
#elif defined(__AVR__ATmega1280__) || defined(__AVR_ATmega2560__)
#define BTR_USART_MIN_ID 1
#define BTR_USART_MAX_ID 4
#endif
#endif

#ifndef BTR_USART1_ENABLED
#define BTR_USART1_ENABLED 0
#endif
#ifndef BTR_USART2_ENABLED
#define BTR_USART2_ENABLED 0
#endif
#ifndef BTR_USART3_ENABLED
#define BTR_USART3_ENABLED 0
#endif
#ifndef BTR_USART4_ENABLED
#define BTR_USART4_ENABLED 0
#endif

#ifndef BTR_USART_TX_Q_DELAY
#define BTR_USART_TX_Q_DELAY    10
#endif
#ifndef BTR_USART_IO_TIMEOUT
#define BTR_USART_IO_TIMEOUT    100
#endif

#ifndef BTR_USART_RX_BUFF_SIZE
#define BTR_USART_RX_BUFF_SIZE  64
#endif
#ifndef BTR_USART_TX_BUFF_SIZE
#define BTR_USART_TX_BUFF_SIZE  64
#endif

#define BTR_USART1_NAME "USART1"
#define BTR_USART2_NAME "USART2"
#define BTR_USART3_NAME "USART3"
#define BTR_USART4_NAME "USART4"

#ifndef BTR_USART1_BAUD
#define BTR_USART1_BAUD 115200
#endif
#ifndef BTR_USART2_BAUD
#define BTR_USART2_BAUD 115200
#endif
#ifndef BTR_USART3_BAUD
#define BTR_USART3_BAUD 115200
#endif
#ifndef BTR_USART4_BAUD
#define BTR_USART4_BAUD 115200
#endif

#ifndef BTR_USART1_DATA_BITS
#define BTR_USART1_DATA_BITS 8
#endif
#ifndef BTR_USART2_DATA_BITS
#define BTR_USART2_DATA_BITS 8
#endif
#ifndef BTR_USART3_DATA_BITS
#define BTR_USART3_DATA_BITS 8
#endif
#ifndef BTR_USART4_DATA_BITS
#define BTR_USART4_DATA_BITS 8
#endif

#ifndef BTR_USART1_PARITY
#define BTR_USART1_PARITY   'N'
#endif
#ifndef BTR_USART2_PARITY
#define BTR_USART2_PARITY   'N'
#endif
#ifndef BTR_USART3_PARITY
#define BTR_USART3_PARITY   'N'
#endif
#ifndef BTR_USART4_PARITY
#define BTR_USART4_PARITY   'N'
#endif

#ifndef BTR_USART1_STOP_BITS
#define BTR_USART1_STOP_BITS 1
#endif
#ifndef BTR_USART2_STOP_BITS
#define BTR_USART2_STOP_BITS 1
#endif
#ifndef BTR_USART3_STOP_BITS
#define BTR_USART3_STOP_BITS 1
#endif
#ifndef BTR_USART4_STOP_BITS
#define BTR_USART4_STOP_BITS 1
#endif

#ifndef BTR_USART1_RTS
#define BTR_USART1_RTS 0
#endif
#ifndef BTR_USART2_RTS
#define BTR_USART2_RTS 0
#endif
#ifndef BTR_USART3_RTS
#define BTR_USART3_RTS 0
#endif
#ifndef BTR_USART4_RTS
#define BTR_USART4_RTS 0
#endif

#ifndef BTR_USART1_CTS
#define BTR_USART1_CTS 0
#endif
#ifndef BTR_USART2_CTS
#define BTR_USART2_CTS 0
#endif
#ifndef BTR_USART3_CTS
#define BTR_USART3_CTS 0
#endif
#ifndef BTR_USART4_CTS
#define BTR_USART4_CTS 0
#endif

// } USART

typedef enum
{
  NONE,
  ODD,
  EVEN
} ParityType;

typedef enum
{
  IN,
  OUT,
  INOUT
} DirectionType;

} // namespace btr

#endif // _btr_Defines_hpp_
