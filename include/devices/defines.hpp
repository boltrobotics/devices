// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Defines_hpp_
#define _btr_Defines_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

namespace btr
{

#if defined(stm32)

// The LED in BlackPill use GPIOB, GPIO12, in BluePill GPIOC, GPIO13
#ifndef BTR_BUILTIN_LED_CLK
#define BTR_BUILTIN_LED_CLK RCC_GPIOB
#endif
#ifndef BTR_BUILTIN_LED_PORT
#define BTR_BUILTIN_LED_PORT GPIOB
#endif
#ifndef BTR_BUILTIN_LED_PIN
#define BTR_BUILTIN_LED_PIN GPIO12
#endif

#endif // defined(stm32)

#ifndef BTR_TX_Q_DELAY
#define BTR_TX_Q_DELAY 10
#endif
#ifndef BTR_USART_IO_TIMEOUT
#define BTR_USART_IO_TIMEOUT 100
#endif
#ifndef BTR_USART_RX_BUFF_SIZE
#define BTR_USART_RX_BUFF_SIZE 64
#endif
#ifndef BTR_USART_TX_BUFF_SIZE
#define BTR_USART_TX_BUFF_SIZE 64
#endif

#ifndef BTR_USART1_BAUD 
#define BTR_USART1_BAUD 115200
#endif
#ifndef BTR_USART2_BAUD 
#define BTR_USART2_BAUD 115200
#endif
#ifndef BTR_USART3_BAUD 
#define BTR_USART3_BAUD 115200
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

#ifndef BTR_USART1_PARITY 
#define BTR_USART1_PARITY   'N'
#endif
#ifndef BTR_USART2_PARITY 
#define BTR_USART2_PARITY   'N'
#endif
#ifndef BTR_USART3_PARITY 
#define BTR_USART3_PARITY   'N'
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

#ifndef BTR_USART1_RTS 
#define BTR_USART1_RTS 0
#endif
#ifndef BTR_USART2_RTS 
#define BTR_USART2_RTS 0
#endif
#ifndef BTR_USART3_RTS 
#define BTR_USART3_RTS 0
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
