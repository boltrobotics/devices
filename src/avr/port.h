/*
 * FreeModbus Libary: AVR Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *   - Initial version + ATmega168 support
 * Modfications Copyright (C) 2006 Tran Minh Hoang:
 *   - ATmega8, ATmega16, ATmega32 support
 *   - RS485 support for DS75176
 * Modfications Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
 *   - ATmega168 USART vectors
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#ifndef _PORT_H
#define _PORT_H

/* ----------------------- Platform includes --------------------------------*/

#include <avr/io.h>
#include <avr/interrupt.h>

/* ----------------------- Defines ------------------------------------------*/
#define	INLINE                      
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

#define ENTER_CRITICAL_SECTION( )   cli()
#define EXIT_CRITICAL_SECTION( )    sei()

#define assert( x )

typedef char    BOOL;

typedef unsigned char UCHAR;
typedef char    CHAR;

typedef unsigned short USHORT;
typedef short   SHORT;

typedef unsigned long ULONG;
typedef long    LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__,(uint32_t) __LINE__))
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif // USE_FULL_ASSERT

/* ----------------------- AVR platform specifics ---------------------------*/

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define UCSRB           UCSR0B
#define TXEN            TXEN0
#define RXEN            RXEN0
#define RXCIE           RXCIE0
#define TXCIE           TXCIE0
#define UDRE            UDRE0
#define UBRR            UBRR0
#define UCSRC           UCSR0C
#define UPM1            UPM01
#define UPM0            UPM00
#define UCSZ0           UCSZ00
#define UCSZ1           UCSZ01
#define UDR             UDR0
#define USART_TX_VECT   USART0_TX_vect
#define USART_RX_VECT   USART0_RX_vect
#define USART_UDRE_VECT USART0_UDRE_vect

#elif defined (__AVR_ATmega169__)

#define SIG_UART_TRANS  SIG_USART_TRANS

#elif defined (__AVR_ATmega8__)
#define UBRR            UBRRL
#define TCCR1C          TCCR1A  /* dummy */
#define TIMSK1          TIMSK
#define TIFR1           TIFR
#define SIG_USART_DATA  SIG_UART_DATA
#define SIG_USART_RECV  SIG_UART_RECV

#elif defined (__AVR_ATmega16__)
#define UBRR            UBRRL
#define TCCR1C          TCCR1A  /* dummy */
#define TIMSK1          TIMSK
#define TIFR1           TIFR

#elif defined (__AVR_ATmega32__)
#define UBRR            UBRRL
#define TCCR1C          TCCR1A  /* dummy */
#define TIMSK1          TIMSK
#define TIFR1           TIFR

#elif defined (__AVR_ATmega128__)
#define UCSRB           UCSR0B
#define UBRR            UBRR0L
#define UDR             UDR0
#define TIMSK1          TIMSK
#define TIFR1           TIFR
#define SIG_UART_TRANS  SIG_USART0_TRANS
#define SIG_USART_DATA  SIG_USART0_DATA
#define SIG_USART_RECV  SIG_USART0_RECV
#define UCSZ0           UCSZ00
#define UCSZ1           UCSZ01
#define UPM0            UPM00
#define UPM1            UPM01
#define UCSRC           UCSR0C

#endif

/* ----------------------- RS485 specifics ----------------------------------*/
#ifdef  RTS_ENABLE

#define RTS_PIN         PB0
#define RTS_DDR         DDRB
#define RTS_PORT        PORTB

#define RTS_INIT        \
    do { \
        RTS_DDR |= _BV( RTS_PIN ); \
        RTS_PORT &= ~( _BV( RTS_PIN ) ); \
    } while( 0 );

#define RTS_HIGH        \
    do { \
        RTS_PORT |= _BV( RTS_PIN ); \
    } while( 0 );

#define RTS_LOW         \
    do { \
        RTS_PORT &= ~( _BV( RTS_PIN ) ); \
    } while( 0 );

#endif

#endif
