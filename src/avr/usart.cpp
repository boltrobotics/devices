// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <avr/io.h>
#include <util/atomic.h>

// PROJECT INCLUDES
#include "devices/avr/usart.hpp"  // class implemented

#if BTR_USART1_ENABLED > 0 || BTR_USART2_ENABLED > 0 || \
    BTR_USART3_ENABLED > 0 || BTR_USART4_ENABLED > 0

extern "C" {

#if BTR_USART_USE_2X > 0
#define BAUD_CALC(BAUD) (((F_CPU) + 4UL * (BAUD)) /  (8UL * (BAUD)) - 1UL)
#else
#define BAUD_CALC(BAUD) (((F_CPU) + 8UL * (BAUD)) / (16UL * (BAUD)) - 1UL)
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
// Register bits {

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

// UCSRnA (status)
#define RXC       RXC0    // Bit 7. Receive complete
#define TXC       TXC0    // Bit 6. Transmit complete
#define UDRE      UDRE0   // Bit 5. Transmit buffer empty
#define FE        FE0     // Bit 4. Frame error
#define DOR       DOR0    // Bit 3. Data OverRun
#define UPE       UPE0    // Bit 2. Parity error
#define U2X       U2X0    // Bit 1. Double transmission speed
#define MPCM      MPCM0   // Bit 0. Multi-processor communication mode
// UCSRnB (control 1)
#define RXCIE     RXCIE0  // Bit 7. Receive complete interrupt enable
#define TXCIE     TXCIE0  // Bit 6. Transmit complete interrupt enable
#define UDRIE     UDRIE0  // Bit 5. Transmit buffer empty interrupt enable
#define RXEN      RXEN0   // Bit 4. Receive enable
#define TXEN      TXEN0   // Bit 3. Transmit enable
#define UCSZ2     UCSZ02  // Bit 2. Character size 2
// UCSRnC (control 2)
#define UCSZ1     UCSZ01  // Bit 2. Character size 1
#define UCSZ0     UCSZ00  // Bit 1. Character size 0
#endif // __AVR

// } Register bits

////////////////////////////////////////////////////////////////////////////////////////////////////
// RS485 {

#if BTR_RTS_ENABLED > 0

#define RTS_PIN   PB0
#define RTS_DDR   DDRB
#define RTS_PORT  PORTB

#define RTS_INIT \
  do { \
    RTS_DDR |= _BV(RTS_PIN); \
    RTS_PORT &= ~(_BV(RTS_PIN)); \
  } while (0);

#define RTS_HIGH \
  do { \
    RTS_PORT |= _BV(RTS_PIN); \
  } while (0);

#define RTS_LOW \
  do { \
    RTS_PORT &= ~(_BV(RTS_PIN)); \
  } while (0);

#endif // BTR_RTS_ENABLED

// } RS485

////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware I/O {

// ISRs: http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html

#if defined (__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#if BTR_USART1_ENABLED > 0
ISR(USART_RX_vect)
{
  btr::Usart::onRecv(1);
}
ISR(USART_UDRE_vect)
{
  btr::Usart::onSend(1);
}
#endif

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#if BTR_USART1_ENABLED > 0
ISR(USART0_RX_vect)
{
  btr::Usart::onRecv(1);
}
ISR(USART0_UDRE_vect)
{
  btr::Usart::onSend(1);
}
#endif
#if BTR_USART2_ENABLED > 0
ISR(USART1_RX_vect)
{
  btr::Usart::onRecv(2);
}
ISR(USART1_UDRE_vect)
{
  btr::Usart::onSend(2);
}
#endif
#if BTR_USART3_ENABLED > 0
ISR(USART2_RX_vect)
{
  btr::Usart::onRecv(3);
}
ISR(USART2_UDRE_vect)
{
  btr::Usart::onSend(3);
}
#endif
#if BTR_USART4_ENABLED > 0
ISR(USART3_RX_vect)
{
  btr::Usart::onRecv(4);
}
ISR(USART3_UDRE_vect)
{
  btr::Usart::onSend(4);
}
#endif // BTR_USARTx
#endif // __AVR_

// } Hardware I/O

} // extern "C"

namespace btr
{

#if BTR_USART1_ENABLED > 0
#if defined(UBRRH) && defined(UBRRL)
static Usart usart_1(1, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UCSRC, &UDR);
#else
static Usart usart_1(1, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0);
#endif // UBRRH && UBRRL
#endif // BTR_USART1_ENABLED

#if BTR_USART2_ENABLED > 0
static Usart usart_2(2, &UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UCSR1C, &UDR1);
#endif

#if BTR_USART3_ENABLED > 0
static Usart usart_3(3, &UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UCSR2C, &UDR2);
#endif

#if BTR_USART4_ENABLED > 0
static Usart usart_4(4, &UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UCSR3C, &UDR3);
#endif

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

Usart::Usart(
    uint8_t id,
    volatile uint8_t* ubrr_h,
    volatile uint8_t* ubrr_l,
    volatile uint8_t* ucsr_a,
    volatile uint8_t* ucsr_b,
    volatile uint8_t* ucsr_c,
    volatile uint8_t* udr
    )
  :
    id_(id),
    ubrr_h_(ubrr_h),
    ubrr_l_(ubrr_l),
    ucsr_a_(ucsr_a),
    ucsr_b_(ucsr_b),
    ucsr_c_(ucsr_c),
    udr_(udr),
    rx_error_(0),
    rx_head_(0),
    rx_tail_(0),
    tx_head_(0),
    tx_tail_(0),
    rx_buff_(),
    tx_buff_()
{
} 

//============================================= OPERATIONS =========================================

// static
Usart* Usart::instance(uint32_t usart_id)
{
  switch (usart_id) {
#if BTR_USART1_ENABLED > 0
    case 1:
      return &usart_1;
#endif
#if BTR_USART2_ENABLED > 0
    case 2:
      return &usart_2;
#endif
#if BTR_USART3_ENABLED > 0
    case 3:
      return &usart_3;
#endif
#if BTR_USART4_ENABLED > 0
    case 4:
      return &usart_4;
#endif
    default:
      return nullptr;
  }
}

bool Usart::isOpen()
{
  return (bit_is_set(*ucsr_b_, TXEN) || bit_is_set(*ucsr_b_, RXEN));
}

int Usart::open()
{
  if (true == isOpen()) {
    return 0;
  }

  uint32_t baud_rate = 0;
  uint8_t data_bits = 8;
  uint8_t stop_bits = 1;
  char parity = 'N';

  switch (id_) {
    case 1:
      baud_rate = BTR_USART1_BAUD;
      data_bits = BTR_USART1_DATA_BITS,
      stop_bits = BTR_USART1_STOP_BITS,
      parity = BTR_USART1_PARITY;
      break;
    case 2:
      baud_rate = BTR_USART2_BAUD;
      data_bits = BTR_USART2_DATA_BITS,
      stop_bits = BTR_USART2_STOP_BITS,
      parity = BTR_USART2_PARITY;
      break;
    case 3:
      baud_rate = BTR_USART3_BAUD;
      data_bits = BTR_USART3_DATA_BITS,
      stop_bits = BTR_USART3_STOP_BITS,
      parity = BTR_USART3_PARITY;
      break;
    case 4:
      baud_rate = BTR_USART4_BAUD;
      data_bits = BTR_USART4_DATA_BITS,
      stop_bits = BTR_USART4_STOP_BITS,
      parity = BTR_USART4_PARITY;
      break;
    default:
      return -1;
  }

  uint16_t baud = BAUD_CALC(baud_rate);

#if BTR_USART_USE_2X > 0
  *ucsr_a_ = (1 << U2X);
#else
  *ucsr_a_ &= ~(1 << U2X);
#endif

  *ubrr_h_ = (uint8_t) (baud >> 8);
  *ubrr_l_ = (uint8_t) (baud & 0xFF);
  *ucsr_c_ = (uint8_t) BTR_USART_CONFIG(parity, stop_bits, data_bits);

  set_bit(*ucsr_b_, TXEN);
  set_bit(*ucsr_b_, RXEN);
  set_bit(*ucsr_b_, RXCIE);
  clear_bit(*ucsr_b_, UDRIE);

  return 0;
}

void Usart::close()
{
  flush(DirectionType::OUT);
  clear_bit(*ucsr_b_, TXEN);
  clear_bit(*ucsr_b_, RXEN);
  clear_bit(*ucsr_b_, RXCIE);
  clear_bit(*ucsr_b_, UDRIE);
  rx_head_ = rx_tail_;
}

// static
void Usart::onRecv(uint8_t usart_id)
{
  btr::Usart* dev = btr::Usart::instance(usart_id);

  if (nullptr != dev) {
    dev->rx_error_ = (*(dev->ucsr_a_) & ((1 << FE) | (1 << DOR) | (1 << UPE)));
    uint16_t head_next = (dev->rx_head_ + 1) % BTR_USART_RX_BUFF_SIZE;

    if (head_next != dev->rx_tail_) {
      dev->rx_buff_[dev->rx_head_] = *(dev->udr_);
      dev->rx_head_ = head_next;
    } else {
      dev->rx_error_ |= (BTR_USART_OVERFLOW_ERR >> 8);
    }
    LED_TOGGLE();
  }
}

// static
void Usart::onSend(uint8_t usart_id)
{
  btr::Usart* dev = btr::Usart::instance(usart_id);

  if (nullptr != dev) {
    uint8_t ch = dev->tx_buff_[dev->tx_tail_];
    dev->tx_tail_ = (dev->tx_tail_ + 1) % BTR_USART_TX_BUFF_SIZE;
    *(dev->udr_) = ch;

    if (dev->tx_head_ == dev->tx_tail_) {
      // Disable transmit buffer empty interrupt since there is no more data to send.
      clear_bit(*(dev->ucsr_b_), UDRIE);
    }
    LED_TOGGLE();
  }
}

int Usart::available()
{
  uint16_t bytes = BTR_USART_RX_BUFF_SIZE + rx_head_ - rx_tail_;
  return (bytes % BTR_USART_RX_BUFF_SIZE);
}

int Usart::flush(DirectionType queue_selector)
{
  (void) queue_selector;

  while (bit_is_set(*ucsr_b_, UDRIE) || bit_is_clear(*ucsr_a_, TXC)) {
    if (bit_is_clear(SREG, SREG_I) && bit_is_set(*ucsr_b_, UDRIE)) {
      if (bit_is_set(*ucsr_a_, UDRE)) {
        // Call manually since global interrupts are disabled.
        onSend(id_);
      }
    }
  }
  return 0;
}

int Usart::send(char ch, bool drain)
{
  uint16_t head_next = (tx_head_ + 1) % BTR_USART_TX_BUFF_SIZE;

  while (head_next == tx_tail_) {
    if (bit_is_clear(SREG, SREG_I)) {
      if (bit_is_set(*ucsr_a_, UDRE)) {
        onSend(id_);
      }
    } else {
      // Wait while data is being drained from tx_buff_.
    }
  }

  tx_buff_[head_next] = ch;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    tx_head_ = head_next;
    set_bit(*ucsr_b_, UDRIE);
  }

  if (true == drain) {
    flush(DirectionType::OUT);
  }
  return 0;
}

int Usart::send(const char* buff, bool drain)
{
  int rc = 0;

  while (*buff) {
    if (0 != (rc = send(*buff++, false))) {
      break;
    }
  }

  if (0 == rc && true == drain) {
    flush(DirectionType::OUT);
  }
  return rc;
}

int Usart::send(const char* buff, uint32_t bytes, bool drain)
{
  int rc = 0;

  while (bytes-- > 0) {
    if (0 != (rc = send(*buff, false))) {
      break;
    }
    ++buff;
  }

  if (0 == rc && true == drain) {
    flush(DirectionType::OUT);
  }
  return rc;
}

uint16_t Usart::recv()
{
  if (rx_head_ != rx_tail_) {
    uint8_t ch = rx_buff_[rx_tail_];  
    rx_tail_ = (rx_tail_ + 1 ) % BTR_USART_RX_BUFF_SIZE;

    uint16_t rc = (rx_error_ << 8);
    rx_error_ = 0;

    return (rc + ch);
  }
  return BTR_USART_NO_DATA;
}

uint16_t Usart::recv(char* buff, uint16_t bytes)
{
  uint16_t errors = 0;
  uint16_t byte_idx = 0;

  while (bytes > 0 && (byte_idx + 1) < bytes) {
    uint16_t ch = recv();

    if (BTR_USART_NO_DATA & ch) {
      // TODO delay and use timeout
      continue;
    }
    errors |= (ch & 0xFF00);
    buff[byte_idx++] = ch;
  }

  buff[byte_idx] = 0;
  return errors;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // BTR_USARTn_ENABLED
