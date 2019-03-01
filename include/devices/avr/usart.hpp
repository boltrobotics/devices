// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Usart_hpp_
#define _btr_Usart_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "devices/defines.hpp"

#define BTR_USART_NO_DATA       0x010000
#define BTR_USART_OVERFLOW_ERR  0x020000
#define BTR_USART_PARITY_ERR    0x040000
#define BTR_USART_OVERRUN_ERR   0x080000
#define BTR_USART_FRAME_ERR     0x100000
#define BTR_USART_TIMEDOUT_ERR  0x110000

namespace btr
{

/**
 * The class provides an interface to USART devices on a microcontroller.
 */
class Usart
{
public:

// LIFECYCLE

  /**
   * Ctor.
   */
  Usart(
      uint8_t id,
      volatile uint8_t* ubrr_h,
      volatile uint8_t* ubrr_l,
      volatile uint8_t* ucsr_a,
      volatile uint8_t* ucsr_b,
      volatile uint8_t* ucsr_c,
      volatile uint8_t* udr);

// OPERATIONS

  /**
   * Create new or return an instance of a USART identified by usart_id.
   *
   * @param usart_id - port number of a USART per the platform this code is built for
   * @return an instance of a USART device. The instance may need to be initialized.
   */
  static Usart* instance(uint32_t usart_id);

  /**
   * Check if device is open.
   *
   * @return true if port is open, false otherwise
   */
  bool isOpen();

  /**
   * Initialize the device.
   */
  int open();

  /**
   * Stop the device, queues, clocks.
   */
  void close();

  /**
   * Check if there is data in receive queue.
   *
   * @return bytes available on the serial port or -1 if failed to retrieve the value
   */
  int available();

  /**
   * Flush pending, not-transmitted and non-read, data on the serial port.
   *
   * @param queue_selector - one of:
   *  IN - flushes data received but not read.
   *  OUT - flushes data written but not transmitted.
   *  INOUT - flushes both data received but not read, and data written but not transmitted.
   */
  int flush(DirectionType queue_selector);

  /**
   * Handle received data.
   */
  void onRecv();

  /**
   * Process event of the data just sent.
   */
  void onSend();

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @param timeout - maximum microseconds to wait to submit one character
   * @return bits from 16 up to 24 contain error code(s), lower 16 bits contains the number of bytes
   *  submitted
   */
  uint32_t send(const char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_TX_TIMEOUT_MS);

  /**
   * Receive a number of bytes and store in the buffer.
   *
   * @param buff - buffer to store received data
   * @param bytes - the number of bytes to receive
   * @param timeout - maximum microseconds to wait to receive one character
   * @return bits from 16 up to 24 contain error code(s), lower 16 bits contains the number of bytes
   *  received
   */
  uint32_t recv(char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_RX_TIMEOUT_MS);

// ATTRIBUTES

  uint8_t id_;
  volatile uint8_t* ubrr_h_;
  volatile uint8_t* ubrr_l_;
  volatile uint8_t* ucsr_a_;
  volatile uint8_t* ucsr_b_;
  volatile uint8_t* ucsr_c_;
  volatile uint8_t* udr_;
  volatile uint8_t rx_error_;

#if BTR_USART_RX_BUFF_SIZE > 256
  volatile uint16_t rx_head_;
  volatile uint16_t rx_tail_;
#else
  volatile uint8_t rx_head_;
  volatile uint8_t rx_tail_;
#endif
#if BTR_USART_TX_BUFF_SIZE > 256
  volatile uint16_t tx_head_;
  volatile uint16_t tx_tail_;
#else
  volatile uint8_t tx_head_;
  volatile uint8_t tx_tail_;
#endif
  uint8_t rx_buff_[BTR_USART_RX_BUFF_SIZE];
  uint8_t tx_buff_[BTR_USART_TX_BUFF_SIZE];
  bool enable_flush_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // _btr_Usart_hpp_
