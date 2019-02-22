// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Usart_hpp_
#define _btr_Usart_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "devices/defines.hpp"

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
  Usart(uint8_t id);

  /**
   * Dtor.
   */
  ~Usart() = default;

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
   * @param timeout - timeout in milliseconds
   * @return 0 on success, -1 on failure
   */
  int setTimeout(uint32_t timeout);

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
   * Send a single character.
   *
   * @param ch - the character to send
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  int send(char ch, bool drain = false);

  /**
   * Send data from buff up to null character.
   *
   * @param buff - data buffer
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  int send(const char* buff, bool drain = false);

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  int send(const char* buff, uint32_t bytes, bool drain = false);

  /**
   * Receive a single character.
   *
   * @return the received character or -1 on error
   */
  int recv();

  /**
   * Receive a number of bytes and store in the buffer.
   *
   * @param buff - buffer to store received data
   * @param bytes - the number of bytes to receive
   * @return bytes received or -1 on error
   */
  int recv(char* buff, uint32_t bytes);

  /**
   * @return head position of receive circular buffer
   */
  volatile uint16_t& rxHead();

  /**
   * @return tail position of receive circular buffer
   */
  volatile uint16_t& rxTail();

  /**
   * @return receive circular buffer
   */
  uint8_t* rxBuff();

  /**
   * @return USART ID, [1, 3]
   */
  uint8_t id();

private:

// ATTRIBUTES

  uint8_t id_;
  volatile uint16_t rx_head_;
  volatile uint16_t rx_tail_;
  uint8_t rx_buff_[BTR_USART_RX_BUFF_SIZE];
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

inline volatile uint16_t& Usart::rxHead()
{
  return rx_head_;
}

inline volatile uint16_t& Usart::rxTail()
{
  return rx_head_;
}

inline uint8_t* Usart::rxBuff()
{
  return rx_buff_;
}

inline uint8_t Usart::id()
{
  return id_;
}

} // namespace btr

#endif // _btr_Usart_hpp_
