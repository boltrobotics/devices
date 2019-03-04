// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Usart_hpp_
#define _btr_Usart_hpp_

// SYSTEM INCLUDES
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// PROJECT INCLUDES
#include "devices/defines.hpp"

namespace btr
{

/**
 * The class provides an interface to USART devices on STM32F103C8T6 microcontroller.
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
   * @param usart_id - number from 1 to 3 for USARTx ports
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
   * @return 0 on success, -1 if queue was full
   */
  int send(char ch);

  /**
   * Send data from buff up to null character.
   *
   * @param buff - data buffer
   * @return 0 on success, -1 if queue was full
   */
  int send(const char* buff);

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @return 0 on success, -1 if queue was full
   */
  int send(const char* buff, uint32_t bytes);

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

  /**
   * @return reference to transmit queue
   */
  TaskHandle_t& txh();

  /**
   * @return reference to transmit queue
   */
  QueueHandle_t& txq();

private:

// ATTRIBUTES

  uint8_t id_;
  TaskHandle_t tx_h_;
  QueueHandle_t tx_q_;
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

inline TaskHandle_t& Usart::txh()
{
  return tx_h_;
}

inline QueueHandle_t& Usart::txq()
{
  return tx_q_;
}

} // namespace btr

#endif // _btr_Usart_hpp_
