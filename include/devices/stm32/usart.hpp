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
 * The class provides an interface to USB device on stm32f103c8t6 microcontroller.
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
   * @param usart_id - number from 1 to 3
   * @param initialize - pass true to initialize usart unless already initialized
   * @return an instance of a USART device. The instance may need to be initialized.
   */
  static Usart* instance(uint32_t usart_id);

  /**
   * @see HardwareStream::isOpen
   */
  bool isOpen();

  /**
   * @see HardwareStream::open
   */
  int open();

  /**
   * @see HardwareStream::close
   */
  void close();

  /**
   * @see HardwareStream::setTimeout
   */
  int setTimeout(uint32_t timeout);

  /**
   * @see HardwareStream::available
   */
  int available();

  /**
   * @see HardwareStream::flush
   */
  int flush(DirectionType queue_selector);

  /**
   * @see HardwareStream::send
   */
  int send(char ch, bool drain = false);

  /**
   * @see HardwareStream::send
   */
  int send(const char* buff, bool drain = false);

  /**
   * @see HardwareStream::send
   */
  int send(const char* buff, uint32_t bytes, bool drain = false);

  /**
   * @see HardwareStream::recv
   */
  int recv();

  /**
   * @see HardwareStream::recv
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
   * @return USART id
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
