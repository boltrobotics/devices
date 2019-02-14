// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Usart_hpp_
#define _btr_Usart_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "devices/hardware_stream.hpp"

namespace btr
{

/**
 * The class provides an interface to USB device on stm32f103c8t6 microcontroller.
 */
class Usart : public HardwareStream
{
public:

// LIFECYCLE

  /**
   * Dtor.
   */
  ~Usart() = default;

// OPERATIONS

  /**
   * Create new or return single instance.
   *
   * @return an instance of a USART device. The instance may need to be initialized.
   */
  static Usart* instance(uint32_t usart_id);

  /**
   * @see HardwareStream::isOpen
   */
  virtual bool isOpen() const override;

  /**
   * @see HardwareStream::open
   */
  virtual int open(bool init_gpio, uint32_t priority) override;

  /**
   * @see HardwareStream::close
   */
  virtual void close() override;

  /**
   * @see HardwareStream::setTimeout
   */
  virtual int setTimeout(uint32_t timeout);

  /**
   * @see HardwareStream::available
   */
  virtual int available() override;

  /**
   * @see HardwareStream::flush
   */
  virtual int flush(DirectionType queue_selector) override;

  /**
   * @see HardwareStream::send
   */
  virtual int send(char ch) override;

  /**
   * @see HardwareStream::send
   */
  virtual int send(const char* buff) override;

  /**
   * @see HardwareStream::send
   */
  virtual int send(const char* buff, uint16_t bytes) override;

  /**
   * @see HardwareStream::recv
   */
  virtual int recv() override;

  /**
   * @see HardwareStream::recv
   */
  virtual int recv(char* buff, uint16_t bytes) override;

private:

// LIFECYCLE

  /**
   * Ctor.
   */
  Usart();

// ATTRIBUTES

  TaskHandle_t tx_h_;
  TaskHandle_t rx_h_;
  QueueHandle_t tx_q_;
  QueueHandle_t rx_q_;
};

} // namespace btr

#endif // _btr_Usart_hpp_
