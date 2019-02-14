// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Usb_hpp_
#define _btr_Usb_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "devices/hardware_stream.hpp"

namespace btr
{

/**
 * The class provides an interface to USB device on stm32f103c8t6 microcontroller.
 */
class Usb : public HardwareStream
{
public:

// LIFECYCLE

  /**
   * Dtor.
   */
  ~Usb() = default;

// OPERATIONS

  /**
   * Create new or return single instance.
   *
   * @return an instance of a USB device. The instance may need to be initialized.
   */
  static Usb* instance();

  /**
   * @see HardwareStream::init
   */
  virtual int open(bool init_gpio, uint32_t priority) override;

  /**
   * @see HardwareStream::shutdown
   */
  virtual void close() override;

  /**
   * @see HardwareStream::setTimeout
   */
  virtual int setTimeout(uint32_t timeout) = 0;

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
  Usb();
};

} // namespace btr

#endif // _btr_Usb_hpp_
