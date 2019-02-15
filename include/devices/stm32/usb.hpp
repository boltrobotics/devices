// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Usb_hpp_
#define _btr_Usb_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "devices/defines.hpp"

namespace btr
{

/**
 * The class provides an interface to USB device on stm32f103c8t6 microcontroller.
 */
class Usb
{
public:

// LIFECYCLE

  Usb() = default;
  ~Usb() = default;

// OPERATIONS

  /**
   * Create new or return single instance.
   *
   * @return an instance of a USB device. The instance may need to be initialized.
   */
  static Usb* instance();

  /**
   * @see HardwareStream::isOpen
   */
  static bool isOpen();

  /**
   * @see HardwareStream::init
   */
  static int open();

  /**
   * @see HardwareStream::shutdown
   */
  static void close();

  /**
   * @see HardwareStream::setTimeout
   */
  static int setTimeout(uint32_t timeout);

  /**
   * @see HardwareStream::available
   */
  static int available();

  /**
   * @see HardwareStream::flush
   */
  static int flush(DirectionType queue_selector);

  /**
   * @see HardwareStream::send
   */
  static int send(char ch, bool drain = false);

  /**
   * @see HardwareStream::send
   */
  static int send(const char* buff, bool drain = false);

  /**
   * @see HardwareStream::send
   */
  static int send(const char* buff, uint32_t bytes, bool drain = false);

  /**
   * @see HardwareStream::recv
   */
  static int recv();

  /**
   * @see HardwareStream::recv
   */
  static int recv(char* buff, uint32_t bytes);
};

} // namespace btr

#endif // _btr_Usb_hpp_
