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
 * The class provides an interface to USB device on STM32F103C8T6 microcontroller.
 */
class Usb
{
public:

// LIFECYCLE

  Usb() = default;
  ~Usb() = default;

// OPERATIONS

  /**
   * @return an instance of a USB device. The instance may need to be initialized.
   */
  static Usb* instance();

  /**
   * Check if device is open.
   *
   * @return true if port is open, false otherwise
   */
  static bool isOpen();

  /**
   * Initialize the device.
   */
  static int open();

  /**
   * Stop the device, queues, clocks.
   */
  static void close();

  /**
   * @param timeout - timeout in milliseconds
   * @return 0 on success, -1 on failure
   */
  static int setTimeout(uint32_t timeout);

  /**
   * Check if there is data in receive queue.
   *
   * @return bytes available on the serial port or -1 if failed to retrieve the value
   */
  static int available();

  /**
   * Flush pending, not-transmitted and non-read, data on the serial port.
   *
   * @param queue_selector - one of:
   *  IN - flushes data received but not read.
   *  OUT - flushes data written but not transmitted.
   *  INOUT - flushes both data received but not read, and data written but not transmitted.
   */
  static int flush(DirectionType queue_selector);

  /**
   * Send a single character.
   *
   * @param ch - the character to send
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  static int send(char ch, bool drain = false);

  /**
   * Send data from buff up to null character.
   *
   * @param buff - data buffer
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  static int send(const char* buff, bool drain = false);

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  static int send(const char* buff, uint32_t bytes, bool drain = false);

  /**
   * Receive a single character.
   *
   * @return the received character or -1 on error
   */
  static int recv();

  /**
   * Receive a number of bytes and store in the buffer.
   *
   * @param buff - buffer to store received data
   * @param bytes - the number of bytes to receive
   * @return bytes received or -1 on error
   */
  static int recv(char* buff, uint32_t bytes);
};

} // namespace btr

#endif // _btr_Usb_hpp_
