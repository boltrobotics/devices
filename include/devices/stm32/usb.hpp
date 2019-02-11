// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Usb_hpp_
#define _btr_Usb_hpp_

// SYSTEM INCLUDES

namespace btr
{

/**
 * The class provides an interface to USB device on stm32f103c8t6 microcontroller.
 */
class Usb
{
public:

// LIFECYCLE

  Usb() = delete;
  ~Usb() = delete;

// OPERATIONS

  /**
   * Initialize USB device and I/O queues.
   *
   * @param init_gpio - initialize GPIO
   * @param priority - USB task priority
   */
  static int init(bool init_gpio, uint32_t priority);

  /**
   * Shut down the USB device.
   */
  static void shutdown();

  /**
   * Send a single character.
   *
   * @param ch - the character to send
   * @return 0 on success, -1 if queue was full
   */
  static int send(char ch);

  /**
   * Send data from buff up to null character.
   *
   * @param buff - data buffer
   * @return 0 on success, -1 if queue was full
   */
  static int send(const char* buff);

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @return 0 on success, -1 if queue was full
   */
  static int send(const char* buff, uint16_t bytes);

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
  static int recv(char* buff, uint16_t bytes);

  /**
   * Check if there is data in receive queue.
   *
   * @return 1 - there is data, 0 - no data, -1 - failed to check
   */
  static int peek();
};

} // namespace btr

#endif // _btr_Usb_hpp_
