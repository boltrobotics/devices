// Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_UsartTermios_hpp__
#define _btr_UsartTermios_hpp__

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "devices/defines.hpp"

namespace btr
{

/**
 * The class provides a send/receive interface to a serial port.
 */
class UsartTermios
{
public:

// LIFECYCLE

  /**
   * Ctor.
   *
   */
  UsartTermios();

  /**
   * Dtor.
   */
  ~UsartTermios();

// OPERATIONS

  /**
   * Open serial port.
   *
   * @param port - serial IO port name (e.g., /dev/ttyS0)
   * @param baud_rate - baud rate. It must be one of values specified by in termios.h
   *  @see http://man7.org/linux/man-pages/man3/termios.3.html
   * @param data_bits
   * @param parity - @see ParityType
   * @param timeout - serial operation timeout in milliseconds
   * @return 0 on success, -1 on failure
   */
  void configure(
      const char* port_name,
      uint32_t baud_rate,
      uint8_t data_bits,
      uint8_t parity,
      uint32_t timeout = BTR_USART_IO_TIMEOUT);

  /**
   * @see HardwareStream::open
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

private:

// OPERATIONS

  /**
   * Convert numeric BAUD rate to termios-specific one.
   *
   * @param num - the number baud rate
   * @return termios baud rate
   */
  static int getNativeBaud(int num);

// ATTRIBUTES

  const char* port_name_;
  uint32_t baud_rate_;
  uint8_t data_bits_;
  uint8_t parity_;
  int port_;
  size_t timeout_;

}; // class UsartTermios

} // namespace btr

#endif // _btr_UsartTermios_hpp__
