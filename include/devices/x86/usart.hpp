// Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Usart_hpp__
#define _btr_Usart_hpp__

// SYSTEM INCLUDES
#include <boost/asio.hpp>

// PROJECT INCLUDES
#include "devices/defines.hpp"

namespace bio = boost::asio;

namespace btr
{

/**
 * The class provides a send/receive interface to a serial port.
 */
class Usart
{
public:

// LIFECYCLE

  Usart();

  /**
   * Close the port.
   */
  ~Usart();

// OPERATIONS

  /**
   * (Re)-configure USART parameters.
   *
   * @param port - serial IO port name (e.g., /dev/ttyS0)
   * @param baud_rate - baud rate. It must be one of values specified by in termios.h
   *  @see http://man7.org/linux/man-pages/man3/termios.3.html
   * @param data_bits
   * @param parity - @see ParityType
   * @param timeout - serial operation timeout in milliseconds
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
   * Schedule a timer for asynchronous operation execution.
   */
  void timeAsyncOpr();

  /**
   * Asynchronous operation completion handler.
   *
   * @param error - the error if any occured
   * @param bytes_transferred - the number of bytes transferred
   */
  void onOprComplete(const boost::system::error_code& err, size_t bytes_transferred);

  /**
   * Timer callback.
   *
   * @param error - the error if any
   */
  void onTimeout(const boost::system::error_code& error);

// ATTRIBUTES

  bio::io_service     io_service_;
  bio::serial_port    serial_port_;
  bio::deadline_timer timer_;
  size_t              bytes_transferred_;
  const char*         port_name_;
  uint32_t            baud_rate_;
  uint8_t             data_bits_;
  uint8_t             parity_;
  size_t              timeout_;

}; // class Usart

} // namespace btr

#endif // _btr_Usart_hpp__
