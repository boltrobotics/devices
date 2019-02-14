// Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Usart_hpp__
#define _btr_Usart_hpp__

// SYSTEM INCLUDES
#include <boost/asio.hpp>

// PROJECT INCLUDES
#include "devices/defines.hpp"
#include "devices/hardware_stream.hpp"

namespace bio = boost::asio;

namespace btr
{

/**
 * The class provides a send/receive interface to a serial port.
 */
class Usart : public HardwareStream
{
public:

// LIFECYCLE

  /**
   * Initialize members to default values, don't open the port.
   */
  Usart();

  /**
   * Close the port.
   */
  ~Usart();

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
  int open(
      const char* port_name,
      uint32_t baud_rate,
      uint8_t data_bits,
      uint8_t parity,
      uint32_t timeout = SERIAL_IO_TIMEOUT);

  /**
   * @see HardwareStream::close
   */
  virtual void close() override;

  /**
   * @see HardwareStream::setTimeout
   */
  virtual int setTimeout(uint32_t timeout) override;

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
  virtual int send(char ch, bool drain = false) override;

  /**
   * @see HardwareStream::send
   */
  virtual int send(const char* buff, bool drain = false) override;

  /**
   * @see HardwareStream::send
   */
  virtual int send(const char* buff, uint32_t bytes, bool drain = false) override;

  /**
   * @see HardwareStream::recv
   */
  virtual int recv() override;

  /**
   * @see HardwareStream::recv
   */
  virtual int recv(char* buff, uint32_t bytes) override;

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
  size_t              timeout_;

}; // class Usart

} // namespace btr

#endif // _btr_Usart_hpp__
