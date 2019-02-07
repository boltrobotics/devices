// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_SerialIOBoost_hpp__
#define _btr_SerialIOBoost_hpp__

// SYSTEM INCLUDES
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <sys/ioctl.h>

// PROJECT INCLUDES

#define BOOST_SYSTEM_NO_DEPRECATED

#ifndef SERIAL_IO_TIMEOUT
#define SERIAL_IO_TIMEOUT 100
#endif

namespace bio = boost::asio;

namespace btr
{

/**
 * The class provides a send/receive interface to a serial port.
 */
class SerialIOBoost
{
public:

  typedef enum
  {
    PARITY_NONE,
    PARITY_ODD,
    PARITY_EVEN
  } ParityType;

  typedef enum
  {
    FLUSH_IN,
    FLUSH_OUT,
    FLUSH_INOUT
  } FlushType;

// LIFECYCLE

  /**
   * Initialize members to default values, don't open the port.
   */
  SerialIOBoost();

  /**
   * Close the port.
   */
  ~SerialIOBoost();

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
      uint32_t timeout_millis);

  /**
   * Close serial port.
   */
  void close();

  /**
   * @param timeout_millis 
   */
  void setTimeout(uint32_t timeout_millis);

  /**
   * Flush not-transmitted and non-read data on the serial port.
   *
   * @param queue_selector - one of:
   *  FLUSH_IN - flushes data received but not read.
   *  FLUSH_OUT - flushes data written but not transmitted.
   *  FLUSH_INOUT - flushes both data received but not read, and data written but not transmitted.
   *
   *  @see termios(3)
   */
  int flush(FlushType queue_selector);

  /**
   * @return bytes available on the serial port
   */
  uint32_t available();

  /**
   * Read data from serial port.
   *
   * @param buff - container for received bytes
   * @param bytes - the number of bytes to read
   * @return the number of bytes transferred
   */
  ssize_t recv(char* buff, uint32_t bytes);

  /**
   * Write data to serial port.
   *
   * @param data - the data to send
   * @param bytes - the number of bytes to send
   * @param drain - block until all output has been transmitted
   * @return the number of bytes transferred
   */
  ssize_t send(const char* buff, uint32_t bytes, bool drain = false);

  /**
   * Transmit a continuous stream of 0 bits.
   *
   * @param duration - the length of the transmission. If duration is greater than 0, 0 bits are
   *  transmitted for duration milliseconds. If duration is 0, 0 bits are transmitted for 0.25
   *  seconds.
   */
  int sendBreak(uint32_t duration);

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

}; // class SerialIOBoost

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

inline SerialIOBoost::SerialIOBoost()
  :
  io_service_(),
  serial_port_(io_service_),
  timer_(io_service_),
  bytes_transferred_(0),
  timeout_(SERIAL_IO_TIMEOUT)
{
}

inline SerialIOBoost::~SerialIOBoost()
{
  close();
}

//============================================= OPERATIONS =========================================

inline int SerialIOBoost::open(
    const char* port_name,
    uint32_t baud_rate,
    uint8_t data_bits,
    uint8_t parity,
    uint32_t timeout_millis)
{
  (void) timeout_millis;

  close();

  errno = 0;
  boost::system::error_code ec;
  serial_port_.open(port_name, ec);

  if (0 == ec.value()) {
    serial_port_.set_option(bio::serial_port::baud_rate(baud_rate));
    serial_port_.set_option(bio::serial_port::character_size(data_bits));

    switch (parity) {
      case PARITY_EVEN:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::even));
        break;
      case PARITY_ODD:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::odd));
        break;
      case PARITY_NONE:
      default:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::none));
    }
  } else {
    errno = ec.value();
    return -1;
  }
  return 0;
}

inline void SerialIOBoost::close()
{
  if (serial_port_.is_open()) {
    timer_.cancel();
    serial_port_.cancel();
    serial_port_.close();
  }
}

inline void SerialIOBoost::setTimeout(uint32_t timeout_millis)
{
  timeout_ = timeout_millis;
}

inline int SerialIOBoost::flush(FlushType queue_selector)
{
  errno = 0;
  int rc = 0;

  switch (queue_selector) {
    case FLUSH_IN:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCIFLUSH);
      break;
    case FLUSH_OUT:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCOFLUSH);
      break;
    case FLUSH_INOUT:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCIOFLUSH);
      break;
    default:
      errno = EINVAL;
      rc = -1;
  }
  return rc;
}

inline uint32_t SerialIOBoost::available()
{
  uint32_t bytes_available;
  ioctl(serial_port_.lowest_layer().native_handle(), FIONREAD, &bytes_available);
  return bytes_available;
}

inline ssize_t SerialIOBoost::recv(char* buff, uint32_t bytes)
{
  io_service_.reset();
  errno = 0;
  bytes_transferred_ = 0;

  bio::async_read(
      serial_port_,
      bio::buffer(buff, bytes),
      boost::bind(
        &SerialIOBoost::onOprComplete,
        this,
        bio::placeholders::error,
        bio::placeholders::bytes_transferred));

  timeAsyncOpr();
  return bytes_transferred_;
}

inline ssize_t SerialIOBoost::send(const char* buff, uint32_t bytes, bool drain)
{
  io_service_.reset();
  errno = 0;
  bytes_transferred_ = 0;

  bio::async_write(
      serial_port_,
      bio::buffer(buff, bytes),
      boost::bind(&SerialIOBoost::onOprComplete,
        this,
        bio::placeholders::error,
        bio::placeholders::bytes_transferred));

  timeAsyncOpr();

  if (drain) {
    tcdrain(serial_port_.lowest_layer().native_handle());
  }

  return bytes_transferred_;
}

inline int SerialIOBoost::sendBreak(uint32_t duration)
{
  return tcsendbreak(serial_port_.lowest_layer().native_handle(), duration);
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

inline void SerialIOBoost::timeAsyncOpr()
{
  timer_.expires_from_now(boost::posix_time::milliseconds(timeout_));
  timer_.async_wait(boost::bind(&SerialIOBoost::onTimeout, this, bio::placeholders::error));
  io_service_.run();
}

inline void SerialIOBoost::onOprComplete(
    const boost::system::error_code& err, size_t bytes_transferred)
{
  bytes_transferred_ = bytes_transferred;

  if (err) {
    // When timer cancels operation, the error is 89, Operation canceled.
    errno = err.value();
  }
  timer_.cancel();
}

inline void SerialIOBoost::onTimeout(const boost::system::error_code& error)
{
  // When the timer is cancelled, the error generated is bio::operation_aborted.
  //
  if (!error) {
    // When the timer fires, there is no error, therefore just cancel pending operation.
    serial_port_.cancel();
  }
}

} // namespace btr

#endif // _btr_SerialIOBoost_hpp__
