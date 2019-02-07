// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_SerialIOTermios_hpp__
#define _btr_SerialIOTermios_hpp__

// SYSTEM INCLUDES
#include <inttypes.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

namespace btr
{

/**
 * The class provides a send/receive interface to a serial port.
 */
class SerialIOTermios
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
  } FlashType;

// LIFECYCLE

  /**
   * Ctor.
   *
   */
  SerialIOTermios();

  /**
   * Dtor.
   */
  ~SerialIOTermios();

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
   * @return -1 on error, 0 otherwise
   */
  int setTimeout(uint32_t timeout_millis);

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
  int flush(FlashType queue_selector);

  /**
   * @return bytes available on the serial port
   */
  uint32_t available();

  /**
   * Set the number of bytes to read.
   *
   * @param bytes - the byte count
   */
  void setReadMinimum(uint32_t bytes);

  /**
   * Read data from serial port.
   *
   * @param buff - the buffer to read into
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

}; // class SerialIOTermios

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

inline SerialIOTermios::SerialIOTermios()
  :
  port_name_(""),
  baud_rate_(),
  data_bits_(),
  parity_(),
  port_(-1)
{
}

inline SerialIOTermios::~SerialIOTermios()
{
  close();
}

//============================================= OPERATIONS =========================================

inline int SerialIOTermios::open(
    const char* port_name,
    uint32_t baud_rate,
    uint8_t data_bits,
    uint8_t parity,
    uint32_t timeout_millis)
{
  port_name_ = port_name;
  data_bits_ = data_bits;
  parity_ = parity;

  baud_rate_ = getNativeBaud(baud_rate);
  port_ = ::open(port_name_, O_RDWR | O_NOCTTY);

  if (port_ < 0) {
    errno = EBADF;
    return -1;
  }

  struct termios options;

  if (tcgetattr(port_, &options) != 0) {
    return -1;
  }

  cfmakeraw(&options);
  //bzero(&options, sizeof(struct termios));
  //options.c_cflag = CS8 | CREAD | CLOCAL | baud_rate_;
  //options.c_iflag = IGNPAR | IGNCR;
  //options.c_lflag &= ~ICANON;

  switch (parity) {
    case PARITY_NONE:
      break;
    case PARITY_EVEN:
      options.c_cflag |= PARENB;
      break;
    case PARITY_ODD:
      options.c_cflag |= PARENB | PARODD;
      break;
    default:
      errno = EINVAL;
      return -1;
  }

  switch (data_bits) {
    case 8:
      options.c_cflag |= CS8;
      break;
    case 7:
      options.c_cflag |= CS7;
      break;
    default:
      errno = EINVAL;
      return -1;
  }

  baud_rate_ = getNativeBaud(baud_rate);

  if (cfsetospeed(&options, baud_rate_) != 0
      || cfsetispeed(&options, baud_rate_) != 0
      || tcsetattr(port_, TCSANOW, &options) != 0
      || flush(FLUSH_INOUT) != 0)
  {
    return -1;
  }

  int rc = setTimeout(timeout_millis);
  return rc;
}

inline void SerialIOTermios::close()
{
  if (port_ != -1) {
    ::close(port_);
    port_ = -1;
  }
}

inline int SerialIOTermios::setTimeout(uint32_t timeout_millis)
{
  struct termios options;
  int rc = 0;

  if ((rc = tcgetattr(port_, &options)) == 0) {
    // 1. VMIN = 0 and VTIME > 0
    //  read returns if one or more bytes are available or VTIME expires
    // 2. VMIN > 0 and VTIME > 0
    //  read returns when either VMIN characters are received or VTIME BETWEEN characters expires
    //
    // See http://unixwiz.net/techtips/termios-vmin-vtime.html
    //
    // WARNING: VTIME is in tenths of a second, so the minimum can be set is 100 milliseconds
    //
    options.c_cc[VTIME] = timeout_millis / 100;
    options.c_cc[VMIN] = 0;
    rc = tcsetattr(port_, TCSANOW, &options);
  }
  return rc;
}

inline int SerialIOTermios::flush(FlashType queue_selector)
{
  int rc = 0;

  switch (queue_selector) {
    case FLUSH_IN:
      rc = tcflush(port_, TCIFLUSH);
      break;
    case FLUSH_OUT:
      rc = tcflush(port_, TCOFLUSH);
      break;
    case FLUSH_INOUT:
      rc = tcflush(port_, TCIOFLUSH);
      break;
    default:
      errno = EINVAL;
      rc = -1;
  };
  return rc;
}

inline uint32_t SerialIOTermios::available()
{
  uint32_t bytes_available;
  ioctl(port_, FIONREAD, &bytes_available);
  return bytes_available;
}

inline void SerialIOTermios::setReadMinimum(uint32_t bytes)
{
  struct termios options;
  tcgetattr(port_, &options);
  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = bytes;
  tcsetattr(port_, TCSANOW, &options);
}

inline ssize_t SerialIOTermios::recv(char* buff, uint32_t bytes)
{
  ssize_t rc = read(port_, buff, bytes);
  return rc;
}

inline ssize_t SerialIOTermios::send(const char* buff, uint32_t bytes, bool drain)
{
  ssize_t rc = write(port_, buff, bytes);

  if (drain) {
    tcdrain(port_);
  }
  return rc;
}

inline int SerialIOTermios::sendBreak(uint32_t duration)
{
  return tcsendbreak(port_, duration);
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

inline int SerialIOTermios::getNativeBaud(int num)
{
  int baud = B57600;

  switch (num) {
    case 9600:
      baud = B9600;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      break;
  };

  return baud;
}

} // namespace btr

#endif // _btr_SerialIOTermios_hpp__
