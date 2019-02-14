// Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_UsartTermios_hpp__
#define _btr_UsartTermios_hpp__

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "devices/defines.hpp"
#include "devices/hardware_stream.hpp"

namespace btr
{

/**
 * The class provides a send/receive interface to a serial port.
 */
class UsartTermios : public HardwareStream
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

}; // class UsartTermios

} // namespace btr

#endif // _btr_UsartTermios_hpp__
