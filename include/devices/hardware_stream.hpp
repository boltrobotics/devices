// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_HardwareStream_hpp_
#define _btr_HardwareStream_hpp_

namespace btr
{

/**
 * The class provides an interface to a hardware input/output device such as USB or USART.
 */
class HardwareStream
{
public:

// LIFECYCLE

  /**
   * Dtor.
   */
  virtual ~HardwareStream() = default;

// OPERATIONS

  /**
   * Close the stream.
   */
  virtual void close() = 0;

  /**
   * @param timeout - timeout in milliseconds
   * @return 0 on success, -1 on failure
   */
  virtual int setTimeout(uint32_t timeout) = 0;

  /**
   * Check if there is data in receive queue.
   *
   * @return bytes available on the serial port or -1 if failed to retrieve the value
   */
  virtual int available() = 0;

  /**
   * Flush pending, not-transmitted and non-read, data on the serial port.
   *
   * @param queue_selector - one of:
   *  IN - flushes data received but not read.
   *  OUT - flushes data written but not transmitted.
   *  INOUT - flushes both data received but not read, and data written but not transmitted.
   */
  virtual int flush(DirectionType queue_selector) = 0;

  /**
   * Send a single character.
   *
   * @param ch - the character to send
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  virtual int send(char ch, bool drain = false) = 0;

  /**
   * Send data from buff up to null character.
   *
   * @param buff - data buffer
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  virtual int send(const char* buff, bool drain = false) = 0;

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  virtual int send(const char* buff, uint32_t bytes, bool drain = false) = 0;

  /**
   * Receive a single character.
   *
   * @return the received character or -1 on error
   */
  virtual int recv() = 0;

  /**
   * Receive a number of bytes and store in the buffer.
   *
   * @param buff - buffer to store received data
   * @param bytes - the number of bytes to receive
   * @return bytes received or -1 on error
   */
  virtual int recv(char* buff, uint32_t bytes) = 0;

protected:

// LIFECYCLE

  /**
   * Ctor.
   */
  HardwareStream() = default;
};

} // namespace btr

#endif // _btr_HardwareStream_hpp_
