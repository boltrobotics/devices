// Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <boost/bind.hpp>
#include <sys/ioctl.h>

// PROJECT INCLUDES
#include "devices/x86/usart.hpp"

#define BOOST_SYSTEM_NO_DEPRECATED

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

Usart::Usart()
  :
  io_service_(),
  serial_port_(io_service_),
  timer_(io_service_),
  bytes_transferred_(0),
  port_name_(nullptr),
  baud_rate_(BTR_USART1_BAUD),
  data_bits_(BTR_USART1_DATA_BITS),
  parity_(ParityType::NONE),
  timeout_(BTR_USART_IO_TIMEOUT_MS)
{
}

Usart::~Usart()
{
  close();
}

//============================================= OPERATIONS =========================================

void Usart::configure(
    const char* port_name,
    uint32_t baud_rate,
    uint8_t data_bits,
    uint8_t parity,
    uint32_t timeout)
{
  port_name_ = port_name;
  baud_rate_ = baud_rate;
  data_bits_ = data_bits;
  parity_ = parity;
  timeout_ = timeout;
}

bool Usart::isOpen()
{
  return serial_port_.is_open();
}

int Usart::open()
{
  errno = 0;
  boost::system::error_code ec;
  serial_port_.open(port_name_, ec);

  if (0 == ec.value()) {
    serial_port_.set_option(bio::serial_port::baud_rate(baud_rate_));
    serial_port_.set_option(bio::serial_port::character_size(data_bits_));

    switch (parity_) {
      case EVEN:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::even));
        break;
      case ODD:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::odd));
        break;
      case NONE:
      default:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::none));
    }
  } else {
    errno = ec.value();
    return -1;
  }
  return 0;
}

void Usart::close()
{
  if (serial_port_.is_open()) {
    timer_.cancel();
    serial_port_.cancel();
    serial_port_.close();
  }
}

int Usart::setTimeout(uint32_t timeout)
{
  timeout_ = timeout;
  return 0;
}

int Usart::available()
{
  int bytes_available;
  ioctl(serial_port_.lowest_layer().native_handle(), FIONREAD, &bytes_available);
  return bytes_available;
}

int Usart::flush(DirectionType queue_selector)
{
  errno = 0;
  int rc = 0;

  switch (queue_selector) {
    case IN:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCIFLUSH);
      break;
    case OUT:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCOFLUSH);
      break;
    case INOUT:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCIOFLUSH);
      break;
    default:
      errno = EINVAL;
      rc = -1;
  }
  return rc;
}

int Usart::send(char ch, bool drain)
{
  return send(&ch, 1, drain);
}

int Usart::send(const char* buff, bool drain)
{
  uint32_t bytes = strlen(buff);
  return send(buff, bytes, drain);
}

int Usart::send(const char* buff, uint32_t bytes, bool drain)
{
  io_service_.reset();
  errno = 0;
  bytes_transferred_ = 0;

  bio::async_write(
      serial_port_,
      bio::buffer(buff, bytes),
      boost::bind(&Usart::onOprComplete,
        this,
        bio::placeholders::error,
        bio::placeholders::bytes_transferred));

  timeAsyncOpr();

  if (drain) {
    tcdrain(serial_port_.lowest_layer().native_handle());
  }

  return bytes_transferred_;
}

int Usart::recv()
{
  char buff[1];
  return recv(buff, 1);
}

int Usart::recv(char* buff, uint32_t bytes)
{
  io_service_.reset();
  errno = 0;
  bytes_transferred_ = 0;

  bio::async_read(
      serial_port_,
      bio::buffer(buff, bytes),
      boost::bind(
        &Usart::onOprComplete,
        this,
        bio::placeholders::error,
        bio::placeholders::bytes_transferred));

  timeAsyncOpr();
  return bytes_transferred_;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

void Usart::timeAsyncOpr()
{
  timer_.expires_from_now(boost::posix_time::milliseconds(timeout_));
  timer_.async_wait(boost::bind(&Usart::onTimeout, this, bio::placeholders::error));
  io_service_.run();
}

void Usart::onOprComplete(
    const boost::system::error_code& err, size_t bytes_transferred)
{
  bytes_transferred_ = bytes_transferred;

  if (err) {
    // When timer cancels operation, the error is 89, Operation canceled.
    errno = err.value();
  }
  timer_.cancel();
}

void Usart::onTimeout(const boost::system::error_code& error)
{
  // When the timer is cancelled, the error generated is bio::operation_aborted.
  //
  if (!error) {
    // When the timer fires, there is no error, therefore just cancel pending operation.
    serial_port_.cancel();
  }
}

} // namespace btr
