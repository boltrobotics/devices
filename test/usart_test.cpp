// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <chrono>

// PROJECT INCLUDES
#include "devices/x86/usart.hpp"
#include "devices/x86/pseudo_tty.hpp"
#include "devices/defines.hpp"
#include "utility/buff.hpp"
#include "utility/test_helpers.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

namespace btr
{

#define BAUD 115200
#define DATA_BITS 8

//------------------------------------------------------------------------------

class UsartTest : public testing::Test
{
public:

  // LIFECYCLE

  UsartTest()
    :
      tty_(),
      reader_(),
      sender_(),
      wbuff_(),
      rbuff_()
  {
    reader_.open(TTY_SIM_0, BAUD, DATA_BITS, ParityType::NONE, SERIAL_IO_TIMEOUT);
    sender_.open(TTY_SIM_1, BAUD, DATA_BITS, ParityType::NONE, SERIAL_IO_TIMEOUT);
    resetBuffers();
  }

  void resetBuffers()
  {
    uint8_t h[] =  { 'h','e','l','l','o' };

    wbuff_.reset();
    wbuff_.resize(sizeof(h));
    wbuff_.write(h, sizeof(h) / sizeof(uint8_t));

    // Don't expect to receive endline character(s)
    rbuff_.reset();
    rbuff_.resize(wbuff_.size());
  }

protected:

  // ATTRIBUTES

  PseudoTTY tty_;
  Usart reader_;
  Usart sender_;
  Buff wbuff_;
  Buff rbuff_;
};

//------------------------------------------------------------------------------

// Tests {

TEST_F(UsartTest, readWriteOK)
{
  ssize_t rc = sender_.send((char*)wbuff_.read_ptr(), wbuff_.available());
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);

  rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());

  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size()));
  TEST_MSG << TestHelpers::toHex(rbuff_);
}

TEST_F(UsartTest, flush)
{
  ssize_t rc = sender_.send((char*)wbuff_.read_ptr(), wbuff_.available(), true);
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);

  std::this_thread::sleep_for(20ms);

  rc = reader_.available();
  ASSERT_EQ(5, rc);
  rc = reader_.flush(DirectionType::IN);
  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
  rc = reader_.available();
  ASSERT_EQ(0, rc);

  rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());
  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);

  resetBuffers();

  rc = sender_.send((char*)wbuff_.read_ptr(), wbuff_.available());
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);

  rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size())) << TestHelpers::toHex(rbuff_);
}

TEST_F(UsartTest, readTimeout)
{
  high_resolution_clock::time_point start = high_resolution_clock::now();

  ssize_t rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());

  high_resolution_clock::time_point now = high_resolution_clock::now();
  auto elapsed = duration_cast<milliseconds>(now - start).count();

  ASSERT_LE(SERIAL_IO_TIMEOUT, elapsed);
  ASSERT_GT(SERIAL_IO_TIMEOUT + 20, elapsed);

  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
}

TEST_F(UsartTest, setTimeout)
{
  uint32_t timeout = 200;
  reader_.setTimeout(timeout);
  high_resolution_clock::time_point start = high_resolution_clock::now();

  ssize_t rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());

  high_resolution_clock::time_point now = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(now - start).count();

  ASSERT_LE((timeout - 10), duration);
  ASSERT_GT((timeout + 10), duration);

  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
}

TEST_F(UsartTest, DISABLED_writeTimeout)
{
#if 0
  // FIXME: Write time-out simulation doesn't work.
  Buff large_buff;
  large_buff.resize(65536);
  int rc = sender_.send(&large_buff);
  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
#endif
}

// } Tests

} // namespace btr
