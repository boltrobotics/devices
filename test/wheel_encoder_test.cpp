/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

// SYSTEM INCLUDES
#include <gtest/gtest.h>

// PROJECT INCLUDES
#include "devices/wheel_encoder.hpp"

namespace btr
{

//================================ TEST FIXTURES ===============================

class WheelEncoderTest : public testing::Test
{
public:

  // LIFECYCLE

  WheelEncoderTest()
  : enc_(0, 0, 1)
  {
  }

protected:

  // ATTRIBUTES

  WheelEncoder enc_;

}; // WheelEncoderTest

//=================================== TESTS ====================================

TEST_F(WheelEncoderTest, testClicks)
{
    // Test forward motion. B output follows A.
    enc_.update(1, 0);
    enc_.update(1, 1);
    enc_.update(0, 1);
    enc_.update(0, 0);
    EXPECT_EQ(4, enc_.clicks());

    // Test reverse motion. A output follows B.
    enc_.update(0, 1);
    enc_.update(1, 1);
    enc_.update(1, 0);
    enc_.update(0, 0);
    EXPECT_EQ(0, enc_.clicks());
}

TEST_F(WheelEncoderTest, testReset)
{
}

} // namespace btr
