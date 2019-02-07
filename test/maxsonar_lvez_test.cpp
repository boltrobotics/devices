// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <gtest/gtest.h>

// PROJECT INCLUDES
#include "devices/maxsonar_lvez.hpp"

namespace btr
{

//========================================== TEST FIXTURES =========================================

class MaxSonarLvEzTest : public testing::Test
{
public:

  // LIFECYCLE

  MaxSonarLvEzTest()
  {
  }

protected:

  // ATTRIBUTES

  //MaxSonarLvEz sonar_;

}; // MaxSonarLvEzTest

//============================================= TESTS ==============================================

TEST_F(MaxSonarLvEzTest, testRange)
{
  uint16_t adc_sample = 61;
  uint16_t range = MaxSonarLvEz::range(adc_sample);
  ASSERT_EQ(774, range);
}

} // namespace btr
