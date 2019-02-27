// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Time_hpp_
#define _btr_Time_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "devices/defines.hpp"

namespace btr
{

/**
 * The class provides an interface to time-keeping functions.
 */
class Time
{
public:

// OPERATIONS

  /**
   * Initialize.
   */
  static void init();

  /**
   * Shut down the time-keeping functions.
   */
  static void shutdown();

  /**
   * @return microseconds counter value
   */
  static uint32_t micros();

  /**
   * @return micros() divided by 1000
   */
  static uint32_t millis();

  /**
   * @param head_time - the time that comes after or equal to tail_time
   * @param tail_time - the time that comes before or equal to head_time
   * @return time difference between two values
   */
  static uint32_t difftime(uint32_t head_time, uint32_t tail_time);
};

} // namespace btr

#endif // _btr_Time_hpp_
