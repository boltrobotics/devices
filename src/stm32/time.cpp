// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include "FreeRTOS.h"
#include "task.h"

// PROJECT INCLUDES
#include "devices/time.hpp"  // class implemented

#if BTR_TIME_ENABLED > 0

////////////////////////////////////////////////////////////////////////////////////////////////////
// Local defines {

// } Local defines

////////////////////////////////////////////////////////////////////////////////////////////////////
// Static members {

// } Static members

////////////////////////////////////////////////////////////////////////////////////////////////////
// ISRs {

extern "C" {
} // extern "C"

// } ISRs

namespace btr
{

static constexpr uint16_t RATE_SCALER_MS = 1000 / configTICK_RATE_HZ;

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

// static
void Time::init()
{
  // Noop
}

// static
void Time::shutdown()
{
  // Noop
}

// static
uint32_t Time::sec()
{
  return (xTaskGetTickCount() / configTICK_RATE_HZ);
}

// static
uint32_t Time::millis()
{
  return (xTaskGetTickCount() * RATE_SCALER_MS);
}

// static
uint32_t Time::diff(uint32_t head_time, uint32_t tail_time)
{
  return ((UINT32_MAX + head_time - tail_time) % UINT32_MAX);
}

} // namespace btr

#endif // BTR_TIME_ENABLED > 0
