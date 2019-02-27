// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <avr/io.h>
#include <util/atomic.h>

// PROJECT INCLUDES
#include "devices/avr/time.hpp"  // class implemented

#if BTR_TIME_ENABLED > 0

////////////////////////////////////////////////////////////////////////////////////////////////////
// Local defines {

#define TCCRB   TCCR0B
#define TCNT    TCNT0
#define TIMSK   TIMSK0
#define TOIE    TOIE0
#define TIFR    TIFR0
#define TOV     TOV0
#define CS0     CS00
#define CS1     CS01
#define CS2     CS02

#define BTR_TIME_TICK_FREQ            64
#define BTR_TIME_TICKS_PER_USEC       (F_CPU / 1000000UL)
#define BTR_TIME_TICKS_TO_USEC(ticks) ((ticks) * BTR_TIME_TICK_FREQ / (BTR_TIME_TICKS_PER_USEC))
#define BTR_TIME_USEC_TO_TICKS(usec)  ((usec) * (BTR_TIME_TICKS_PER_USEC))

// } Local defines

////////////////////////////////////////////////////////////////////////////////////////////////////
// Static members {

static volatile uint32_t ovf_count_ = 0;

// } Static members

////////////////////////////////////////////////////////////////////////////////////////////////////
// ISRs {
// See: http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html

ISR(TIMER0_OVF_vect)
{
  ++ovf_count_;
}

// } ISRs

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

// static
void Time::init()
{
  set_bit(TCCRB, CS1);
  set_bit(TCCRB, CS0);
  set_bit(TIMSK, TOIE);
}

// static
void Time::shutdown()
{
  clear_bit(TIMSK, TOIE);
  clear_bit(TCCRB, CS0);
  clear_bit(TCCRB, CS1);
}

// static
uint32_t Time::micros()
{
  uint32_t usec;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    uint32_t ovf_count_tot = ovf_count_; 

    if (bit_is_set(TIFR, TOV)) {
      ++ovf_count_tot;
    }

    // Left shift multiplies by 256, which is a maximum value of overflow counter.
    uint32_t ticks = ((ovf_count_tot << 8) + TCNT);
    usec = BTR_TIME_TICKS_TO_USEC(ticks);
  }
  return usec;
}

// static
uint32_t Time::millis()
{
  return (micros() / 1000);
}

} // namespace btr

#endif // BTR_TIME_ENABLED > 0
