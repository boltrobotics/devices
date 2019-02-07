// Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_TCRT5000_hpp_
#define _btr_TCRT5000_hpp_

// SYSTEM INCLUDES
#include <Arduino.h>

namespace btr
{

/**
 *
 */
class TCRT5000
{
public:

// LIFECYCLE

  TCRT5000();

// OPERATIONS

  void update();

private:

// OPERATIONS

// ATTRIBUTES


}; // class TCRT5000

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

inline TCRT5000::TCRT5000()
{
}

//============================================= OPERATIONS =========================================

inline void TCRT5000::update()
{
#if 0;
  pinMode(6, OUTPUT);

  // Turn on TCRT emitting led
  digitalWrite(6, HIGH);
  delayMicroseconds(500);

  int noisy_signal = analogRead(A3);

  // Turn off LED
  digitalWrite(6, LOW);
  delayMicroseconds(500);

  // Read from photodiode (noise only)
  int noise = analogRead(A3);
  // Output filtered value
  Serial.println(noisy_signal - noise);
#endif
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // _btr_TCRT5000_hpp_
