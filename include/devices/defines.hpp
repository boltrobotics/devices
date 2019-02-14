// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Defines_hpp_
#define _btr_Defines_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

namespace btr
{

#ifndef SERIAL_IO_TIMEOUT
#define SERIAL_IO_TIMEOUT 100
#endif

typedef enum
{
  NONE,
  ODD,
  EVEN
} ParityType;

typedef enum
{
  IN,
  OUT,
  INOUT
} DirectionType;

} // namespace btr

#endif // _btr_Defines_hpp_
