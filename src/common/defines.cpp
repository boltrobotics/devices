// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// PROJECT INCLUDES
#include "devices/defines.hpp"  // class implemented

namespace btr
{
namespace dev
{

#if BTR_STATUS_ENABLED > 0
static uint32_t status_;

uint32_t* status()
{
  return &status_;
}
#else
uint32_t* status()
{
  return nullptr;
}
#endif // BTR_STATUS_ENABLED > 0

} // namespace dev
} // namespace btr
