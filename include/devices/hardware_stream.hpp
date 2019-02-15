// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_HardwareStream_hpp_
#define _btr_HardwareStream_hpp_

#include "devices/defines.hpp"

namespace btr
{

/**
 * The class provides an interface to a hardware input/output device such as USB or USART.
 */
template<typename... Mixins>
class HardwareStream : public Mixins...
{
public:

// LIFECYCLE

  /**
   * Ctor.
   */
  HardwareStream(const Mixins&... mixins);

// OPERATIONS

private:

// ATTRIBUTES

  DevType dev_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

template<typename... Mixins>
HardwareStream<Mixins...>::HardwareStream(const Mixins&... mixins)
  :
  Mixins(mixins)...
{
}

//============================================= OPERATIONS =========================================

template<typename... Mixins>
inline HardwareStream* HardwareStream<Mixins...>::makeInstance(const Mixins&... mixins)
{

}

} // namespace btr

#endif // _btr_HardwareStream_hpp_
