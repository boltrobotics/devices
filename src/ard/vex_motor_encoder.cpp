// SYSTEM INCLUDES
#include "Arduino.h"

// PROJECT INCLUDES
#include "devices/ard/vex_motor_encoder.hpp"
#include "devices/ard/i2c.hpp"

// INITIALIZE
unsigned char VexMotorEncoder::nextAddress = I2CENCODER_STARTING_ADDRESS;
VexMotorEncoder* VexMotorEncoder::lastEncoder = NULL;

// CONSTRUCTOR
/**
 * Create the encoder with it's automatically assigned address.
 */
VexMotorEncoder::VexMotorEncoder()
{
  address = nextAddress;
  nextAddress++;
  is_reversed = false;
  rotation_factor = 0;
  time_delta = 0;
  ticks = 0;
}

// PUBLIC METHODS

/*
 * Initialize the encoder to it's address.
 */
void VexMotorEncoder::init(double rotation_factor, double time_delta, int ticks)
{
  // Unterminates previous encoder so that messages flow to this one.
  if (lastEncoder) {
    lastEncoder->unTerminate();
  }
  lastEncoder = this;
  
  this->rotation_factor = rotation_factor;
  this->time_delta = time_delta;
  this->ticks = ticks;

#if 0
  // Assign it's address
  Wire.beginTransmission(I2CENCODER_DEFAULT_ADDRESS);
  Wire.write(I2CENCODER_ADDRESS_REGISTER);
  Wire.write(address << 1); // Shift to an 8-bit address for the encoder.
  Wire.endTransmission();
#else
  I2c.write(
      uint8_t(I2CENCODER_DEFAULT_ADDRESS),
      uint8_t(I2CENCODER_ADDRESS_REGISTER),
      uint8_t(address << 1));
#endif
  // Zero it on initialization
  zero();
}

void VexMotorEncoder::init(double rotation_factor, double time_delta)
{
  init(rotation_factor, time_delta, TICKS);
}

/**
 * Sets whether or not the encoder is setup "backwards" or flipped.
 */
void VexMotorEncoder::setReversed(bool is_reversed)
{
  this->is_reversed = is_reversed;
}

/**
 * Returns the speed of the encoder rotation per minute for the output
 * shaft of the motor. (Assumes 269)
 */
double VexMotorEncoder::getSpeed()
{
  // TODO: Check sanity of the values
  unsigned int vb = getVelocityBits();
  if (vb == 0xFFFF) return 0;
  return rotation_factor / (double(vb) * time_delta);
}

/**
 * Returns the unsigned velocity bits. This is the time-delta between
 * ticks in multiples of 64 microseconds/tick. Stopped is 0xFFFF or 4
 * seconds. (Assumes 269)
 */
unsigned int VexMotorEncoder::getVelocityBits()
{
#if 0
  accessRegister(I2CENCODER_VELOCITY_REGISTER);
  Wire.requestFrom(address, 2);
  unsigned int speed = 0;
  speed |= Wire.read() << 8;
  speed |= Wire.read();
  return speed;
#else
  uint16_t speed = 0;
  I2c.read(address, I2CENCODER_VELOCITY_REGISTER, sizeof(speed));
  speed = uint16_t(I2c.receive()) << 8;
  speed |= I2c.receive();
  return speed;
#endif
}

/**
 * Returns the position in rotations since power on or last reset.
 */
double VexMotorEncoder::getPosition()
{
  return rotation_factor / ((double) ticks) * ((double) getRawPosition());
}

/**
 * Returns the position in encoder ticks since power on or last reset.
 */
long VexMotorEncoder::getRawPosition()
{
  long position = 0;
#if 0
  // TODO: Deal with the two extra bytes
  accessRegister(I2CENCODER_POSITION_REGISTER);
  Wire.requestFrom(address, 4);

  position |= Wire.read() << 8;
  position |= Wire.read();
  position |= Wire.read() << 24;
  position |= Wire.read() << 16;

#else
  uint32_t pos = 0;
  I2c.read(address, I2CENCODER_POSITION_REGISTER, sizeof(position));

  pos |= uint32_t(I2c.receive()) << 8;
  pos |= I2c.receive();
  pos |= uint32_t(I2c.receive()) << 24;
  pos |= uint32_t(I2c.receive()) << 16;
  position = pos;
#endif
  return is_reversed ? -position : position;
}

/**
 * Zero the position of this encoder.
 */
void VexMotorEncoder::zero()
{
  accessRegister(I2CENCODER_ZERO_REGISTER);
}

/**
 * UnTerminate this encoder. This allows access to all I2C devices
 * after this encoder.
 */
void VexMotorEncoder::unTerminate()
{
  accessRegister(I2CENCODER_UNTERMINATE_REGISTER);
}

/**
 * Terminate this encoder. This prevents access to all I2C devices
 * after this encoder.
 */
void VexMotorEncoder::terminate()
{
  accessRegister(I2CENCODER_TERMINATE_REGISTER);
}

/**
 * Gets the I2C Address of this encoder for manual access.
 */
int VexMotorEncoder::getAddress()
{
  return address;
}

// Private Functions
void VexMotorEncoder::accessRegister(unsigned char reg)
{
#if 0
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
#else
  I2c.write(uint8_t(address), uint8_t(reg));
#endif
}
