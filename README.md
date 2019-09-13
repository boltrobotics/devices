# Overview

The project contains device drivers for use on AVR / STM32 / x86 platforms.

# Table of Contents

* [Build](#Build)
* [x86](#x86)
  * [PseudoTTY](#PseudoTTY)
  * [Usart](#Usart)
  * [UsartTermios](#UsartTermios)
* [STM32](#stm32)
  * [Usb](#Usb)
  * [Usart](#stm32_Usart)
  * [PwmMotor2Wire](#PwmMotor2Wire)
  * [PwmMotor3Wire](#PwmMotor3Wire)
* [AVR](#avr)
  * [PwmMotor3Wire](#PwmMotor3Wire)
* [Common Code](#common_code)
  * [MaxSonarLvEz](#MaxSonarLvEx)
  * [WheelEncoder](#WheelEncoder)

## <a name="Build">Build</a>

Set up <a href="https://github.com/boltrobotics/cmake-helpers" target="_blank">cmake build</a> by
following an <a href="https://github.com/boltrobotics/cmake-helpers#Example">example</a>.

## <a name="x86">x86</a>

### <a name="PseudoTTY" href="https://github.com/boltrobotics/devices/tree/master/include/devices/x86/pseudo_tty.hpp" target="_blank">PseudoTTY</a>

The class creates two serial devices using <a href="https://linux.die.net/man/1/socat" target="_blank">socat</a> utility. PseudoTTY is useful for testing serial client or server. The unit tests for Usart class use it for similating serial ports.

### <a name="Usart" href="https://github.com/boltrobotics/devices/tree/master/include/devices/x86/usart.hpp" target="_blank">Usart</a>

The class provides an interface for communication over serial connection. Each read/write
is configured to time out if the operation doesn't complete within a specified window. The code uses
Boost ASIO library.

<a name="usart_test" href="https://github.com/boltrobotics/devices/tree/master/test/usart_test.cpp" target="_blank">usart_test.cpp</a> contains unit tests for Usart class.

### <a name="UsartTermios" href="https://github.com/boltrobotics/devices/tree/master/include/devices/x86/usart_termios.hpp" target="_blank">UsartTermios</a>

Class functionality is similar to [Usart](#Usart) but using termios library.

<a name="usart_termios_test" href="https://github.com/boltrobotics/devices/tree/master/test/usart_termios_test.cpp" target="_blank">usart_termios_test.cpp</a> contains unit tests for Usart class.

## <a name="stm32">STM32</a>

### <a name="Usb" href="https://github.com/boltrobotics/devices/tree/master/include/devices/stm32/usb.hpp" target="_blank">Usb</a>

The class provides an interface to transfer data over USB connection.

### <a name="stm32_Usart" href="https://github.com/boltrobotics/devices/tree/master/include/devices/stm32/usart.hpp" target="_blank">Usart</a>

The class provides an interface for communication over serial connection.

### <a name="PwmMotor3Wire" href="https://github.com/boltrobotics/devices/tree/master/include/devices/stm32/pwm_motor_3wire.hpp" target="_blank">PwmMotor3Wire</a>

The class can drive a motor that uses three wires for direction and speed control.

### <a name="PwmMotor2Wire" href="https://github.com/boltrobotics/devices/tree/master/include/devices/stm32/pwm_motor_2wire.hpp" target="_blank">PwmMotor2Wire</a>

The class can drive a motor that uses two wires for direction and speed control.

## <a name="avr">AVR</a>

### <a name="PwmMotor3Wire" href="https://github.com/boltrobotics/devices/tree/master/include/devices/avr/pwm_motor_3wire.hpp" target="_blank">PwmMotor3Wire</a>

The class can drive a motor that uses three wires for direction and speed control.

## <a name="common_code">Common Code</a>

### <a name="MaxSonarLvEx" href="https://github.com/boltrobotics/devices/tree/master/include/devices/maxsonar_lvez.hpp" target="_blank">MaxSonarLvEx</a>

The class calculates range in millimeters from an ADC sample of MaxSonar ultrasonic range finder.

### <a name="WheelEncoder" href="https://github.com/boltrobotics/devices/tree/master/include/devices/wheel_encoder.hpp" target="_blank">WheelEncoder</a>

The class counts the number of clicks that a virtual wheel moved forward and backward.
