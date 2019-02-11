# Introduction

The project contains device drivers for use on AVR / STM32 / x86 platforms.

Software build uses <a href="https://github.com/boltrobotics/cmake-helpers" target="_blank">
cmake-helpers</a>.

# Table of Contents

* [x86](#x86)
  * [PseudoTTY](#PseudoTTY)
  * [Uart](#Uart)
  * [UartTermios](#UartTermios)
* [STM32](#stm32)
  * [Usb](#Usb)
  * [PwmMotor2Wire](#PwmMotor2Wire)
  * [PwmMotor3Wire](#PwmMotor3Wire)
* [AVR](#avr)
  * [PwmMotor3Wire](#PwmMotor3Wire)
* [Contribute](#Contribute)

## x86

### <a name="PseudoTTY" href="https://github.com/boltrobotics/devices/tree/master/include/devices/x86/pseudo_tty.hpp" target="_blank">PseudoTTY</a>

The class creates two serial devices using <a href="https://linux.die.net/man/1/socat" target="_blank">socat</a> utility. PseudoTTY is useful for testing serial client or server. The unit tests for Uart class use it for similating serial ports.

### <a name="Uart" href="https://github.com/boltrobotics/devices/tree/master/include/devices/x86/uart.hpp" target="_blank">Uart</a>

The class provides an interface for communication over serial connection. Each read/write
is configured to time out if the operation doesn't complete within a specified window. The code uses
Boost ASIO library.

<a name="uart_test" href="https://github.com/boltrobotics/devices/tree/master/test/uart_test.cpp" target="_blank">uart_test.cpp</a> contains unit tests for Uart class.

### <a name="UartTermios" href="https://github.com/boltrobotics/devices/tree/master/include/devices/x86/uart_termios.hpp" target="_blank">UartTermios</a>

Class functionality is similar to [Uart](#Uart) but using termios library.

<a name="uart_termios_test" href="https://github.com/boltrobotics/devices/tree/master/test/uart_termios_test.cpp" target="_blank">uart_termios_test.cpp</a> contains unit tests for Uart class.

## STM32

### <a name="Usb" href="https://github.com/boltrobotics/devices/tree/master/include/devices/stm32/usb.hpp" target="_blank">Usb</a>

The class provides an interface to transfer data over USB connection.

### <a name="PwmMotor3Wire" href="https://github.com/boltrobotics/devices/tree/master/include/devices/stm32/pwm_motor_3wire.hpp" target="_blank">PwmMotor3Wire</a>

The class can drive a motor that uses three wires for direction and speed control.

### <a name="PwmMotor2Wire" href="https://github.com/boltrobotics/devices/tree/master/include/devices/stm32/pwm_motor_2wire.hpp" target="_blank">PwmMotor2Wire</a>

The class can drive a motor that uses two wires for direction and speed control.

## AVR

### <a name="PwmMotor3Wire" href="https://github.com/boltrobotics/devices/tree/master/include/devices/avr/pwm_motor_3wire.hpp" target="_blank">PwmMotor3Wire</a>

The class can drive a motor that uses three wires for direction and speed control.

# <a name="Contribute" href="https://boltrobotics.com/contribute/" target="_blank">Contribute</a>

Consider supporting our projects by contributing to their development.
<a href="https://boltrobotics.com/contribute/" target="_blank">Learn more at boltrobotics.com</a>
