Provides drivers for USART, USB, 2-3 wire motors, time functions on ESP32 / AVR / STM32 / X86.

<a name="Build"></a>
## Build 

Set up the build following an
<a href="https://github.com/boltrobotics/cmake-helpers/#Example">example</a> from
cmake-helpers project.

<a name="x86"></a>
## x86

<a name="PseudoTTY"></a>
### <a href="include/devices/x86/pseudo_tty.hpp">PseudoTTY</a>

The class creates two serial devices using <a href="https://linux.die.net/man/1/socat">socat</a>
utility. PseudoTTY is useful for testing serial client or server. The unit tests for Usart class
use it for similating serial ports.

<a name="Usart"></a>
### <a href="include/devices/usart.hpp">Usart</a>

The class provides an interface for communication over serial connection. Each read/write is
configured to time out if the operation doesn't complete within a specified window. The code uses
Boost ASIO library.

<a name="usart_test" href="test/usart_test.cpp">usart_test.cpp</a>
contains unit tests for Usart class.

<a name="UsartTermios"></a> 
### <a href="include/devices/x86/usart_termios.hpp">Usart Termios</a>

Class functionality is similar to [Usart](#Usart) but using termios library.

<a name="usart_termios_test" href="test/usart_termios_test.cpp">usart_termios_test.cpp</a>
contains unit tests for Usart class.

<a name="stm32"></a>
## STM32

<a name="Usb"></a>
### <a href="include/devices/stm32/usb.hpp">Usb</a>

The class provides an interface to transfer data over USB connection.

<a name="stm32_Usart"></a>
### <a href="include/devices/stm32/usart.hpp">Usart</a>

The class provides an interface for communication over serial connection.

<a name="PwmMotor3Wire"></a>
### <a href="include/devices/stm32/pwm_motor_3wire.hpp">PwmMotor3Wire</a>

The class can drive a motor that uses three wires for direction and speed control.

<a name="PwmMotor2Wire"></a>
### <a href="include/devices/stm32/pwm_motor_2wire.hpp">PwmMotor2Wire</a>

The class can drive a motor that uses two wires for direction and speed control.

<a name="avr"></a>
## AVR

<a name="PwmMotor3Wire"></a>
### <a href="include/devices/avr/pwm_motor_3wire.hpp">PwmMotor3Wire</a>

The class can drive a motor that uses three wires for direction and speed control.

## Common Code

<a name="MaxSonarLvEx"></a>
### <a href="include/devices/maxsonar_lvez.hpp">MaxSonarLvEx</a>

The class calculates range in millimeters from an ADC sample of MaxSonar ultrasonic range finder.

<a name="WheelEncoder"></a>
### <a href="include/devices/wheel_encoder.hpp">WheelEncoder</a>

The class counts the number of clicks that a virtual wheel moved forward and backward.
