cmake_minimum_required(VERSION 3.16)
project(devices)

if (NOT AVR_MCU)
  set(AVR_MCU atmega2560)
endif ()
if (NOT AVR_UPLOADTOOL_PORT)
  set(AVR_UPLOADTOOL_PORT "/dev/ttyACM0")
endif ()

include(avr_project)
setup_dep(utility $ENV{UTILITY_HOME} SUB_DIR "src/${BOARD_FAMILY}")
setup_avr()
find_srcs()
build_lib(
  SRCS ${SOURCES}
  INC_DIRS ${utility_INC_DIR})
