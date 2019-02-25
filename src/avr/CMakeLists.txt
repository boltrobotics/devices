cmake_minimum_required(VERSION 3.7)
set(CMAKE_MODULE_PATH $ENV{CMAKEHELPERS_HOME}/cmake/Modules)

include(avr_project)

# Remember to enable features in parent cmake, e.g.:
#add_definitions(-DBTR_USART1_ENABLED=1)
#set(AVR_UPLOADTOOL_PORT /dev/tty.usbmodemFD14511)

setup()
find_srcs()
build_lib(SRCS ${SOURCES})
