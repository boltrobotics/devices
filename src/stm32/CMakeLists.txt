cmake_minimum_required(VERSION 3.16)
project(devices)

include(stm32_project)
setup_stm32()

set(UTILITY_HOME $ENV{UTILITY_HOME})
include_directories(${UTILITY_HOME}/include)
include(libopencm3)
include_directories(${libopencm3_INC_DIR})
include(freertos)
include_directories(${FREERTOS_INC_DIRS})

find_srcs()
build_lib(SRCS ${SOURCES})
