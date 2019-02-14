cmake_minimum_required(VERSION 3.5)
set(CMAKE_MODULE_PATH $ENV{CMAKEHELPERS_HOME}/cmake/Modules)

include(stm32f103c8t6)
include(stm32_project)

include(libopencm3)
include_directories(${libopencm3_INC_DIR})
include(freertos)
include_directories(${FREERTOS_INC_DIRS})

add_definitions(-DBTR_ENABLE_USB=1)

find_srcs()
build_lib(SRCS ${SOURCES})
