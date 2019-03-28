cmake_minimum_required(VERSION 3.7)
set(CMAKE_MODULE_PATH $ENV{CMAKEHELPERS_HOME}/cmake/Modules)

include(arduino_project)

setup_dep(utility $ENV{UTILITY_HOME} SUB_DIR "src/${BOARD_FAMILY}")

setup_arduino()

find_srcs()
build_lib(SRCS ${SOURCES})
