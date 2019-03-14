cmake_minimum_required(VERSION 3.7)
set(CMAKE_MODULE_PATH $ENV{CMAKEHELPERS_HOME}/cmake/Modules)

include(avr_project)

####################################################################################################
# utility {

set(UTILITY_HOME $ENV{UTILITY_HOME})
include_directories(${UTILITY_HOME}/include)

if (NOT TARGET utility)
  add_subdirectory(${UTILITY_HOME} ${PROJECT_BINARY_DIR}/utility)
endif ()

# } utility

setup_avr()
find_srcs()
build_lib(SRCS ${SOURCES})
