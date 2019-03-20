cmake_minimum_required(VERSION 3.5)
set(CMAKE_MODULE_PATH $ENV{CMAKEHELPERS_HOME}/cmake/Modules)

include(arduino_project)

####################################################################################################
# utility {

function(add_utility)
  set(UTILITY_HOME $ENV{UTILITY_HOME})
  include_directories(${UTILITY_HOME}/include)

  if (NOT TARGET utility)
    set(ROOT_SOURCE_DIR ${UTILITY_HOME})
    set(SUBPROJECT_NAME utility)
    add_subdirectory(${UTILITY_HOME}/src/${BOARD_FAMILY} ${PROJECT_BINARY_DIR}/utility)
  endif ()
endfunction()

add_utility()

# } utility

setup_arduino()
find_srcs()
build_lib(SRCS ${SOURCES})
