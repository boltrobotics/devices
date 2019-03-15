set(CMAKE_MACOSX_RPATH 1)

include(x86_project)

####################################################################################################
# utility {

set(UTILITY_HOME $ENV{UTILITY_HOME})
include_directories(${UTILITY_HOME}/include)

if (NOT TARGET utility)
  add_subdirectory(${UTILITY_HOME} ${PROJECT_BINARY_DIR}/utility)
endif ()

# } utility

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)

setup_x86()
find_srcs(FILTER ${MAIN_SRC})
list(APPEND LIBS ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})
build_lib(SRCS ${SOURCES} LIBS ${LIBS})
