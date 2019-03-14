set(CMAKE_MACOSX_RPATH 1)

include(x86_project)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)

setup_x86()
find_srcs(FILTER ${MAIN_SRC})
list(APPEND LIBS ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})
build_lib(SRCS ${SOURCES} LIBS ${LIBS})
