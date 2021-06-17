include(x86_project)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)

setup_dep(utility $ENV{UTILITY_HOME} SUB_DIR "./")

find_srcs(FILTER ${MAIN_SRC})
list(APPEND LIBS ${BTR_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})
build_lib(SRCS ${SOURCES} LIBS ${LIBS} INC_DIRS ${utility_INC_DIR})
