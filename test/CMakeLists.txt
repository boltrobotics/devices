####################################################################################################
# utility {

set(UTILITY_HOME $ENV{UTILITY_HOME})
include_directories(${UTILITY_HOME}/include)

if (NOT TARGET utility)
  add_subdirectory(${UTILITY_HOME} ${PROJECT_BINARY_DIR}/utility)
endif ()

# }

####################################################################################################
# devices test {

include(test_project)
find_test_srcs()
build_exe(SRCS ${SOURCES} LIBS ${PROJECT_NAME} utility SUFFIX "-tests")

# } devices test
